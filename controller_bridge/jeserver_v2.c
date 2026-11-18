/* Original JEServer_v2 controller engine. SDK-independent; no clear/recover,
 * power, legacy gripper initialization, SDK patch or physical opt-in here. */
#include "jeserver_v2.h"
#include <math.h>
#include <string.h>

static int tracked_start(void *p, double q, double r)
{
    struct jeserver_v2 *v = p;
    v->had_writer = true; v->stop_issued = v->end_issued = false;
    return v->raw_io.start(v->raw_io.context, q, r);
}
static int tracked_joint(void *p, const double *q, double t)
{ struct jeserver_v2 *v = p; return v->raw_io.joint(v->raw_io.context, q, t); }
static int tracked_stop(void *p)
{
    struct jeserver_v2 *v = p;
    if (v->stop_issued) return 0;
    v->stop_issued = true;
    int result = v->raw_io.stop(v->raw_io.context);
    if (result && !v->cleanup_error) v->cleanup_error = result;
    return result;
}
static int tracked_end(void *p)
{
    struct jeserver_v2 *v = p;
    if (v->end_issued) return 0;
    v->end_issued = true;
    int result = v->raw_io.end(v->raw_io.context);
    if (result && !v->cleanup_error) v->cleanup_error = result;
    return result;
}
static int tracked_stop_run(void *p)
{ struct jeserver_v2 *v = p; return v->raw_io.stop_run(v->raw_io.context); }
static int stop_jog(struct jeserver_v2 *v)
{
    if (v->jog_stop_issued) return 0;
    v->jog_stop_issued = true;
    int result = v->teach.jog_stop(v->teach.context);
    if (result && !v->cleanup_error) v->cleanup_error = result;
    return result;
}
static void fault(struct jeserver_v2 *v, enum mk2_fault reason)
{
    v->pending = false;
    v->idle_qualified = false;
    if (v->jog_live) {
        (void)stop_jog(v);
        (void)tracked_stop(v); /* Genuine current-owner fault, never stale-PC cleanup. */
    }
    /* If End returned but the writer has not exited, this engine still has
     * cleanup responsibility. Never forward a jog while in that condition. */
    if (v->had_writer) {
        int stop = tracked_stop(v);
        int end = tracked_end(v);
        if (!v->core.sdk_error) v->core.sdk_error = stop ? stop : end;
    }
    if (!v->core.sdk_error && v->cleanup_error) v->core.sdk_error = v->cleanup_error;
    if (!v->core.fault) v->core.fault = v->core.sdk_error ? MK2_SDK_ERROR : reason;
    v->core.owns_servo = false;
    v->core.phase = MK2_FAULT;
    v->owner = JE2_FAULT;
}
static bool clock_ok(struct jeserver_v2 *v, double now)
{
    if (!isfinite(now) || now < v->last_now) { fault(v, MK2_CLOCK_ERROR); return false; }
    v->last_now = now;
    return true;
}
void je2_abort(struct jeserver_v2 *v, enum mk2_fault reason)
{
    fault(v, reason == MK2_NO_FAULT ? MK2_STATE_ERROR : reason);
}
static bool fresh_healthy(const struct jeserver_v2 *v, double now, const struct je2_feedback *f)
{
    if (!f || !f->sample.valid || !isfinite(f->acquired_at) || f->acquired_at > now ||
        now - f->acquired_at > v->core.policy.watchdog || f->sample.power != 1 ||
        f->sample.driver_error || f->sample.move_error || f->sample.move_state < 0 || f->sample.move_state == 7) return false;
    for (int i = 0; i < MK2_AXES; ++i)
        if (!isfinite(f->sample.joint[i]) || f->sample.joint[i] < v->core.policy.lower[i] ||
            f->sample.joint[i] > v->core.policy.upper[i]) return false;
    return true;
}
static bool observe_healthy(struct jeserver_v2 *v, double now, const struct je2_feedback *f)
{
    if (fresh_healthy(v, now, f)) return true;
    /* Preserve an error seen by ANY eligible executor entry, even if another
     * read is healthy before tick. A busy jog/claim refusal must not discard
     * that evidence. Retired PC requests are fenced before reaching here.
     * Never-engaged startup can remain disabled without latching a fault;
     * AVAILABLE after a handover is still part of the engaged lifecycle.
     * This remembers observations, NOT vendor stop provenance or a guarantee
     * against an external stop which none of our reads has observed. */
    if (v->owner != JE2_AVAILABLE || !v->legacy_open) fault(v, MK2_STATE_ERROR);
    return false;
}
static bool retire(struct jeserver_v2 *v)
{
    v->legacy_open = false;
    if (v->generation == UINT64_MAX) { fault(v, MK2_CLOCK_ERROR); return false; }
    ++v->generation;
    return true;
}
static bool stationary(struct jeserver_v2 *v, double now, const struct je2_feedback *f)
{
    (void)now;
    bool stable = f->sample.move_state == 0 && v->stable_at >= 0;
    for (int i = 0; stable && i < MK2_AXES; ++i)
        if (fabs(f->sample.joint[i] - v->stable_joint[i]) > 1e-5) stable = false;
    if (!stable) {
        memcpy(v->stable_joint, f->sample.joint, sizeof(v->stable_joint));
        v->stable_at = f->sample.move_state == 0 ? f->acquired_at : -1;
        v->idle_qualified = false;
        return false;
    }
    v->idle_qualified = f->acquired_at - v->stable_at >= 0.1;
    return v->idle_qualified;
}
int je2_init(struct jeserver_v2 *v, const struct mk2_policy *p, const struct mk2_io *io, const struct je2_io *teach)
{
    if (!v || !io || !teach || !teach->writer || !teach->jog || !teach->jog_stop) return -1;
    memset(v, 0, sizeof(*v));
    if (mk2_core_init(&v->core, p, io)) return -1;
    v->raw_io = *io;
    v->core.io = (struct mk2_io){.context = v, .start = tracked_start, .joint = tracked_joint,
        .stop = tracked_stop, .end = tracked_end, .stop_run = io->stop_run ? tracked_stop_run : NULL};
    v->teach = *teach;
    v->generation = 1; v->legacy_open = true; v->owner = JE2_AVAILABLE;
    v->stable_at = v->last_feedback_at = -1;
    return 0;
}
int je2_pc_frame(struct jeserver_v2 *v, uint64_t generation, const char *frame, size_t length,
                 double now, const struct je2_feedback *f)
{
    /* Fencing precedes core_handle/tick: a retired PC's malformed/late frame
     * cannot invoke core reject()->Stop/End against the new pendant owner. */
    if ((v->owner != JE2_AVAILABLE && v->owner != JE2_PC) ||
        !(generation == v->generation || (generation == 0 && v->legacy_open))) {
        ++v->dropped_pc; return -1;
    }
    if (!clock_ok(v, now)) return -1;
    if (!observe_healthy(v, now, f)) return -1;
    if (f->acquired_at <= v->last_feedback_at) return -1;
    v->last_feedback_at = f->acquired_at;
    const bool owned = v->core.owns_servo;
    const enum mk2_phase before = v->core.phase;
    int result = mk2_core_handle(&v->core, frame, length, now, &f->sample);
    if (v->core.phase == MK2_FAULT || v->core.phase == MK2_STOPPED) {
        fault(v, v->core.fault ? v->core.fault : MK2_NOT_READY); return result;
    }
    if (v->core.phase == MK2_RELEASING || (result == 0 && before == MK2_WAIT_FIRST && v->core.phase == MK2_READY)) {
        v->had_writer = v->had_writer || owned;
        v->release_at = v->barrier_after = now; v->stable_at = -1; v->idle_qualified = false;
        if (!retire(v)) return -1;
        v->owner = v->core.phase == MK2_RELEASING ? JE2_YIELDING : JE2_EXIT_BARRIER;
    } else if (v->core.phase == MK2_WAIT_FIRST || v->core.phase == MK2_ACTIVE) v->owner = JE2_PC;
    return result;
}
int je2_pendant_jog(struct jeserver_v2 *v, int axis, int direction, double received,
                     double now, const struct je2_feedback *f)
{
    if (!clock_ok(v, now)) return -1;
    if (!v->core.policy.allow_commands || axis < 0 || axis >= MK2_AXES || (direction != -1 && direction != 1) ||
        !isfinite(received) || received < 0 || received > now || now - received > 0.5 ||
        v->owner == JE2_FAULT) return -1;
    if (!observe_healthy(v, now, f)) return -1;
    if (v->pending || v->jog_live || v->owner == JE2_YIELDING || v->owner == JE2_EXIT_BARRIER) return -1;
    /* Deny before touching any SDK callback if clean handover is unavailable. */
    if ((v->core.owns_servo || v->core.phase == MK2_WAIT_FIRST) && !v->core.io.stop_run) return -1;
    if (f->acquired_at <= v->last_feedback_at) return -1;
    v->last_feedback_at = f->acquired_at;
    v->axis = axis; v->direction = direction; v->request_at = received;
    v->pending = true; v->had_writer = v->core.owns_servo; v->release_at = now;
    v->stable_at = -1; v->idle_qualified = false; v->barrier_after = now;
    if (!retire(v)) return -1;
    if (v->core.owns_servo || v->core.phase == MK2_WAIT_FIRST) {
        static const char release[] = "Release {\"Release\":true}";
        if (mk2_core_handle(&v->core, release, sizeof(release) - 1, now, &f->sample)) {
            fault(v, v->core.fault ? v->core.fault : MK2_STATE_ERROR); return -1;
        }
        v->owner = v->core.phase == MK2_RELEASING ? JE2_YIELDING : JE2_EXIT_BARRIER;
    } else v->owner = JE2_EXIT_BARRIER;
    return 0; /* Accepted intent, NOT motion success. */
}
int je2_pendant_stop(struct jeserver_v2 *v, double now)
{
    if (!clock_ok(v, now)) return -1;
    v->pending = false;
    if (!v->jog_live) return 0; /* Release during yield cancels; no jog replay. */
    int result = stop_jog(v);
    if (result) { if (!v->core.sdk_error) v->core.sdk_error = result; fault(v, MK2_SDK_ERROR); }
    /* Keep jog_live until qualified idle feedback: no premature PC claim. */
    return result;
}
void je2_tick(struct jeserver_v2 *v, double now, const struct je2_feedback *f)
{
    if (v->owner == JE2_FAULT || !clock_ok(v, now)) return;
    if (!observe_healthy(v, now, f)) return;
    if (v->owner == JE2_AVAILABLE) return;
    if (f->acquired_at < v->last_feedback_at) { fault(v, MK2_CLOCK_ERROR); return; }
    if ((v->owner == JE2_YIELDING || v->owner == JE2_EXIT_BARRIER) &&
        now - v->release_at >= v->core.policy.first_timeout) { fault(v, MK2_WATCHDOG); return; }
    if (f->acquired_at == v->last_feedback_at) {
        if (v->owner == JE2_PC && now - v->core.last_receive >
            (v->core.phase == MK2_WAIT_FIRST ? v->core.policy.first_timeout : v->core.policy.watchdog))
            fault(v, MK2_WATCHDOG);
        return;
    }
    v->last_feedback_at = f->acquired_at;
    if (v->owner == JE2_PENDANT) {
        /* Idle feedback alone cannot revoke a continuous jog which may not
         * have started executing yet. Retain responsibility until STOP. */
        if (stationary(v, now, f) && v->jog_stop_issued) v->jog_live = false;
        return;
    }
    if (v->owner == JE2_PC || v->owner == JE2_YIELDING) {
        if (v->owner == JE2_YIELDING && f->acquired_at < v->core.last_receive) return;
        mk2_core_tick(&v->core, v->owner == JE2_YIELDING ? f->acquired_at : now, &f->sample);
        if (v->core.phase == MK2_FAULT) { fault(v, v->core.fault); return; }
        if (v->owner == JE2_PC) return;
        if (v->core.phase == MK2_READY) {
            v->owner = JE2_EXIT_BARRIER; v->barrier_after = now;
            v->stable_at = -1; v->idle_qualified = false;
        }
    }
    if (v->pending && now - v->request_at > 0.5) v->pending = false;
    if (now - v->release_at >= v->core.policy.first_timeout) { fault(v, MK2_WATCHDOG); return; }
    if (v->owner != JE2_EXIT_BARRIER) return;
    if (v->had_writer) {
        enum je2_writer writer = v->teach.writer(v->teach.context);
        if (writer == JE2_WRITER_UNKNOWN) { fault(v, MK2_STATE_ERROR); return; }
        if (writer != JE2_WRITER_EXITED) return;
        v->had_writer = false;
        v->barrier_after = now; v->stable_at = -1; v->idle_qualified = false;
        return; /* Never reuse a pre-exit sample as post-exit evidence. */
    }
    if (f->acquired_at <= v->barrier_after) return;
    if (!stationary(v, now, f)) return;
    v->owner = JE2_PENDANT;
    if (v->pending) {
        v->pending = false;
        v->stable_at = -1;
        v->idle_qualified = false; v->jog_stop_issued = false;
        /* Failed jog can have partial effects: retain stop responsibility. */
        v->jog_live = true;
        int result = v->teach.jog(v->teach.context, v->axis, v->direction);
        if (result) { v->core.sdk_error = result; fault(v, MK2_SDK_ERROR); }
    }
}
int je2_claim_pc(struct jeserver_v2 *v, uint64_t observed, double now, const struct je2_feedback *f)
{
    if (!v->core.policy.allow_commands || observed != v->generation ||
        (v->owner != JE2_PENDANT && v->owner != JE2_AVAILABLE) ||
        v->core.fault != MK2_NO_FAULT || v->core.sdk_error) return -1;
    if (!clock_ok(v, now)) return -1;
    if (!observe_healthy(v, now, f)) return -1;
    if (v->pending || v->jog_live || v->had_writer || f->acquired_at <= v->last_feedback_at) return -1;
    v->last_feedback_at = f->acquired_at;
    if (!stationary(v, now, f)) return -1;
    if (!retire(v)) return -1;
    struct mk2_policy p = v->core.policy;
    struct mk2_io io = v->core.io;
    if (mk2_core_init(&v->core, &p, &io)) { v->owner = JE2_FAULT; return -1; }
    v->owner = JE2_AVAILABLE;
    return 0; /* New Switch + fresh hold still required; no SDK start here. */
}
int je2_decode_button(const char *text, size_t size, int *axis, int *direction)
{
    if (!text || !axis || !direction || size < 4 || size > 6 || text[0] != 't' || text[1] != 'v' || text[size - 1] != '!') return -1;
    unsigned value = 0;
    if (size > 4 && text[2] == '0') return -1;
    for (size_t i = 2; i + 1 < size; ++i) {
        if (text[i] < '0' || text[i] > '9') return -1;
        value = value * 10 + (unsigned)(text[i] - '0');
    }
    if (value == 33) return 2;
    static const unsigned negative[] = {113, 119, 101, 114, 116, 121, 117};
    if (value >= 49 && value <= 55) { *axis = (int)value - 49; *direction = 1; return 1; }
    for (int i = 0; i < MK2_AXES; ++i)
        if (value == negative[i]) { *axis = i; *direction = -1; return 1; }
    return 0;
}