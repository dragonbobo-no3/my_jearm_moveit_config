#include "../jeserver_v2.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

struct fixture {
    struct jeserver_v2 v;
    struct mk2_sample sample;
    enum je2_writer writer;
    int start, joint, stop, end, stop_run, jog, jog_stop;
    int start_result, end_result, stop_run_result, jog_result, jog_stop_result;
    int jog_stop_state, implicit_jog_recover;
    bool model_implicit_jog_recover;
};
static int start(void *p, double q, double r)
{
    struct fixture *f = p; (void)q; (void)r;
    ++f->start; f->writer = JE2_WRITER_RUNNING; f->sample.move_state = 13;
    return f->start_result;
}
static int joint(void *p, const double *q, double t)
{ struct fixture *f = p; (void)q; (void)t; ++f->joint; return 0; }
static int stop(void *p)
{ struct fixture *f = p; ++f->stop; f->sample.move_state = 0; return 0; }
static int end(void *p)
{ struct fixture *f = p; ++f->end; f->sample.move_state = 0; return f->end_result; }
static int stop_run(void *p)
{ struct fixture *f = p; ++f->stop_run; f->sample.move_state = 0; return f->stop_run_result; }
static enum je2_writer writer(void *p) { return ((struct fixture *)p)->writer; }
static int jog(void *p, int axis, int dir)
{
    struct fixture *f = p; assert(axis == 0 && dir == 1);
    /* Fixed SDK robot_teach_joint has an internal recovery call. This is a
     * synthetic side-effect trap, not an emulation or execution of the SDK. */
    if (f->model_implicit_jog_recover) {
        ++f->implicit_jog_recover;
        f->sample.move_state = 0;
    }
    ++f->jog; f->sample.move_state = 1; return f->jog_result;
}
static int jog_stop(void *p)
{ struct fixture *f = p; ++f->jog_stop; f->sample.move_state = f->jog_stop_state; return f->jog_stop_result; }
static struct je2_feedback feedback(struct fixture *f, double now)
{ return (struct je2_feedback){.sample = f->sample, .acquired_at = now}; }
static void init(struct fixture *f, bool handover)
{
    memset(f, 0, sizeof(*f));
    f->sample.valid = true; f->sample.power = 1; f->writer = JE2_WRITER_EXITED;
    struct mk2_policy p = {.allow_commands = true, .watchdog = .1, .first_timeout = 1,
        .min_dt = .005, .max_dt = .05, .clock_skew = .1, .first_hold_tolerance = .001,
        .following_tolerance = .02, .servo_q = 1, .servo_r = .0001};
    for (int i = 0; i < MK2_AXES; ++i) {
        p.lower[i] = -1; p.upper[i] = 1; p.velocity[i] = .2; p.acceleration[i] = .4;
    }
    struct mk2_io io = {.context = f, .start = start, .joint = joint, .stop = stop,
        .end = end, .stop_run = handover ? stop_run : NULL};
    struct je2_io teach = {.context = f, .writer = writer, .jog = jog, .jog_stop = jog_stop};
    assert(je2_init(&f->v, &p, &io, &teach) == 0);
}
static int pc(struct fixture *f, uint64_t generation, const char *text, double now)
{
    struct je2_feedback sample = feedback(f, now);
    return je2_pc_frame(&f->v, generation, text, strlen(text), now, &sample);
}
static void tick(struct fixture *f, double now)
{ struct je2_feedback sample = feedback(f, now); je2_tick(&f->v, now, &sample); }
static int press(struct fixture *f, double now)
{ struct je2_feedback sample = feedback(f, now); return je2_pendant_jog(&f->v, 0, 1, now, now, &sample); }
static void active(struct fixture *f)
{
    assert(pc(f, 0, "Switch {\"Switch\":true}", .01) == 0);
    assert(pc(f, 0, "Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0,0,0,0,0,0,0]}}", .02) == 0);
    assert(f->start == 1 && f->joint == 1 && f->v.owner == JE2_PC);
}
static void release_to_barrier(struct fixture *f)
{
    assert(press(f, .03) == 0);
    assert(f->v.owner == JE2_YIELDING && f->stop_run == 1);
    tick(f, .04); tick(f, .15);
    assert(f->end == 1 && f->v.owner == JE2_EXIT_BARRIER && !f->jog);
}
static void complete(struct fixture *f)
{
    f->writer = JE2_WRITER_EXITED;
    tick(f, .16); tick(f, .17); tick(f, .28);
    assert(f->jog == 1 && f->v.owner == JE2_PENDANT);
}
static void auto_yield_and_fence(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f);
    const char *late[] = {"Switch {\"Switch\":true}", "Switch {\"Switch\":false}",
        "Release {\"Release\":true}", "Joint garbage", "bad frame"};
    for (size_t i = 0; i < sizeof(late) / sizeof(late[0]); ++i)
        assert(pc(&f, 0, late[i], .151) == -1);
    assert(f.end == 1 && !f.stop && f.start == 1);
    complete(&f);
    for (size_t i = 0; i < sizeof(late) / sizeof(late[0]); ++i)
        assert(pc(&f, 1, late[i], .28) == -1);
    assert(f.jog == 1 && !f.stop && f.end == 1 && f.v.owner == JE2_PENDANT);
}
static void cancelled_and_expired(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f);
    assert(je2_pendant_stop(&f.v, .155) == 0);
    f.writer = JE2_WRITER_EXITED; tick(&f, .16); tick(&f, .17); tick(&f, .28);
    assert(!f.jog && f.v.owner == JE2_PENDANT && !f.jog_stop);
    init(&f, true); active(&f); release_to_barrier(&f);
    f.writer = JE2_WRITER_EXITED; tick(&f, .55); tick(&f, .56); tick(&f, .67);
    assert(!f.jog && f.v.owner == JE2_PENDANT);
}
static void writer_unknown_running_timeout(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f);
    tick(&f, .3); assert(!f.jog && f.v.owner == JE2_EXIT_BARRIER);
    f.writer = JE2_WRITER_UNKNOWN; tick(&f, .31);
    assert(f.v.owner == JE2_FAULT && !f.jog);
    init(&f, true); active(&f); release_to_barrier(&f);
    tick(&f, 1.04); assert(f.v.owner == JE2_FAULT && !f.jog);
}
static void new_session_only(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f); complete(&f);
    struct je2_feedback sample = feedback(&f, .28);
    assert(je2_claim_pc(&f.v, f.v.generation, .28, &sample) == -1);
    assert(je2_pendant_stop(&f.v, .29) == 0 && f.jog_stop == 1);
    tick(&f, .3); tick(&f, .41);
    sample = feedback(&f, .42);
    uint64_t old = f.v.generation;
    assert(je2_claim_pc(&f.v, old, .42, &sample) == 0);
    assert(f.v.generation != old && !f.v.legacy_open);
    assert(pc(&f, 0, "Switch {\"Switch\":true}", .43) == -1);
    assert(pc(&f, old, "Switch {\"Switch\":false}", .43) == -1);
    assert(pc(&f, f.v.generation, "Switch {\"Switch\":true}", .43) == 0);
    assert(f.start == 1); /* Claim and Switch never automatically start. */
}
static void errors_and_stale_feedback(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f);
    f.sample.driver_error = 1; tick(&f, .16); assert(f.v.owner == JE2_FAULT && !f.jog);
    f.sample.driver_error = 0;
    assert(press(&f, .17) == -1 && f.v.owner == JE2_FAULT);
    init(&f, true); active(&f);
    struct je2_feedback stale = feedback(&f, .0);
    je2_tick(&f.v, .2, &stale); assert(f.v.owner == JE2_FAULT);
    init(&f, true); active(&f); f.stop_run_result = -4;
    assert(press(&f, .03) == -1 && f.v.owner == JE2_FAULT && !f.jog);
    init(&f, true); active(&f); release_to_barrier(&f); f.jog_result = 13;
    f.writer = JE2_WRITER_EXITED; tick(&f, .16); tick(&f, .17); tick(&f, .28);
    assert(f.v.owner == JE2_FAULT && f.jog == 1 && f.jog_stop == 1);
}
static void guards_and_read_only_default(void)
{
    struct fixture f; init(&f, false); active(&f);
    assert(press(&f, .03) == -1 && f.v.owner == JE2_PC && !f.stop_run && !f.end);
    init(&f, true); active(&f);
    assert(pc(&f, 0, "Joint {\"Robot0\":{\"time\":0.02,\"joint\":[2,0,0,0,0,0,0]}}", .03) == -1);
    assert(f.v.owner == JE2_FAULT && f.stop == 1 && f.end == 1);
    init(&f, true); active(&f); tick(&f, .2);
    assert(f.v.owner == JE2_FAULT && f.v.core.fault == MK2_WATCHDOG);
    init(&f, true); active(&f); tick(&f, NAN); assert(f.v.owner == JE2_FAULT);
}
static void strict_buttons(void)
{
    int axis = -1, dir = 0;
    assert(je2_decode_button("tv49!", 5, &axis, &dir) == 1 && axis == 0 && dir == 1);
    assert(je2_decode_button("tv117!", 6, &axis, &dir) == 1 && axis == 6 && dir == -1);
    assert(je2_decode_button("tv33!", 5, &axis, &dir) == 2);
    assert(je2_decode_button("tv48!", 5, &axis, &dir) == 0);
    const char *invalid[] = {"tv49", "tv049!", "tv+49!", "tv49!tv33!", "tv9999!", "tvx!", "\ntv49!"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
        assert(je2_decode_button(invalid[i], strlen(invalid[i]), &axis, &dir) == -1);
}
static void claim_fenced_before_clock(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f); complete(&f);
    struct je2_feedback sample = feedback(&f, .29);
    assert(je2_claim_pc(&f.v, 1, NAN, &sample) == -1);
    assert(je2_claim_pc(&f.v, 1, 1000000, &sample) == -1);
    assert(f.v.last_now == .28 && f.v.owner == JE2_PENDANT && !f.jog_stop);
    assert(je2_pendant_stop(&f.v, .29) == 0);
}
static void duplicate_feedback_no_barrier_credit(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f);
    f.writer = JE2_WRITER_EXITED; tick(&f, .16);
    struct je2_feedback same = feedback(&f, .17);
    je2_tick(&f.v, .17, &same);
    je2_tick(&f.v, .20, &same); je2_tick(&f.v, .24, &same);
    assert(!f.jog && f.v.owner == JE2_EXIT_BARRIER && f.v.stable_at == .17);
    tick(&f, .28); assert(f.jog == 1);
}
static void first_intent_release_and_hard_stop(void)
{
    struct fixture f; init(&f, true);
    assert(pc(&f, 0, "Switch {\"Switch\":true}", .01) == 0);
    assert(pc(&f, 0, "Release {\"Release\":true}", .02) == 0);
    assert(!f.v.legacy_open && f.v.owner == JE2_EXIT_BARRIER && !f.start && !f.end);
    tick(&f, .03); tick(&f, .14);
    assert(f.v.owner == JE2_PENDANT && !f.jog);
    assert(pc(&f, 0, "Switch {\"Switch\":true}", .15) == -1);
    init(&f, true); active(&f);
    assert(pc(&f, 0, "Switch {\"Switch\":false}", .03) == 0);
    assert(f.v.owner == JE2_FAULT && f.stop == 1 && f.end == 1 && f.v.had_writer);
    assert(press(&f, .04) == -1);
    struct je2_feedback sample = feedback(&f, .05);
    assert(je2_claim_pc(&f.v, f.v.generation, .05, &sample) == -1);
}
static void cleanup_once_and_first_fault(void)
{
    struct fixture f; init(&f, true); active(&f);
    assert(press(&f, .03) == 0); f.sample.driver_error = 2; tick(&f, .04);
    assert(f.v.owner == JE2_FAULT && f.stop == 1 && f.end == 1 && f.v.had_writer);
    init(&f, true); active(&f); release_to_barrier(&f); complete(&f);
    f.jog_stop_result = -51;
    assert(je2_pendant_stop(&f.v, .29) == -51);
    assert(f.v.owner == JE2_FAULT && f.jog_stop == 1 && f.v.jog_live);
    assert(f.v.core.sdk_error == -51 && f.v.cleanup_error == -51);
    init(&f, true); f.start_result = -8;
    assert(pc(&f, 0, "Switch {\"Switch\":true}", .01) == 0);
    assert(pc(&f, 0, "Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0,0,0,0,0,0,0]}}", .02) == -1);
    assert(f.v.owner == JE2_FAULT && f.v.had_writer && f.stop == 1 && f.end == 1 && !f.jog);
    init(&f, true); active(&f); assert(press(&f, .03) == 0);
    tick(&f, .04); f.end_result = -73; tick(&f, .15);
    assert(f.v.owner == JE2_FAULT && f.end == 1 && f.v.cleanup_error == -73 && !f.jog);
    init(&f, true); active(&f); release_to_barrier(&f);
    struct je2_feedback reversed = feedback(&f, .14);
    je2_tick(&f.v, .16, &reversed);
    assert(f.v.owner == JE2_FAULT && f.v.core.fault == MK2_CLOCK_ERROR && !f.jog);
    init(&f, true); active(&f); release_to_barrier(&f); complete(&f);
    f.sample.driver_error = 3; tick(&f, .29);
    assert(f.v.owner == JE2_FAULT && f.stop == 1 && f.jog_stop == 1);
}
static void idle_qualification_revoked_and_commands_locked(void)
{
    struct fixture f; init(&f, true); active(&f); release_to_barrier(&f); complete(&f);
    assert(je2_pendant_stop(&f.v, .29) == 0); tick(&f, .3); tick(&f, .41);
    f.sample.move_state = 1; tick(&f, .42); assert(!f.v.idle_qualified);
    f.sample.move_state = 0;
    struct je2_feedback sample = feedback(&f, .43);
    assert(je2_claim_pc(&f.v, f.v.generation, .43, &sample) == -1);
    init(&f, true); f.v.core.policy.allow_commands = false;
    sample = feedback(&f, .1);
    assert(je2_claim_pc(&f.v, f.v.generation, .1, &sample) == -1);
    assert(press(&f, .2) == -1 && !f.start && !f.jog);
}
static void stopped_state_cannot_reach_implicit_recovery(void)
{
    /* Identical state=7/errors=0 observations with and without our STOP.
     * A pending STOP is not evidence that an external stop did not occur. */
    for (int requested_stop = 0; requested_stop <= 1; ++requested_stop) {
        struct fixture f; init(&f, true);
        f.model_implicit_jog_recover = true;
        f.jog_stop_state = 7;
        active(&f); release_to_barrier(&f); complete(&f);
        assert(f.implicit_jog_recover == 1);
        const uint64_t generation = f.v.generation;
        if (requested_stop) assert(je2_pendant_stop(&f.v, .29) == 0);
        f.sample.move_state = 7;
        assert(f.sample.power == 1 && !f.sample.driver_error && !f.sample.move_error);
        /* Even a new button arriving before tick must not call the SDK. */
        assert(press(&f, .295) == -1 && f.implicit_jog_recover == 1);
        tick(&f, .3);
        assert(f.v.owner == JE2_FAULT && f.v.core.fault == MK2_STATE_ERROR);
        assert(!f.v.core.sdk_error && !f.v.pending && !f.v.idle_qualified);
        assert(f.jog_stop == 1 && f.stop == 1 && f.end == 1 && f.stop_run == 1);

        /* Later healthy feedback (including cleanup's synthetic zero) cannot
         * clear the latch, start a new jog or mint a new PC generation. */
        f.sample.move_state = 0;
        tick(&f, .31); tick(&f, .42);
        assert(press(&f, .43) == -1);
        struct je2_feedback sample = feedback(&f, .44);
        assert(je2_claim_pc(&f.v, generation, .44, &sample) == -1);
        assert(pc(&f, 0, "Switch {\"Switch\":true}", .45) == -1);
        assert(pc(&f, generation - 1, "Switch {\"Switch\":true}", .455) == -1);
        assert(pc(&f, generation, "Switch {\"Switch\":true}", .46) == -1);
        assert(je2_pendant_stop(&f.v, .47) == 0);
        je2_abort(&f.v, MK2_UNLOAD);
        assert(f.v.owner == JE2_FAULT && f.v.core.fault == MK2_STATE_ERROR);
        assert(f.v.generation == generation && f.implicit_jog_recover == 1);
        assert(f.start == 1 && f.joint == 1 && f.jog == 1);
        assert(f.jog_stop == 1 && f.stop == 1 && f.end == 1 && f.stop_run == 1);
    }
    puts("PASS: requested/unrequested state7 rejects implicit jog recovery, healthy-frame rearm and PC reclaim; fake SDK only");
}
static void transient_feedback_failure_is_latched_at_entry(void)
{
    unsigned cases = 0;
    /* A trusted feedback read may see an error that disappears BEFORE tick.
     * Include busy requests: denying a duplicate jog is not enough to retain
     * the observed error. No SDK emulation or vendor event provenance here. */
    for (int stage = 0; stage < 5; ++stage) {
        for (int entry = 0; entry < 4; ++entry) {
            if (entry == 1 && stage < 2) continue; /* Claim owner gate. */
            if (entry == 2 && stage != 0 && stage != 4) continue; /* PC fence. */
            for (int error = 0; error < 7; ++error) {
                struct fixture f; init(&f, true); active(&f);
                if (stage >= 1) release_to_barrier(&f);
                if (stage >= 2) complete(&f);
                if (stage >= 3) {
                    assert(je2_pendant_stop(&f.v, .29) == 0);
                    tick(&f, .3); tick(&f, .41);
                }
                if (stage == 4) {
                    struct je2_feedback sample = feedback(&f, .42);
                    assert(je2_claim_pc(&f.v, f.v.generation, .42, &sample) == 0);
                    assert(f.v.owner == JE2_AVAILABLE && !f.v.legacy_open);
                }
                const double at[] = {.025, .155, .285, .415, .425};
                double now = at[stage];
                const uint64_t generation = f.v.generation;
                switch (error) {
                case 0: f.sample.move_state = 7; break;
                case 1: f.sample.move_state = -1; break;
                case 2: f.sample.driver_error = 2; break;
                case 3: f.sample.move_error = 3; break;
                case 4: f.sample.power = 0; break;
                case 5: f.sample.valid = false; break;
                default: f.sample.joint[0] = NAN; break;
                }
                struct je2_feedback bad = feedback(&f, now);
                if (entry == 0) assert(press(&f, now) == -1);
                else if (entry == 1) assert(je2_claim_pc(&f.v, generation, now, &bad) == -1);
                else if (entry == 2) assert(pc(&f, generation, "Switch {\"Switch\":true}", now) == -1);
                else je2_tick(&f.v, now, &bad);
                assert(f.v.owner == JE2_FAULT && f.v.core.fault == MK2_STATE_ERROR);
                assert(!f.v.pending && !f.v.idle_qualified);
                const int stops = f.stop, ends = f.end, jog_stops = f.jog_stop;
                const int jogs = f.jog, stop_runs = f.stop_run;
                /* Simulate a later healthy sample without a tick ever seeing
                 * the error. It must not erase this executor's observation. */
                f.sample = (struct mk2_sample){.valid = true, .power = 1};
                tick(&f, now + .01); tick(&f, now + .12);
                assert(press(&f, now + .13) == -1);
                struct je2_feedback healthy = feedback(&f, now + .14);
                assert(je2_claim_pc(&f.v, generation, now + .14, &healthy) == -1);
                assert(pc(&f, generation, "Switch {\"Switch\":true}", now + .15) == -1);
                assert(je2_pendant_stop(&f.v, now + .16) == 0);
                je2_abort(&f.v, MK2_UNLOAD);
                assert(f.v.owner == JE2_FAULT && f.v.core.fault == MK2_STATE_ERROR);
                assert(f.v.generation == generation && f.start == 1 && f.joint == 1);
                assert(f.jog == jogs && f.stop_run == stop_runs);
                assert(f.stop == stops && f.end == ends && f.jog_stop == jog_stops);
                ++cases;
            }
        }
    }
    assert(cases == 105);
    puts("PASS: 105 transient feedback/entry/owner interleavings latch before tick; fake SDK only");
}
static void transient_feedback_preserves_startup_and_fencing(void)
{
    struct fixture f; init(&f, true);
    f.sample.power = 0;
    tick(&f, .01);
    assert(press(&f, .02) == -1);
    assert(pc(&f, 0, "Switch {\"Switch\":true}", .03) == -1);
    struct je2_feedback sample = feedback(&f, .04);
    assert(je2_claim_pc(&f.v, f.v.generation, .04, &sample) == -1);
    assert(f.v.owner == JE2_AVAILABLE && !f.v.core.fault && !f.start && !f.stop);
    /* A never-engaged disabled startup remains a readiness refusal. */
    f.sample.power = 1;
    assert(pc(&f, 0, "Switch {\"Switch\":true}", .05) == 0 && !f.start);

    init(&f, true); active(&f); release_to_barrier(&f); complete(&f);
    f.sample.move_state = 7;
    sample = feedback(&f, .29);
    /* Stale generations are fenced BEFORE their supplied time/feedback can
     * disturb the current owner. Only trusted eligible entry reads latch. */
    assert(je2_claim_pc(&f.v, f.v.generation - 1, NAN, &sample) == -1);
    assert(pc(&f, 0, "Switch {\"Switch\":false}", NAN) == -1);
    assert(f.v.owner == JE2_PENDANT && !f.v.core.fault && !f.stop && !f.jog_stop);
    tick(&f, .3);
    assert(f.v.owner == JE2_FAULT && f.stop == 1 && f.jog_stop == 1);
}
int main(void)
{
    auto_yield_and_fence(); cancelled_and_expired(); writer_unknown_running_timeout();
    new_session_only(); errors_and_stale_feedback(); guards_and_read_only_default(); strict_buttons();
    claim_fenced_before_clock(); duplicate_feedback_no_barrier_credit(); first_intent_release_and_hard_stop();
    cleanup_once_and_first_fault(); idle_qualification_revoked_and_commands_locked();
    stopped_state_cannot_reach_implicit_recovery();
    transient_feedback_failure_is_latched_at_entry();
    transient_feedback_preserves_startup_and_fencing();
    puts("PASS: JEServer_v2 + real guarded core: automatic yield/fencing/exit/cancel/fault/limits/button tests; fake SDK only");
    return 0;
}