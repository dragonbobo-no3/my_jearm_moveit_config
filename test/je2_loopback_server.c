#define _POSIX_C_SOURCE 200809L
#define JE2_SIM_SESSION_EXPERIMENT 1
#include "je2_loopback_server.h"
#include <json-c/json.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Include the actual parser with test-local IO/time providers, not a parallel
 * protocol implementation. Only a serialized call may select a fixture. */
static _Thread_local struct je2_loopback_server *current;
static double monotonic_time(void) { return current->now; }
static struct je2_feedback je2_read_feedback(void)
{
    ++current->reads;
    return (struct je2_feedback){.sample = current->sample, .acquired_at = current->now};
}
#include "jeserver_v2_session.inc"
#include "jeserver_v2_owner.inc"

static int start(void *p, double q, double r)
{
    (void)q; (void)r;
    struct je2_loopback_server *s = p;
    ++s->starts; s->sample.move_state = 13; s->writer = JE2_WRITER_RUNNING;
    return 0;
}
static int joint(void *p, const double *q, double time)
{
    struct je2_loopback_server *s = p;
    ++s->joints;
    if (s->joint_result) return s->joint_result;
    memcpy(s->target, q, sizeof(s->target));
    memcpy(s->sample.joint, q, sizeof(s->sample.joint)); // idealized test feedback only
    s->data_time = time;
    return 0;
}
static int stop(void *p) { ++((struct je2_loopback_server *)p)->stops; return 0; }
static int end(void *p) { ++((struct je2_loopback_server *)p)->ends; return 0; }
static int release(void *p)
{
    struct je2_loopback_server *s = p;
    ++s->releases;
    if (s->release_result) return s->release_result;
    s->sample.move_state = 0; // FAKE clean release, no RobotStopRun call
    return 0;
}
static enum je2_writer writer(void *p) { return ((struct je2_loopback_server *)p)->writer; }
static int jog(void *p, int axis, int direction)
{
    (void)axis; (void)direction;
    struct je2_loopback_server *s = p;
    ++s->jogs; s->sample.move_state = 1;
    return 0;
}
static int jog_stop(void *p) { ((struct je2_loopback_server *)p)->sample.move_state = 0; return 0; }

int je2_loopback_init(struct je2_loopback_server *s)
{
    memset(s, 0, sizeof(*s));
    s->sample.valid = true; s->sample.power = 1;
    s->sample.joint[0] = .125;
    s->writer = JE2_WRITER_UNKNOWN;
    struct mk2_policy policy = {.allow_commands = true, .watchdog = .2,
        .first_timeout = 1, .min_dt = .005, .max_dt = .05, .clock_skew = .1,
        .first_hold_tolerance = .001, .following_tolerance = .02, .servo_q = 1, .servo_r = .0001};
    for (int i = 0; i < MK2_AXES; ++i) {
        policy.lower[i] = -1; policy.upper[i] = 1;
        policy.velocity[i] = .2; policy.acceleration[i] = .4;
    }
    const struct mk2_io io = {.context = s, .start = start, .joint = joint,
        .stop = stop, .end = end, .stop_run = release};
    const struct je2_io teach = {.context = s, .writer = writer, .jog = jog, .jog_stop = jog_stop};
    if (je2_init(&s->engine, &policy, &io, &teach)) return -1;
    return je2_session_init(&s->session); // actual kernel-generated server boot
}
int je2_loopback_receive(struct je2_loopback_server *s, const char *text, size_t size, bool more, double now)
{
    current = s; s->now = now;
    const int result = je2_session_receive(&s->engine, &s->session, text, size, more);
    current = NULL;
    return result;
}
int je2_loopback_snapshot(struct je2_loopback_server *s, double now, char *out, size_t size)
{
    s->now = now;
    return je2_format_owner(out, size, &s->engine, &s->sample, ++s->observation, 0,
        &s->session, false, false, 0); // never advertise real writer-exit verification
}
int je2_loopback_button(struct je2_loopback_server *s, const char *text, size_t size, double now)
{
    int axis = 0, direction = 0;
    const int decoded = je2_decode_button(text, size, &axis, &direction);
    current = s; s->now = now;
    int result = -1;
    if (decoded == 1) {
        const struct je2_feedback f = je2_read_feedback();
        result = je2_pendant_jog(&s->engine, axis, direction, now, now, &f);
    } else if (decoded == 2) result = je2_pendant_stop(&s->engine, now);
    current = NULL;
    return result;
}
void je2_loopback_tick(struct je2_loopback_server *s, double now)
{
    current = s; s->now = now;
    const struct je2_feedback f = je2_read_feedback();
    je2_tick(&s->engine, now, &f);
    current = NULL;
}