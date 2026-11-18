/* Standalone fake-SDK regression for the opt-in forced-stop rearm. No vendor
 * library, ROS, network or robot. Only a clean stop or an active-servo forced
 * stop (move_state 7) and/or power loss (pendant disable) may return to READY, and only after sustained
 * healthy, idle, stationary feedback. The only SDK call is one recover that
 * clears this plugin's own, continuously observed RobotStop latch. */
#include "mk2_control_core.h"
#include <assert.h>
#include <string.h>

struct fake_sdk { int recovers, starts, joints, stops, ends, recover_result; };
static int fs_recover(void *c) { ((struct fake_sdk *)c)->recovers++; return ((struct fake_sdk *)c)->recover_result; }
static int fs_start(void *c, double q, double r) { (void)q; (void)r; ((struct fake_sdk *)c)->starts++; return 0; }
static int fs_joint(void *c, const double *t, double d) { (void)t; (void)d; ((struct fake_sdk *)c)->joints++; return 0; }
static int fs_stop(void *c) { ((struct fake_sdk *)c)->stops++; return 0; }
static int fs_end(void *c) { ((struct fake_sdk *)c)->ends++; return 0; }

static struct mk2_sample healthy(int move_state)
{
    return (struct mk2_sample){.valid = true, .power = 1, .move_state = move_state};
}
static void policy_init(struct mk2_policy *policy)
{
    *policy = (struct mk2_policy){.allow_commands = true, .watchdog = .2, .first_timeout = 1,
        .min_dt = .005, .max_dt = .05, .clock_skew = .1, .first_hold_tolerance = .001,
        .following_tolerance = .03, .servo_q = 1, .servo_r = 1};
    for (int i = 0; i < MK2_AXES; i++) {
        policy->lower[i] = -2; policy->upper[i] = 2;
        policy->velocity[i] = .2; policy->acceleration[i] = .3;
    }
}
static int send(struct mk2_core *core, const char *frame, double now, struct mk2_sample sample)
{
    return mk2_core_handle(core, frame, strlen(frame), now, &sample);
}
/* READY -> Switch -> first hold -> ACTIVE (servo owned, SDK state 13). */
static void activate(struct mk2_core *core, struct fake_sdk *sdk, bool rearm)
{
    struct mk2_policy policy; policy_init(&policy);
    *sdk = (struct fake_sdk){0};
    struct mk2_io io = {.context = sdk, .start = fs_start, .joint = fs_joint,
                        .stop = fs_stop, .end = fs_end, .recover = fs_recover};
    assert(mk2_core_init(core, &policy, &io) == 0);
    core->rearm_after_forced_stop = rearm;
    assert(send(core, "Switch {\"Switch\":true}", 1.0, healthy(0)) == 0);
    assert(send(core, "Joint {\"Robot0\":{\"time\":0.02,\"joint\":[0,0,0,0,0,0,0]}}", 1.01, healthy(0)) == 0);
    assert(core->phase == MK2_ACTIVE && core->owns_servo);
}
static void idle(struct mk2_core *core, double from, double to, struct mk2_sample sample)
{
    for (double t = from; t <= to + 1e-9; t += .005) mk2_core_tick(core, t, &sample);
}
static void forced_stop(struct mk2_core *core)
{
    struct mk2_sample seven = healthy(7);
    mk2_core_tick(core, 1.02, &seven);
    assert(core->phase == MK2_FAULT && core->fault == MK2_STATE_ERROR && !core->owns_servo);
    assert(core->first_state_error.captured &&
           core->first_state_error.failed_checks == MK2_CHECK_MOVE_STATE);
}
int main(void)
{
    struct mk2_core core;
    struct fake_sdk sdk;

    /* Default (jeserver_v2, legacy): a forced stop stays latched. */
    activate(&core, &sdk, false);
    forced_stop(&core);
    idle(&core, 1.03, 5.0, healthy(0));
    assert(core.phase == MK2_FAULT && core.rearm_count == 0);
    assert(send(&core, "Switch {\"Switch\":true}", 5.01, healthy(0)) == -1);

    /* Opt-in: PC stream loss -> SDK 7 -> latched, then READY after 1 s idle. */
    activate(&core, &sdk, true);
    forced_stop(&core);
    assert(sdk.stops == 1 && sdk.ends == 1);
    idle(&core, 1.03, 1.06, healthy(7));             /* still forced-stopped */
    assert(core.phase == MK2_FAULT);
    idle(&core, 1.07, 1.9, healthy(0));              /* < 1 s healthy idle */
    assert(core.phase == MK2_FAULT && core.rearm_count == 0);
    idle(&core, 1.905, 2.2, healthy(0));
    assert(core.phase == MK2_READY && core.fault == MK2_NO_FAULT && core.rearm_count == 1);
    assert(!core.first_state_error.captured && !core.owns_servo);
    assert(sdk.recovers == 1 && sdk.starts == 1 && sdk.stops == 1 && sdk.ends == 1); /* no SDK call */
    /* A fresh session goes through every normal step again. */
    assert(send(&core, "Switch {\"Switch\":true}", 2.3, healthy(0)) == 0);
    assert(core.phase == MK2_WAIT_FIRST);
    assert(send(&core, "Joint {\"Robot0\":{\"time\":0.02,\"joint\":[0,0,0,0,0,0,0]}}", 2.31, healthy(0)) == 0);
    assert(core.phase == MK2_ACTIVE && sdk.recovers == 2 && sdk.starts == 2);

    /* Opt-in: motion during idle restarts the settle timer. */
    activate(&core, &sdk, true);
    forced_stop(&core);
    idle(&core, 1.03, 1.8, healthy(0));
    struct mk2_sample moved = healthy(0); moved.joint[2] = .01;
    idle(&core, 1.805, 2.5, moved);
    assert(core.phase == MK2_FAULT);
    idle(&core, 2.505, 2.81, moved);
    assert(core.phase == MK2_READY);

    /* Opt-in: a clean Switch false stop also rearms. */
    activate(&core, &sdk, true);
    assert(send(&core, "Switch {\"Switch\":false}", 1.02, healthy(0)) == 0);
    assert(core.phase == MK2_STOPPED && core.fault == MK2_NO_FAULT);
    idle(&core, 1.03, 2.1, healthy(0));
    assert(core.phase == MK2_READY);

    /* Opt-in: our own RobotStop leaves 7 latched (observed on hardware). One
     * recover after 1 s stationary latched idle, then 1 s of move_state 0. */
    activate(&core, &sdk, true);
    assert(send(&core, "Switch {\"Switch\":false}", 1.02, healthy(13)) == 0);
    assert(core.phase == MK2_STOPPED && core.stop_latch_ours && sdk.recovers == 1);
    idle(&core, 1.03, 1.06, healthy(13));            /* decelerating: keeps attribution */
    idle(&core, 1.065, 2.0, healthy(7));
    assert(core.phase == MK2_STOPPED && sdk.recovers == 1);
    idle(&core, 2.005, 2.2, healthy(7));
    assert(core.phase == MK2_STOPPED && sdk.recovers == 2 && core.rearm_recovers == 1);
    idle(&core, 2.205, 4.0, healthy(7));             /* still 7: never a second recover */
    assert(core.phase == MK2_STOPPED && sdk.recovers == 2);
    idle(&core, 4.005, 4.9, healthy(0));
    assert(core.phase == MK2_STOPPED);
    idle(&core, 4.905, 5.2, healthy(0));
    assert(core.phase == MK2_READY && core.rearm_count == 1 && sdk.recovers == 2);
    assert(send(&core, "Switch {\"Switch\":true}", 5.3, healthy(0)) == 0);
    assert(send(&core, "Joint {\"Robot0\":{\"time\":0.02,\"joint\":[0,0,0,0,0,0,0]}}", 5.31, healthy(0)) == 0);
    assert(core.phase == MK2_ACTIVE && sdk.recovers == 3 && sdk.starts == 2);

    /* Opt-in: motion while latched restarts the timer before any recover. */
    activate(&core, &sdk, true);
    assert(send(&core, "Switch {\"Switch\":false}", 1.02, healthy(13)) == 0);
    idle(&core, 1.03, 1.8, healthy(7));
    struct mk2_sample moved7 = healthy(7); moved7.joint[1] = .01;
    idle(&core, 1.805, 2.5, moved7);
    assert(sdk.recovers == 1);
    idle(&core, 2.505, 2.81, moved7);
    assert(sdk.recovers == 2);

    /* Opt-in: power/error/bounds or an idle 0 since the stop ends attribution;
     * a later 7 (e.g. pendant stop) is never recovered. */
    struct mk2_sample breaks[4] = {healthy(7), healthy(7), healthy(7), healthy(0)};
    breaks[0].power = 0; breaks[1].move_error = 2; breaks[2].joint[3] = 5;
    for (int c = 0; c < 4; c++) {
        activate(&core, &sdk, true);
        assert(send(&core, "Switch {\"Switch\":false}", 1.02, healthy(13)) == 0);
        idle(&core, 1.03, 1.2, healthy(7));
        mk2_core_tick(&core, 1.205, &breaks[c]);
        assert(!core.stop_latch_ours);
        idle(&core, 1.21, 5.0, healthy(7));
        assert(core.phase == MK2_STOPPED && sdk.recovers == 1 && core.rearm_count == 0);
    }

    /* Opt-in: a failed recover latches as SDK error, never retried. */
    activate(&core, &sdk, true);
    sdk.recover_result = -5;
    assert(send(&core, "Switch {\"Switch\":false}", 1.02, healthy(13)) == 0);
    idle(&core, 1.03, 2.2, healthy(7));
    assert(core.phase == MK2_FAULT && core.fault == MK2_SDK_ERROR && sdk.recovers == 2);
    idle(&core, 2.205, 5.0, healthy(0));
    assert(core.phase == MK2_FAULT && sdk.recovers == 2 && core.rearm_count == 0);

    /* Default off: an own latch is never recovered. */
    activate(&core, &sdk, false);
    assert(send(&core, "Switch {\"Switch\":false}", 1.02, healthy(13)) == 0);
    idle(&core, 1.03, 5.0, healthy(7));
    assert(core.phase == MK2_STOPPED && sdk.recovers == 1);

    /* Opt-in: PC stream loss forced stop that stays 7 is also our latch. */
    activate(&core, &sdk, true);
    forced_stop(&core);
    assert(core.stop_latch_ours);
    idle(&core, 1.03, 2.1, healthy(7));
    assert(core.phase == MK2_FAULT && sdk.recovers == 2);
    idle(&core, 2.105, 3.2, healthy(0));
    assert(core.phase == MK2_READY && core.fault == MK2_NO_FAULT && sdk.recovers == 2);

    /* Opt-in: pendant disable (power loss, with or without 7) while active
     * rearms only after re-enable and 1 s powered healthy idle; no recover. */
    struct mk2_sample offs[2] = {healthy(13), healthy(7)};
    offs[0].power = 0; offs[1].power = 0;
    for (int c = 0; c < 2; c++) {
        activate(&core, &sdk, true);
        mk2_core_tick(&core, 1.02, &offs[c]);
        assert(core.phase == MK2_FAULT && core.fault == MK2_STATE_ERROR && sdk.stops == 1);
        struct mk2_sample off = healthy(0); off.power = 0;
        idle(&core, 1.03, 5.0, off);                  /* still disabled */
        assert(core.phase == MK2_FAULT && core.rearm_count == 0);
        idle(&core, 5.005, 5.9, healthy(0));          /* re-enabled, < 1 s */
        assert(core.phase == MK2_FAULT);
        idle(&core, 5.905, 6.2, healthy(0));
        assert(core.phase == MK2_READY && core.rearm_count == 1 && sdk.recovers == 1);
    }
    /* Re-enabled but left at 7 is not our latch: no recover, stays FAULT. */
    activate(&core, &sdk, true);
    mk2_core_tick(&core, 1.02, &offs[0]);
    idle(&core, 1.03, 5.0, healthy(7));
    assert(core.phase == MK2_FAULT && sdk.recovers == 1);

    /* Opt-in: forced stop/power loss combined with any other cause stays latched. */
    struct mk2_sample causes[3] = {healthy(7), healthy(7), healthy(7)};
    causes[0].driver_error = 3; causes[1].power = 0; causes[1].move_error = 4; causes[2].joint[0] = 5;
    for (int c = 0; c < 3; c++) {
        activate(&core, &sdk, true);
        mk2_core_tick(&core, 1.02, &causes[c]);
        assert(core.phase == MK2_FAULT && core.fault == MK2_STATE_ERROR);
        idle(&core, 1.03, 5.0, healthy(0));
        assert(core.phase == MK2_FAULT && core.rearm_count == 0);
    }

    /* Opt-in: non-state faults (watchdog, following, SDK error) stay latched. */
    activate(&core, &sdk, true);
    mk2_core_tick(&core, 1.5, &(struct mk2_sample){.valid = true, .power = 1, .move_state = 13});
    assert(core.phase == MK2_FAULT && core.fault == MK2_WATCHDOG);
    idle(&core, 1.51, 5.0, healthy(0));
    assert(core.phase == MK2_FAULT);
    activate(&core, &sdk, true);
    assert(send(&core, "Joint {\"Robot0\":{\"time\":0.03,\"joint\":[0.5,0,0,0,0,0,0]}}", 1.02, healthy(13)) == -1);
    assert(core.phase == MK2_FAULT && core.fault == MK2_FOLLOWING);
    idle(&core, 1.03, 5.0, healthy(0));
    assert(core.phase == MK2_FAULT);

    /* Opt-in: forced stop while waiting for the first hold stays latched. */
    struct mk2_policy policy; policy_init(&policy);
    sdk = (struct fake_sdk){0};
    struct mk2_io io = {.context = &sdk, .start = fs_start, .joint = fs_joint, .stop = fs_stop, .end = fs_end};
    assert(mk2_core_init(&core, &policy, &io) == 0);
    core.rearm_after_forced_stop = true;
    assert(send(&core, "Switch {\"Switch\":true}", 1.0, healthy(0)) == 0);
    mk2_core_tick(&core, 1.01, &(struct mk2_sample){.valid = true, .power = 1, .move_state = 7});
    assert(core.phase == MK2_FAULT);
    idle(&core, 1.02, 5.0, healthy(0));
    assert(core.phase == MK2_FAULT);
    return 0;
}
