/* Standalone fake-SDK regression for the first-hold activation recovery.
 * No vendor library, ROS, network or robot. Verifies that an optional recover
 * callback runs exactly once, BEFORE ServoStart, only on the first hold, and
 * that a recover failure faults without opening or cleaning up servo. */
#include "mk2_control_core.h"
#include <assert.h>
#include <string.h>

struct fake_sdk {
    int recovers, starts, joints, stops, ends;
    int recover_result;
    bool start_after_recover, joint_after_start;
};
static struct mk2_sample healthy0(void)
{
    return (struct mk2_sample){.valid = true, .power = 1, .move_state = 0};
}
static int fs_recover(void *context)
{
    struct fake_sdk *sdk = context;
    sdk->recovers++;
    return sdk->recover_result;
}
static int fs_start(void *context, double q, double r)
{
    (void)q; (void)r;
    struct fake_sdk *sdk = context;
    if (sdk->recovers > 0) sdk->start_after_recover = true;
    sdk->starts++;
    return 0;
}
static int fs_joint(void *context, const double *target, double data_time)
{
    (void)target; (void)data_time;
    struct fake_sdk *sdk = context;
    if (sdk->starts > 0) sdk->joint_after_start = true;
    sdk->joints++;
    return 0;
}
static int fs_stop(void *context) { ((struct fake_sdk *)context)->stops++; return 0; }
static int fs_end(void *context) { ((struct fake_sdk *)context)->ends++; return 0; }

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
static void arm_wait_first(struct mk2_core *core, struct fake_sdk *sdk,
                           const struct mk2_policy *policy, bool with_recover)
{
    *sdk = (struct fake_sdk){0};
    struct mk2_io io = {.context = sdk, .start = fs_start, .joint = fs_joint,
                        .stop = fs_stop, .end = fs_end};
    if (with_recover) io.recover = fs_recover;
    assert(mk2_core_init(core, policy, &io) == 0);
    assert(core->phase == MK2_READY);
    struct mk2_sample sample = healthy0();
    const char *sw = "Switch {\"Switch\":true}";
    assert(mk2_core_handle(core, sw, strlen(sw), 1.0, &sample) == 0);
    assert(core->phase == MK2_WAIT_FIRST);
}
static int first_hold(struct mk2_core *core)
{
    struct mk2_sample sample = healthy0();
    const char *frame = "Joint {\"Robot0\":{\"time\":0.02,\"joint\":[0,0,0,0,0,0,0]}}";
    return mk2_core_handle(core, frame, strlen(frame), 1.01, &sample);
}
int main(void)
{
    struct mk2_policy policy;
    policy_init(&policy);

    /* Recover succeeds: runs once, before ServoStart, then ServoJoint. */
    struct mk2_core core;
    struct fake_sdk sdk;
    arm_wait_first(&core, &sdk, &policy, true);
    sdk.recover_result = 0;
    assert(first_hold(&core) == 0);
    assert(core.phase == MK2_ACTIVE && core.fault == MK2_NO_FAULT && core.owns_servo);
    assert(sdk.recovers == 1 && sdk.starts == 1 && sdk.joints == 1);
    assert(sdk.start_after_recover && sdk.joint_after_start);
    assert(core.servo_observation.last_call.call == MK2_SDK_CALL_JOINT);
    assert(core.accepted_joints == 1);

    /* Recover failure: faults as SDK error, never opens or cleans up servo. */
    arm_wait_first(&core, &sdk, &policy, true);
    sdk.recover_result = 7;
    assert(first_hold(&core) == -1);
    assert(core.phase == MK2_FAULT && core.fault == MK2_SDK_ERROR && core.sdk_error == 7);
    assert(!core.owns_servo);
    assert(sdk.recovers == 1 && sdk.starts == 0 && sdk.joints == 0);
    assert(sdk.stops == 0 && sdk.ends == 0);
    assert(core.servo_observation.last_call.call == MK2_SDK_CALL_RECOVER);
    assert(core.servo_observation.last_call.result == 7);

    /* No recover callback: unchanged legacy behavior, start then joint only. */
    arm_wait_first(&core, &sdk, &policy, false);
    assert(first_hold(&core) == 0);
    assert(core.phase == MK2_ACTIVE && core.fault == MK2_NO_FAULT);
    assert(sdk.recovers == 0 && sdk.starts == 1 && sdk.joints == 1);
    assert(core.servo_observation.last_call.call == MK2_SDK_CALL_JOINT);

    assert(!strcmp(mk2_sdk_call_name(MK2_SDK_CALL_RECOVER), "RobotStopRecover"));
    assert(!strcmp(mk2_sdk_call_name(MK2_SDK_CALL_START), "ServoStart"));
    assert(!strcmp(mk2_sdk_call_name(MK2_SDK_CALL_JOINT), "ServoJoint"));
    return 0;
}
