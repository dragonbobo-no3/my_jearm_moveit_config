/* Standalone fake-SDK regression. No vendor library, ROS, network or robot. */
#include "mk2_control_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

struct fake_sdk {
    struct mk2_core *core;
    struct mk2_sample *sample;
    int starts, joints, writes, stops, ends, start_result, joint_result;
    bool mutate_start, mutate_joint, expect_call_before_stop;
};
static struct mk2_sample healthy(int state)
{
    return (struct mk2_sample){.valid = true, .power = 1, .move_state = state};
}
static int start(void *context, double q, double r)
{
    (void)q; (void)r;
    struct fake_sdk *sdk = context;
    sdk->starts++;
    if (sdk->mutate_start) *sdk->sample = healthy(13);
    return sdk->start_result;
}
static int joint(void *context, const double *target, double time)
{
    (void)target; (void)time;
    struct fake_sdk *sdk = context;
    sdk->joints++;
    /* Model ONLY the observed SDK entry gate, NOT its execution thread. */
    if (sdk->sample->move_state == 13 && !sdk->joint_result) sdk->writes++;
    if (sdk->mutate_joint) *sdk->sample = healthy(0);
    return sdk->joint_result;
}
static int stop(void *context)
{
    struct fake_sdk *sdk = context;
    sdk->stops++;
    if (sdk->expect_call_before_stop)
        assert(sdk->core->servo_observation.last_call.call != MK2_SDK_CALL_NONE);
    *sdk->sample = healthy(0);
    return 0;
}
static int end(void *context) { ((struct fake_sdk *)context)->ends++; return 0; }
static void initialize(struct mk2_core *core, struct fake_sdk *sdk, struct mk2_sample *sample)
{
    struct mk2_policy policy = {.allow_commands = true, .watchdog = .2,
        .first_timeout = 1, .min_dt = .005, .max_dt = .05, .clock_skew = .1,
        .first_hold_tolerance = .001, .following_tolerance = .03, .servo_q = 1, .servo_r = 1};
    for (int i = 0; i < MK2_AXES; i++) {
        policy.lower[i] = -2; policy.upper[i] = 2;
        policy.velocity[i] = .2; policy.acceleration[i] = .3;
    }
    *sdk = (struct fake_sdk){.core = core, .sample = sample};
    const struct mk2_io io = {.context = sdk, .start = start, .joint = joint, .stop = stop, .end = end};
    assert(mk2_core_init(core, &policy, &io) == 0);
    core->phase = MK2_ACTIVE; core->owns_servo = true;
    core->first_receive = core->last_receive = 1;
    core->first_data_time = core->last_data_time = .01;
}
static int send_joint(struct mk2_core *core, const struct mk2_sample *sample)
{
    const char *frame = "Joint {\"Robot0\":{\"time\":0.02,\"joint\":[0,0,0,0,0,0,0]}}";
    return mk2_core_handle(core, frame, strlen(frame), 1.01, sample);
}
int main(void)
{
    struct mk2_core core;
    struct fake_sdk sdk;
    struct mk2_sample sample = healthy(13);

    initialize(&core, &sdk, &sample);
    mk2_core_tick(&core, 1.001, &sample);
    assert(core.servo_observation.seen_servo_state && !core.servo_observation.first_nonservo_captured);
    sample = healthy(0);
    mk2_core_tick(&core, 1.01, &sample);
    assert(core.phase == MK2_ACTIVE && core.owns_servo && core.fault == MK2_NO_FAULT);
    assert(core.servo_observation.first_nonservo_captured && core.servo_observation.servo_state_seen_before);
    assert(core.servo_observation.first_nonservo_sample.move_state == 0);
    sample = healthy(13); sample.joint[0] = .1;
    mk2_core_tick(&core, 1.02, &sample);
    assert(core.servo_observation.first_nonservo_sample.joint[0] == 0);
    assert(core.servo_observation.input_monotonic_time == 1.01);
    assert(sdk.starts == 0 && sdk.joints == 0 && sdk.stops == 0 && sdk.ends == 0);

    sample = healthy(0); initialize(&core, &sdk, &sample);
    mk2_core_tick(&core, 1.01, &sample);
    assert(!core.servo_observation.first_nonservo_captured);
    assert(!core.servo_observation.seen_servo_state);
    sample = healthy(13);
    mk2_core_tick(&core, 1.02, &sample);
    sample = healthy(10);
    mk2_core_tick(&core, 1.03, &sample);
    assert(core.servo_observation.first_nonservo_captured && core.servo_observation.servo_state_seen_before);
    assert(core.servo_observation.first_nonservo_sample.move_state == 10);
    assert(core.servo_observation.input_monotonic_time == 1.03);

    sample = healthy(-176); sample.move_error = 1;
    initialize(&core, &sdk, &sample);
    core.servo_observation.seen_servo_state = true;
    mk2_core_tick(&core, 1.01, &sample);
    assert(core.fault == MK2_STATE_ERROR && sdk.stops == 1 && sdk.ends == 1);
    assert(sample.move_state == 0); /* Stop overwrote feedback, not diagnostic. */
    assert(core.servo_observation.first_nonservo_sample.move_state == -176);
    assert(core.first_state_error.sample.move_error == 1);

    sample = healthy(13); initialize(&core, &sdk, &sample);
    sdk.joint_result = 10; sdk.mutate_joint = true; sdk.expect_call_before_stop = true;
    assert(send_joint(&core, &sample) == -1);
    assert(core.fault == MK2_SDK_ERROR && core.sdk_error == 10);
    assert(core.servo_observation.last_call.call == MK2_SDK_CALL_JOINT);
    assert(core.servo_observation.last_call.result == 10);
    assert(core.servo_observation.last_call.input_sample.move_state == 13);
    assert(core.servo_observation.last_call.phase == MK2_ACTIVE);
    assert(sdk.stops == 1 && sdk.ends == 1 && sdk.starts == 0 && sdk.joints == 1);
    sample = healthy(13);
    assert(send_joint(&core, &sample) == -1); /* No retry, reset or overwrite. */
    assert(sdk.joints == 1 && core.servo_observation.last_call.result == 10);

    sample = healthy(0); initialize(&core, &sdk, &sample);
    assert(send_joint(&core, &sample) == 0);
    assert(sdk.joints == 1 && sdk.writes == 0); /* SDK success != ownership proof. */
    assert(core.servo_observation.last_call.result == 0);
    assert(core.servo_observation.last_call.input_sample.move_state == 0);
    assert(core.phase == MK2_ACTIVE && sdk.stops == 0); /* Diagnostics do not change policy. */

    sample = healthy(0); initialize(&core, &sdk, &sample);
    core.phase = MK2_WAIT_FIRST; core.owns_servo = false;
    sdk.start_result = -104; sdk.mutate_start = true; sdk.expect_call_before_stop = true;
    assert(send_joint(&core, &sample) == -1);
    assert(core.fault == MK2_SDK_ERROR && core.servo_observation.last_call.call == MK2_SDK_CALL_START);
    assert(core.servo_observation.last_call.phase == MK2_WAIT_FIRST);
    assert(core.servo_observation.last_call.input_sample.move_state == 0);
    assert(sdk.starts == 1 && sdk.joints == 0 && sdk.stops == 1 && sdk.ends == 1);

    sample = healthy(0); initialize(&core, &sdk, &sample);
    core.phase = MK2_WAIT_FIRST; core.owns_servo = false;
    sdk.mutate_start = true; sdk.joint_result = -102;
    assert(send_joint(&core, &sample) == -1);
    assert(core.servo_observation.last_call.call == MK2_SDK_CALL_JOINT);
    /* Both SDK calls use the SAME command-validation sample, not a fictional
     * atomic post-ServoStart read. The JSON labels this explicitly. */
    assert(core.servo_observation.last_call.input_sample.move_state == 0);
    assert(sdk.starts == 1 && sdk.joints == 1 && sdk.stops == 1 && sdk.ends == 1);

    sample = healthy(0); initialize(&core, &sdk, &sample);
    core.phase = MK2_READY; core.owns_servo = false;
    mk2_core_tick(&core, 1.01, &sample);
    assert(!core.servo_observation.first_nonservo_captured && sdk.stops == 0);

    sample = healthy(13); initialize(&core, &sdk, &sample);
    mk2_core_tick(&core, 1.3, &sample);
    assert(core.fault == MK2_WATCHDOG && sdk.stops == 1 && sdk.ends == 1);
    assert(!core.servo_observation.first_nonservo_captured);

    sample = healthy(13); initialize(&core, &sdk, &sample);
    const char *bad = "Joint {\"Robot0\":{\"time\":0.02,\"joint\":[3,0,0,0,0,0,0]}}";
    assert(mk2_core_handle(&core, bad, strlen(bad), 1.01, &sample) == -1);
    assert(core.fault == MK2_LIMIT && sdk.stops == 1 && sdk.joints == 0);
    assert(core.servo_observation.last_call.call == MK2_SDK_CALL_NONE);

    sample = healthy(0); initialize(&core, &sdk, &sample);
    core.phase = MK2_WAIT_FIRST; core.owns_servo = false;
    core.servo_observation.seen_servo_state = true;
    core.servo_observation.first_nonservo_captured = true;
    core.servo_observation.first_nonservo_sample = healthy(10);
    sdk.mutate_start = true;
    assert(send_joint(&core, &sample) == 0);
    assert(!core.servo_observation.seen_servo_state && !core.servo_observation.first_nonservo_captured);
    assert(core.servo_observation.last_call.call == MK2_SDK_CALL_JOINT);
    assert(sdk.starts == 1 && sdk.joints == 1 && sdk.writes == 1 && sdk.stops == 0);

    assert(strcmp(mk2_sdk_call_name(MK2_SDK_CALL_START), "ServoStart") == 0);
    assert(strcmp(mk2_sdk_call_name(MK2_SDK_CALL_JOINT), "ServoJoint") == 0);
    assert(strcmp(mk2_sdk_call_name((enum mk2_sdk_call)99), "invalid") == 0);
    puts("PASS: 11 servo observation scenarios; guards unchanged; fake SDK only");
    return 0;
}