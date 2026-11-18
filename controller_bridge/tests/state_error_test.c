/* Standalone native validator test, not a ROS/CMake target. No vendor SDK. */
#include "mk2_control_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

struct fake_sdk {
    int stops, ends, starts, joints, stop_result;
    struct mk2_sample *feedback;
};
static struct mk2_sample healthy(void)
{
    struct mk2_sample sample = {.valid = true, .power = 1};
    return sample;
}
static int start(void *context, double q, double r)
{ (void)q; (void)r; ((struct fake_sdk *)context)->starts++; return 0; }
static int joint(void *context, const double *target, double time)
{ (void)target; (void)time; ((struct fake_sdk *)context)->joints++; return 0; }
static int stop(void *context)
{
    struct fake_sdk *sdk = context;
    sdk->stops++;
    /* Simulate SDK Stop changing status immediately, before any next PUB. */
    if (sdk->feedback) *sdk->feedback = healthy();
    return sdk->stop_result;
}
static int end(void *context)
{ ((struct fake_sdk *)context)->ends++; return 0; }
static void initialize(struct mk2_core *core, struct fake_sdk *sdk)
{
    struct mk2_policy policy = {.allow_commands = true, .watchdog = .2,
        .first_timeout = 1, .min_dt = .005, .max_dt = .05, .clock_skew = .1,
        .first_hold_tolerance = .001, .following_tolerance = .03, .servo_q = 1, .servo_r = 1};
    for (int i = 0; i < MK2_AXES; i++) {
        policy.lower[i] = -2; policy.upper[i] = 2;
        policy.velocity[i] = .2; policy.acceleration[i] = .3;
    }
    struct mk2_io io = {.context = sdk, .start = start, .joint = joint, .stop = stop, .end = end};
    assert(mk2_core_init(core, &policy, &io) == 0);
    core->phase = MK2_ACTIVE;
    core->owns_servo = true;
    core->last_receive = core->first_receive = 1;
    core->last_data_time = core->first_data_time = .01;
}
int main(void)
{
    const unsigned expected[] = {MK2_CHECK_POWER, MK2_CHECK_DRIVER, MK2_CHECK_MOVE_ERROR,
        MK2_CHECK_MOVE_STATE, MK2_CHECK_BOUNDS, MK2_CHECK_INVALID_SAMPLE,
        MK2_CHECK_NONFINITE, MK2_CHECK_FIRST_NOT_IDLE, MK2_CHECK_MOVE_STATE};
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        struct mk2_sample sample = healthy();
        struct fake_sdk sdk = {.feedback = &sample};
        struct mk2_core core;
        initialize(&core, &sdk);
        switch (i) {
            case 0: sample.power = 0; break;
            case 1: sample.driver_error = 1; break;
            case 2: sample.move_error = 1; break;
            case 3: sample.move_state = 7; break;
            case 4: sample.joint[2] = 2.1; break;
            case 5: sample.valid = false; break;
            case 6: sample.joint[3] = NAN; break;
            case 7: core.phase = MK2_WAIT_FIRST; core.owns_servo = false; sample.move_state = 13; break;
            case 8: sample.move_state = -1; break;
        }
        struct mk2_sample before = sample;
        mk2_core_tick(&core, 1.01, &sample);
        assert(core.phase == MK2_FAULT && core.fault == MK2_STATE_ERROR);
        assert(core.first_state_error.captured && core.first_state_error.have_sample);
        assert(core.first_state_error.failed_checks & expected[i]);
        assert(core.first_state_error.monotonic_time == 1.01);
        assert(core.first_state_error.sample.power == before.power);
        assert(core.first_state_error.sample.driver_error == before.driver_error);
        assert(core.first_state_error.sample.move_error == before.move_error);
        assert(core.first_state_error.sample.move_state == before.move_state);
        assert(core.first_state_error.sample.valid == before.valid);
        for (int j = 0; j < MK2_AXES; j++)
            assert((isnan(before.joint[j]) && isnan(core.first_state_error.sample.joint[j])) ||
                   before.joint[j] == core.first_state_error.sample.joint[j]);
        assert(sdk.stops == (i == 7 ? 0 : 1) && sdk.ends == sdk.stops);
        assert(sdk.starts == 0 && sdk.joints == 0);
        sample = healthy();
        mk2_core_tick(&core, 1.02, &sample);
        mk2_core_record_state_error(&core, "later_state", 1.03, &sample, false, 0);
        assert(core.first_state_error.monotonic_time == 1.01);
        assert(strcmp(core.first_state_error.site, "control_tick") == 0);
        assert(core.phase == MK2_FAULT);
    }
    {
        struct mk2_sample sample = healthy();
        struct fake_sdk sdk = {.feedback = &sample};
        struct mk2_core core;
        initialize(&core, &sdk);
        const char *frame = "Joint {\"Robot0\":{\"joint\":[0,0,0,0,0,0,0],\"time\":0}}";
        assert(mk2_core_handle(&core, frame, strlen(frame), 1.01, &sample) == -1);
        assert(core.fault == MK2_STATE_ERROR);
        assert(core.first_state_error.failed_checks == MK2_CHECK_COMMAND_TIME);
        assert(core.first_state_error.have_command_time && core.first_state_error.command_time == 0);
        assert(strcmp(core.first_state_error.site, "joint_command") == 0);
        assert(sdk.stops == 1 && sdk.ends == 1 && sdk.starts == 0 && sdk.joints == 0);
    }
    {
        struct mk2_sample sample = healthy();
        sample.power = 0;
        struct fake_sdk sdk = {.feedback = &sample};
        struct mk2_core core;
        initialize(&core, &sdk);
        core.phase = MK2_READY; core.owns_servo = false;
        mk2_core_tick(&core, 1.01, &sample);
        assert(!core.first_state_error.captured && core.phase == MK2_READY);
        assert(sdk.stops == 0 && sdk.ends == 0);
    }
    {
        struct fake_sdk sdk = {0};
        struct mk2_core core;
        initialize(&core, &sdk);
        mk2_core_record_state_error(&core, "backend_verification", 1.01, NULL, false, 0);
        mk2_core_stop(&core, MK2_STATE_ERROR);
        assert(core.first_state_error.captured && !core.first_state_error.have_sample);
        assert(strcmp(core.first_state_error.site, "backend_verification") == 0);
    }
    puts("PASS: 12 state-error snapshot scenarios; zero hardware/SDK calls");
    return 0;
}