/* Compile with function/data sections + linker GC: no vendor SDK library.
 * Include the exact adapter serializer rather than a copied test implementation.
 * PluginMain is never invoked; unused SDK/transport sections are discarded. */
#include "../mk2_control_plugin.c"
#include <assert.h>
#include <float.h>

/* Only publish_state is exercised; these are local test doubles, NOT SDK calls. */
static char captured_frame[MK2_FRAME_MAX];
static size_t captured_length;
const char *get_deviceName(int index, char *name)
{ (void)index; (void)name; return "fake_device"; }
const char *get_name_robot_device(const char *device, int index)
{ (void)device; (void)index; return "fake_group"; }
int GetGroupVelocity(const char *name, double *velocity)
{
    (void)name;
    for (int i = 0; i < MK2_AXES; i++) velocity[i] = -DBL_MAX;
    return 0;
}
int zmq_send(void *socket, const void *buffer, size_t length, int flags)
{
    (void)socket; (void)flags;
    assert(length < sizeof(captured_frame));
    memcpy(captured_frame, buffer, length);
    captured_frame[length] = '\0';
    captured_length = length;
    return (int)length;
}

int main(void)
{
    struct mk2_state_failure failure = {
        .captured = true, .have_sample = true, .have_command_time = true,
        .site = "control_tick", .phase = MK2_ACTIVE,
        .failed_checks = MK2_CHECK_NONFINITE | MK2_CHECK_POWER,
        .monotonic_time = NAN, .command_time = INFINITY,
        .sample = {.valid = false, .power = 0, .move_state = 7}
    };
    failure.sample.joint[0] = NAN;
    failure.sample.joint[1] = INFINITY;
    struct json_object *value = json_state_failure(&failure);
    assert(value != NULL);
    const char *text = json_object_to_json_string_ext(value, JSON_C_TO_STRING_PLAIN);
    assert(strstr(text, "NaN") == NULL && strstr(text, "Infinity") == NULL);
    assert(strlen(text) < MK2_FRAME_MAX);
    struct json_object *parsed = json_tokener_parse(text), *field = NULL;
    assert(parsed != NULL);
    assert(json_object_object_get_ex(parsed, "monotonic_time", &field) && field == NULL);
    assert(json_object_object_get_ex(parsed, "command_time", &field) && field == NULL);
    assert(json_object_object_get_ex(parsed, "Joint", &field));
    assert(json_object_array_length(field) == 7);
    assert(json_object_array_get_idx(field, 0) == NULL);
    assert(json_object_array_get_idx(field, 1) == NULL);
    assert(json_object_get_double(json_object_array_get_idx(field, 2)) == 0);
    assert(json_object_object_get_ex(parsed, "phase_before_stop", &field));
    assert(strcmp(json_object_get_string(field), "active") == 0);
    assert(json_object_object_get_ex(parsed, "PowerState", &field));
    assert(json_object_get_int(field) == 0);
    json_object_put(parsed);
    json_object_put(value);
    failure.captured = false;
    assert(json_state_failure(&failure) == NULL);
    struct mk2_servo_observation observation = {0};
    value = json_servo_observation(&observation);
    assert(json_object_object_get_ex(value, "ownership_evidence", &field));
    assert(!json_object_get_boolean(field));
    assert(!json_object_object_get_ex(value, "first_active_nonservo_sample", &field));
    assert(!json_object_object_get_ex(value, "last_sdk_call", &field));
    json_object_put(value);

    observation.seen_servo_state = true;
    observation.first_nonservo_captured = true;
    observation.servo_state_seen_before = true;
    observation.input_monotonic_time = INFINITY;
    observation.first_nonservo_sample = (struct mk2_sample){.valid = false, .move_state = 10};
    observation.last_call = (struct mk2_sdk_call_observation){
        .call = MK2_SDK_CALL_JOINT, .phase = MK2_ACTIVE, .result = 10,
        .input_monotonic_time = NAN, .command_time = INFINITY,
        .input_sample = {.valid = true, .power = 1, .move_state = 13}
    };
    for (int i = 0; i < MK2_AXES; i++) {
        observation.first_nonservo_sample.joint[i] = NAN;
        observation.last_call.input_sample.joint[i] = -DBL_MAX;
    }
    value = json_servo_observation(&observation);
    text = json_object_to_json_string_ext(value, JSON_C_TO_STRING_PLAIN);
    assert(strstr(text, "NaN") == NULL && strstr(text, "Infinity") == NULL);
    assert(json_object_object_get_ex(value, "first_active_nonservo_sample", &field));
    struct json_object *member = NULL;
    assert(json_object_object_get_ex(field, "input_monotonic_time", &member) && member == NULL);
    assert(json_object_object_get_ex(field, "MoveState", &member) && json_object_get_int(member) == 10);
    assert(json_object_object_get_ex(field, "Joint", &member));
    assert(json_object_array_get_idx(member, 0) == NULL);
    assert(json_object_object_get_ex(value, "last_sdk_call", &field));
    assert(json_object_object_get_ex(field, "call", &member));
    assert(strcmp(json_object_get_string(member), "ServoJoint") == 0);
    assert(json_object_object_get_ex(field, "result", &member) && json_object_get_int(member) == 10);
    assert(json_object_object_get_ex(field, "command_validation_sample", &member));
    assert(json_object_object_get_ex(member, "MoveState", &field) && json_object_get_int(field) == 13);
    /* Reserve space for the actual publisher's other metadata and Robot0.
     * Exercise long finite doubles as well as nulls in the diagnostic objects. */
    struct json_object *diagnostics = json_object_new_object();
    failure.captured = true;
    for (int i = 0; i < MK2_AXES; i++) failure.sample.joint[i] = DBL_MAX;
    json_object_object_add(diagnostics, "first_state_error", json_state_failure(&failure));
    json_object_object_add(diagnostics, "servo_observation", value);
    assert(strlen(json_object_to_json_string_ext(diagnostics, JSON_C_TO_STRING_PLAIN)) < MK2_FRAME_MAX - 1536);
    json_object_put(diagnostics);

    struct mk2_core core = {.phase = MK2_FAULT, .fault = MK2_STATE_ERROR,
        .accepted_joints = UINT64_MAX, .rejected_frames = UINT64_MAX,
        .first_state_error = failure, .servo_observation = observation};
    core.first_state_error.failed_checks = (1u << 9) - 1;
    core.first_state_error.monotonic_time = DBL_MAX;
    core.first_state_error.command_time = DBL_MAX;
    core.servo_observation.input_monotonic_time = DBL_MAX;
    core.servo_observation.last_call.input_monotonic_time = DBL_MAX;
    core.servo_observation.last_call.command_time = DBL_MAX;
    for (int i = 0; i < MK2_AXES; i++)
        core.servo_observation.first_nonservo_sample.joint[i] = -DBL_MAX;
    struct mk2_sample current = {.valid = true, .power = 1, .move_state = 13};
    for (int i = 0; i < MK2_AXES; i++) current.joint[i] = DBL_MAX;
    memset(policy.device_config_sha256, 'f', 64);
    policy.device_config_sha256[64] = '\0';
    publish_state(NULL, &core, &current, UINT64_MAX);
    assert(captured_length > 0 && captured_length < MK2_FRAME_MAX);
    assert(strncmp(captured_frame, "State ", 6) == 0);
    parsed = json_tokener_parse(captured_frame + 6);
    assert(parsed != NULL);
    assert(json_object_object_get_ex(parsed, "Robot0", &field));
    assert(json_object_object_get_ex(field, "Joint", &member));
    assert(json_object_get_double(json_object_array_get_idx(member, 0)) == DBL_MAX);
    assert(json_object_object_get_ex(field, "JointVelocity", &member));
    assert(json_object_array_length(member) == MK2_AXES);
    assert(json_object_object_get_ex(parsed, "Bridge", &field));
    assert(json_object_object_get_ex(field, "sequence", &member));
    assert(json_object_get_uint64(member) == UINT64_MAX);
    assert(json_object_object_get_ex(field, "first_state_error", &member));
    assert(json_object_object_get_ex(field, "servo_observation", &member));
    json_object_put(parsed);
    printf("PASS: full actual feedback publisher with worst-case doubles: %zu/%d bytes\n",
        captured_length, MK2_FRAME_MAX);
    assert(!started); /* No controller/transport/SDK session was created. */
    puts("PASS: actual adapter state/servo JSON nulls, omission, evidence flag and size reserve");
    return 0;
}