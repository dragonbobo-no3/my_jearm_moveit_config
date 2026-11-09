/* Original standalone vendor-SDK C plugin, NOT deployed by the ROS build.
 * No RobotPower/RobotPoweroff/ClearRobotError/RobotStopRecover or gripper API.
 * Compilation and fake-SDK tests do not authorize physical deployment/motion. */
#define _POSIX_C_SOURCE 200809L
#include "HYYRobotInterface.h"
#include "mk2_control_core.h"
#include <json-c/json.h>
#include <openssl/evp.h>
#include <stdatomic.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <zmq.h>

#ifndef PROFILE_ROOT
#define PROFILE_ROOT "/home/robot/Work/system/robot_config_new_left"
#endif
static atomic_bool stopping = ATOMIC_VAR_INIT(false);
static bool started;
static pthread_t worker;
static struct mk2_policy policy;

static double monotonic_time(void)
{
    struct timespec time;
    if (clock_gettime(CLOCK_MONOTONIC, &time)) return NAN;
    return (double)time.tv_sec + (double)time.tv_nsec * 1e-9;
}
static bool correct_profile(void)
{
    char arguments[8192];
    FILE *file = fopen("/proc/self/cmdline", "rb");
    if (!file) return false;
    size_t n = fread(arguments, 1, sizeof(arguments) - 1, file);
    fclose(file); arguments[n] = '\0';
    for (size_t i = 0; i < n;) {
        size_t length = strnlen(arguments + i, n - i);
        if (!strcmp(arguments + i, "--path")) {
            size_t next = i + length + 1;
            return next < n && !strcmp(arguments + next, PROFILE_ROOT);
        }
        i += length + 1;
    }
    return false;
}
static bool configuration_hash_matches(void)
{
    FILE *file = fopen(PROFILE_ROOT "/DeviceConfig.yaml", "rb");
    if (!file) return false;
    EVP_MD_CTX *digest = EVP_MD_CTX_new();
    unsigned char buffer[8192], result[EVP_MAX_MD_SIZE];
    unsigned int length = 0;
    bool ok = digest && EVP_DigestInit_ex(digest, EVP_sha256(), NULL) == 1;
    size_t n;
    while (ok && (n = fread(buffer, 1, sizeof(buffer), file)) > 0)
        ok = EVP_DigestUpdate(digest, buffer, n) == 1;
    ok = ok && !ferror(file) && EVP_DigestFinal_ex(digest, result, &length) == 1 && length == 32;
    fclose(file);
    if (digest) EVP_MD_CTX_free(digest);
    if (!ok) return false;
    char hexadecimal[65];
    for (unsigned int i = 0; i < length; i++) snprintf(hexadecimal + i * 2, 3, "%02x", result[i]);
    return !strcmp(hexadecimal, policy.device_config_sha256);
}
static bool load_policy(void)
{
    FILE *file = fopen(PROFILE_ROOT "/mk2_control_bridge.json", "rb");
    if (!file) return false;
    char text[MK2_FRAME_MAX + 1], error[128];
    size_t length = fread(text, 1, sizeof(text), file);
    bool read_ok = !ferror(file) && length <= MK2_FRAME_MAX;
    fclose(file);
    if (!read_ok || mk2_policy_parse(text, length, &policy, error, sizeof(error))) {
        fprintf(stderr, "[MK2Control] Invalid/missing commissioning policy; refusing startup\n");
        return false;
    }
    if (!configuration_hash_matches()) {
        fprintf(stderr, "[MK2Control] DeviceConfig hash differs from verified policy; refusing startup\n");
        return false;
    }
    return true;
}
static int sdk_start(void *unused, double q, double r)
{ (void)unused; return ServoStart(q, r, 0); }
static int sdk_joint(void *unused, const double *target, double data_time)
{
    (void)unused;
    double copy[MK2_AXES]; memcpy(copy, target, sizeof(copy));
    robjoint joint;
    init_robjoint(&joint, copy, MK2_AXES);
    return ServoJoint(&joint, data_time, 0);
}
static int sdk_stop(void *unused) { (void)unused; return RobotStop(0); }
static int sdk_end(void *unused) { (void)unused; return ServoEnd(0); }
static struct mk2_sample read_sample(void)
{
    struct mk2_sample sample = {.valid = false, .power = -1, .driver_error = -1,
                                .move_error = -1, .move_state = -1};
    for (int i = 0; i < MK2_AXES; i++) sample.joint[i] = NAN;
    if (robot_getNUM() != 1 || robot_getDOF(0) != MK2_AXES) return sample;
    GetCurrentJoint(sample.joint, 0);
    sample.power = GetRobotPowerState(0);
    sample.driver_error = IsRobotDriverError(0);
    sample.move_error = IsRobotMoveError(0);
    sample.move_state = get_robot_move_state(0);
    sample.valid = true;
    for (int i = 0; i < MK2_AXES; i++) if (!isfinite(sample.joint[i])) sample.valid = false;
    return sample;
}
static struct json_object *json_array7(const double *values)
{
    struct json_object *array = json_object_new_array_ext(MK2_AXES);
    for (int i = 0; i < MK2_AXES; i++) json_object_array_add(array, json_object_new_double(values[i]));
    return array;
}
static void publish_state(void *publisher, const struct mk2_core *core, const struct mk2_sample *sample, uint64_t sequence)
{
    if (!sample->valid) return; /* Never fabricate zero or nonfinite feedback. */
    struct json_object *root = json_object_new_object(), *metadata = json_object_new_object(), *robot = json_object_new_object();
    json_object_object_add(root, "Bridge", metadata); json_object_object_add(root, "Robot0", robot);
    json_object_object_add(metadata, "mode", json_object_new_string("control_guarded"));
    json_object_object_add(metadata, "commands_enabled", json_object_new_boolean(mk2_core_commands_available(core, sample)));
    json_object_object_add(metadata, "operator_enable_required", json_object_new_boolean(true));
    json_object_object_add(metadata, "automatic_power_changes", json_object_new_boolean(false));
    json_object_object_add(metadata, "control_state", json_object_new_string(mk2_phase_name(core->phase)));
    json_object_object_add(metadata, "fault", json_object_new_string(mk2_fault_name(core->fault)));
    json_object_object_add(metadata, "sdk_error", json_object_new_int(core->sdk_error));
    json_object_object_add(metadata, "sequence", json_object_new_uint64(sequence));
    json_object_object_add(metadata, "accepted_joints", json_object_new_uint64(core->accepted_joints));
    json_object_object_add(metadata, "rejected_frames", json_object_new_uint64(core->rejected_frames));
    json_object_object_add(robot, "Joint", json_array7(sample->joint));
    json_object_object_add(robot, "PowerState", json_object_new_int(sample->power));
    json_object_object_add(robot, "MoveState", json_object_new_int(sample->move_state));
    double velocity[MK2_AXES];
    for (int i = 0; i < MK2_AXES; i++) velocity[i] = NAN;
    const char *device = get_deviceName(0, NULL);
    const char *name = device ? get_name_robot_device(device, 0) : NULL;
    bool have_velocity = name && GetGroupVelocity(name, velocity) == 0;
    for (int i = 0; i < MK2_AXES; i++) if (!isfinite(velocity[i])) have_velocity = false;
    if (have_velocity) json_object_object_add(robot, "JointVelocity", json_array7(velocity));
    char frame[MK2_FRAME_MAX];
    int n = snprintf(frame, sizeof(frame), "State %s", json_object_to_json_string_ext(root, JSON_C_TO_STRING_PLAIN));
    if (n > 0 && (size_t)n < sizeof(frame)) zmq_send(publisher, frame, (size_t)n, ZMQ_DONTWAIT);
    json_object_put(root);
}
static void *run_bridge(void *unused)
{
    (void)unused;
    struct mk2_io io = {.start = sdk_start, .joint = sdk_joint, .stop = sdk_stop, .end = sdk_end};
    struct mk2_core core;
    if (mk2_core_init(&core, &policy, &io)) return NULL;
    void *context = zmq_ctx_new();
    if (!context) return NULL;
    void *publisher = zmq_socket(context, ZMQ_PUB), *subscriber = NULL;
    int linger = 0, send_hwm = 2, receive_hwm = 16;
    char endpoint[128];
    if (!publisher) { zmq_ctx_term(context); return NULL; }
    zmq_setsockopt(publisher, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(publisher, ZMQ_SNDHWM, &send_hwm, sizeof(send_hwm));
    snprintf(endpoint, sizeof(endpoint), "tcp://*:%d", policy.state_port);
    if (zmq_bind(publisher, endpoint)) {
        fprintf(stderr, "[MK2Control] Cannot bind state PUB; no servo operation started\n");
        zmq_close(publisher); zmq_ctx_term(context); return NULL;
    }
    if (policy.allow_commands) {
        subscriber = zmq_socket(context, ZMQ_SUB);
        snprintf(endpoint, sizeof(endpoint), "tcp://%s:%d", policy.command_host, policy.command_port);
        if (!subscriber || zmq_setsockopt(subscriber, ZMQ_LINGER, &linger, sizeof(linger)) ||
            zmq_setsockopt(subscriber, ZMQ_RCVHWM, &receive_hwm, sizeof(receive_hwm)) ||
            zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0) || zmq_connect(subscriber, endpoint)) {
            if (subscriber) zmq_close(subscriber);
            zmq_close(publisher); zmq_ctx_term(context); return NULL;
        }
    }
    fprintf(stderr, "[MK2Control] %s; NO automatic power/clear/recover/gripper operations\n",
            subscriber ? "command receiver configured; manual enable and valid hold required" : "commands LOCKED; state PUB only");
    fflush(stderr);
    uint64_t sequence = 0;
    double next_publish = monotonic_time();
    enum mk2_phase previous = core.phase;
    while (!atomic_load(&stopping)) {
        struct mk2_sample sample = read_sample();
        double now = monotonic_time();
        mk2_core_tick(&core, now, &sample);
        if (!isfinite(now)) { mk2_core_stop(&core, MK2_CLOCK_ERROR); break; }
        if (now >= next_publish) {
            publish_state(publisher, &core, &sample, ++sequence);
            next_publish = now + 0.01;
        }
        zmq_pollitem_t item = {.socket = subscriber, .events = ZMQ_POLLIN};
        int polled = zmq_poll(subscriber ? &item : NULL, subscriber ? 1 : 0, 5);
        if (polled < 0) { if (!atomic_load(&stopping)) mk2_core_stop(&core, MK2_STATE_ERROR); break; }
        if (subscriber && (item.revents & ZMQ_POLLIN)) {
            /* Bounded work per loop, preserve Switch ordering: never CONFLATE. */
            for (int count = 0; count < 32; count++) {
                zmq_msg_t message; zmq_msg_init(&message);
                int n = zmq_msg_recv(&message, subscriber, ZMQ_DONTWAIT);
                if (n < 0) { zmq_msg_close(&message); break; }
                sample = read_sample();
                bool multipart = zmq_msg_more(&message);
                if (multipart) mk2_core_handle(&core, NULL, 0, monotonic_time(), &sample);
                else mk2_core_handle(&core, zmq_msg_data(&message), zmq_msg_size(&message), monotonic_time(), &sample);
                zmq_msg_close(&message);
                if (multipart) {
                    /* Reject and permanently close this receiver rather than
                     * drain an unbounded multipart attack or execute fragments. */
                    zmq_close(subscriber); subscriber = NULL;
                    core.phase = MK2_FAULT; core.fault = core.sdk_error ? MK2_SDK_ERROR : MK2_BAD_FRAME;
                    break;
                }
            }
        }
        if (core.phase != previous) {
            fprintf(stderr, "[MK2Control] %s -> %s; fault=%s sdk_error=%d\n", mk2_phase_name(previous),
                    mk2_phase_name(core.phase), mk2_fault_name(core.fault), core.sdk_error);
            fflush(stderr); previous = core.phase;
        }
    }
    mk2_core_stop(&core, MK2_UNLOAD); /* Only if this plugin actually acquired servo. */
    if (subscriber) zmq_close(subscriber);
    zmq_close(publisher); zmq_ctx_term(context);
    return NULL;
}
void PluginMain(void)
{
    if (started || !correct_profile() || !load_policy()) {
        fprintf(stderr, "[MK2Control] Refusing duplicate/wrong-profile/unverified configuration\n");
        return;
    }
    atomic_store(&stopping, false);
    if (!pthread_create(&worker, NULL, run_bridge, NULL)) started = true;
    else fprintf(stderr, "[MK2Control] Cannot start bridge worker\n");
}
__attribute__((destructor)) static void unload_bridge(void)
{
    if (started) {
        atomic_store(&stopping, true); pthread_join(worker, NULL); started = false;
    }
}