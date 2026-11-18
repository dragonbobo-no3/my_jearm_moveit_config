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
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zmq.h>

#ifndef PROFILE_ROOT
#define PROFILE_ROOT "/home/robot/Work/system/robot_config_new_left"
#endif
#ifndef MK2_SIMULATION_ONLY
#define MK2_SIMULATION_ONLY 0
#endif
#ifndef MK2_PHYSICAL_CONTROL
#define MK2_PHYSICAL_CONTROL 0
#endif
/* Commissioning opt-in; NEVER enabled by existing physical build scripts. */
#ifndef MK2_PHYSICAL_HANDOVER
#define MK2_PHYSICAL_HANDOVER 0
#endif
#if MK2_PHYSICAL_HANDOVER && !MK2_PHYSICAL_CONTROL
#error "Physical handover requires the verified physical control build"
#endif
#if MK2_SIMULATION_ONLY && MK2_PHYSICAL_CONTROL
#error "Simulation isolation and physical control are mutually exclusive builds"
#endif
static atomic_bool stopping = ATOMIC_VAR_INIT(false);
static bool started;
static pthread_t worker;
static struct mk2_policy policy;

#if defined(JESERVER_V2_PLUGIN)
static bool je2_simulation_guard(void);
#endif
static bool simulation_isolated(void)
{
#if defined(JESERVER_V2_PLUGIN)
    return je2_simulation_guard();
#elif MK2_SIMULATION_ONLY
    char own[128], host[128], line[1024];
    ssize_t a = readlink("/proc/self/ns/mnt", own, sizeof(own));
    ssize_t b = readlink("/proc/1/ns/mnt", host, sizeof(host));
    struct stat blocked, null_device;
    if (a <= 0 || b <= 0 || (a == b && !memcmp(own, host, (size_t)a)) ||
        stat("/dev/EtherCAT0", &blocked) || stat("/dev/null", &null_device) ||
        !S_ISCHR(blocked.st_mode) || blocked.st_rdev != null_device.st_rdev)
        return false;
    FILE *file = fopen(PROFILE_ROOT "/GlobalConfig.yaml", "r");
    if (!file) return false;
    int enabled = 0;
    while (fgets(line, sizeof(line), file)) {
        if (!strncmp(line, "simulation:", 11)) {
            if (strcmp(line, "simulation: true\n")) { fclose(file); return false; }
            enabled++;
        }
    }
    bool ok = !ferror(file) && enabled == 1;
    fclose(file);
    return ok;
#else
    return true;
#endif
}

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
static bool file_hash_matches(const char *path, const char *expected)
{
    FILE *file = fopen(path, "rb");
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
    return !strcmp(hexadecimal, expected);
}
static bool configuration_hash_matches(void)
{
    return file_hash_matches(PROFILE_ROOT "/DeviceConfig.yaml", policy.device_config_sha256);
}
static bool physical_environment_verified(void)
{
#if MK2_PHYSICAL_CONTROL
    /* No implicit physical fallback. Require the original launcher's inherited
     * session lock, actual EtherCAT device, non-simulation profile and pinned
     * user-tuned configuration. This is NOT a certified safety interlock. */
    const char *device_sha = "a542dc92c51e2822e78dcfc385a90f925a909ebd70072fab0b5ab75d4e0c2bab";
    const char *bus_sha = "425db50795d4e2e9fc44e4005e7dcf797e677c7d4fa87a135236a449a5f6ab84";
    struct stat device, null_device, lock, inherited;
    if (strcmp(PROFILE_ROOT, "/home/robot/Work/system/robot_config_new_left") ||
        strcmp(policy.device_config_sha256, device_sha) ||
        stat("/dev/EtherCAT0", &device) || stat("/dev/null", &null_device) ||
        !S_ISCHR(device.st_mode) || device.st_rdev == null_device.st_rdev ||
        stat("/run/lock/robotmain-new-left.lock", &lock) || fstat(9, &inherited) ||
        !S_ISREG(lock.st_mode) || lock.st_uid != 0 || (lock.st_mode & (S_IWGRP | S_IWOTH)) ||
        lock.st_dev != inherited.st_dev || lock.st_ino != inherited.st_ino ||
        flock(9, LOCK_EX | LOCK_NB)) return false;
    FILE *file = fopen(PROFILE_ROOT "/GlobalConfig.yaml", "r");
    if (!file) return false;
    char line[1024];
    int disabled = 0;
    while (fgets(line, sizeof(line), file)) {
        if (!strncmp(line, "simulation:", 11)) {
            if (strcmp(line, "simulation: false\n")) { fclose(file); return false; }
            disabled++;
        }
    }
    bool ok = !ferror(file) && disabled == 1;
    fclose(file);
    return ok && configuration_hash_matches() && file_hash_matches(PROFILE_ROOT "/BusConfig.yaml", bus_sha);
#else
    return false;
#endif
}
static bool backend_verified(void)
{
    return MK2_SIMULATION_ONLY ? simulation_isolated() : physical_environment_verified();
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
#if MK2_PHYSICAL_CONTROL
/* Gen-1-style activation recovery: clears ONLY a latched forced-stop (observed
 * SDK move_state 7) once at first-hold servo start. Never a power cycle, error
 * clear or driver/move fault reset; genuine faults are rejected before start. */
static int sdk_recover(void *unused) { (void)unused; return RobotStopRecover(0); }
#endif
#if (MK2_SIMULATION_ONLY || MK2_PHYSICAL_HANDOVER) && !defined(JESERVER_V2_PLUGIN)
static int sdk_stop_run(void *unused) { (void)unused; return RobotStopRun(0); }
#endif
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
static void json_add_observed_sample(struct json_object *object, const struct mk2_sample *sample)
{
    json_object_object_add(object, "sample_valid", json_object_new_boolean(sample->valid));
    json_object_object_add(object, "PowerState", json_object_new_int(sample->power));
    json_object_object_add(object, "DriverError", json_object_new_int(sample->driver_error));
    json_object_object_add(object, "MoveError", json_object_new_int(sample->move_error));
    json_object_object_add(object, "MoveState", json_object_new_int(sample->move_state));
    struct json_object *joint = json_object_new_array_ext(MK2_AXES);
    for (int i = 0; i < MK2_AXES; i++)
        json_object_array_add(joint, isfinite(sample->joint[i]) ? json_object_new_double(sample->joint[i]) : NULL);
    json_object_object_add(object, "Joint", joint);
}
static struct json_object *json_servo_observation(const struct mk2_servo_observation *observation)
{
    struct json_object *object = json_object_new_object();
    json_object_object_add(object, "ownership_evidence", json_object_new_boolean(false));
    json_object_object_add(object, "sdk_state_reference", json_object_new_int(MK2_OBSERVED_SDK_SERVO_STATE));
    json_object_object_add(object, "seen_servo_state", json_object_new_boolean(observation->seen_servo_state));
    if (observation->first_nonservo_captured) {
        struct json_object *event = json_object_new_object();
        json_object_object_add(event, "servo_state_seen_before",
            json_object_new_boolean(observation->servo_state_seen_before));
        json_object_object_add(event, "input_monotonic_time", isfinite(observation->input_monotonic_time) ?
            json_object_new_double(observation->input_monotonic_time) : NULL);
        json_add_observed_sample(event, &observation->first_nonservo_sample);
        json_object_object_add(object, "first_active_nonservo_sample", event);
    }
    if (observation->last_call.call != MK2_SDK_CALL_NONE) {
        const struct mk2_sdk_call_observation *last = &observation->last_call;
        struct json_object *event = json_object_new_object(), *input = json_object_new_object();
        json_object_object_add(event, "call", json_object_new_string(mk2_sdk_call_name(last->call)));
        json_object_object_add(event, "phase", json_object_new_string(mk2_phase_name(last->phase)));
        json_object_object_add(event, "result", json_object_new_int(last->result));
        json_object_object_add(event, "input_monotonic_time", isfinite(last->input_monotonic_time) ?
            json_object_new_double(last->input_monotonic_time) : NULL);
        json_object_object_add(event, "command_time", isfinite(last->command_time) ?
            json_object_new_double(last->command_time) : NULL);
        json_add_observed_sample(input, &last->input_sample);
        json_object_object_add(event, "command_validation_sample", input);
        json_object_object_add(object, "last_sdk_call", event);
    }
    return object;
}
static struct json_object *json_state_failure(const struct mk2_state_failure *failure)
{
    if (!failure->captured) return NULL;
    struct json_object *object = json_object_new_object(), *checks = json_object_new_array();
    json_object_object_add(object, "site", json_object_new_string(failure->site));
    json_object_object_add(object, "phase_before_stop", json_object_new_string(mk2_phase_name(failure->phase)));
    json_object_object_add(object, "monotonic_time", isfinite(failure->monotonic_time) ?
        json_object_new_double(failure->monotonic_time) : NULL);
    json_object_object_add(object, "have_sample", json_object_new_boolean(failure->have_sample));
    const struct { unsigned flag; const char *name; } names[] = {
        {MK2_CHECK_INVALID_SAMPLE, "invalid_sample"}, {MK2_CHECK_NONFINITE, "nonfinite_joint"},
        {MK2_CHECK_POWER, "power_not_enabled"}, {MK2_CHECK_DRIVER, "driver_error"},
        {MK2_CHECK_MOVE_ERROR, "move_error"}, {MK2_CHECK_MOVE_STATE, "invalid_or_forced_stop_move_state"},
        {MK2_CHECK_BOUNDS, "position_out_of_bounds"}, {MK2_CHECK_FIRST_NOT_IDLE, "waiting_first_joint_not_idle"},
        {MK2_CHECK_COMMAND_TIME, "nonpositive_command_time"}
    };
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
        if (failure->failed_checks & names[i].flag)
            json_object_array_add(checks, json_object_new_string(names[i].name));
    json_object_object_add(object, "failed_checks", checks);
    if (failure->have_sample) json_add_observed_sample(object, &failure->sample);
    if (failure->have_command_time)
        json_object_object_add(object, "command_time", isfinite(failure->command_time) ?
            json_object_new_double(failure->command_time) : NULL);
    return object;
}
static void log_state_failure(const struct mk2_core *core, bool *logged)
{
    if (*logged || core->phase != MK2_FAULT) return;
    if (core->first_state_error.captured) {
        struct json_object *failure = json_state_failure(&core->first_state_error);
        fprintf(stderr, "[MK2Control] first_state_error=%s\n",
            json_object_to_json_string_ext(failure, JSON_C_TO_STRING_PLAIN));
        json_object_put(failure);
    }
    struct json_object *observation = json_servo_observation(&core->servo_observation);
    fprintf(stderr, "[MK2Control] servo_observation=%s\n",
        json_object_to_json_string_ext(observation, JSON_C_TO_STRING_PLAIN));
    fflush(stderr);
    json_object_put(observation);
    *logged = true;
}
static void publish_state(void *publisher, const struct mk2_core *core, const struct mk2_sample *sample, uint64_t sequence)
{
    if (!sample->valid) return; /* Never fabricate zero or nonfinite feedback. */
    struct json_object *root = json_object_new_object(), *metadata = json_object_new_object(), *robot = json_object_new_object();
    json_object_object_add(root, "Bridge", metadata); json_object_object_add(root, "Robot0", robot);
    json_object_object_add(metadata, "mode", json_object_new_string("control_guarded"));
    json_object_object_add(metadata, "backend", json_object_new_string(
        MK2_SIMULATION_ONLY ? "sdk_simulation" : "physical"));
    json_object_object_add(metadata, "simulation_only", json_object_new_boolean(MK2_SIMULATION_ONLY));
    json_object_object_add(metadata, "simulation_isolated", json_object_new_boolean(
        MK2_SIMULATION_ONLY && simulation_isolated()));
    /* Only published after startup and the current loop's backend checks pass. */
    json_object_object_add(metadata, "configuration_verified", json_object_new_boolean(true));
    json_object_object_add(metadata, "profile", json_object_new_string(PROFILE_ROOT));
    json_object_object_add(metadata, "device_config_sha256", json_object_new_string(policy.device_config_sha256));
    json_object_object_add(metadata, "commands_enabled", json_object_new_boolean(mk2_core_commands_available(core, sample)));
    json_object_object_add(metadata, "clean_release_supported", json_object_new_boolean(
        core->io.stop_run != NULL));
    json_object_object_add(metadata, "ownership_handover_supported", json_object_new_boolean(
        core->io.stop_run != NULL));
    json_object_object_add(metadata, "operator_enable_required", json_object_new_boolean(true));
    json_object_object_add(metadata, "automatic_power_changes", json_object_new_boolean(false));
    json_object_object_add(metadata, "control_state", json_object_new_string(mk2_phase_name(core->phase)));
    json_object_object_add(metadata, "fault", json_object_new_string(mk2_fault_name(core->fault)));
    if (core->first_state_error.captured)
        json_object_object_add(metadata, "first_state_error", json_state_failure(&core->first_state_error));
    json_object_object_add(metadata, "servo_observation", json_servo_observation(&core->servo_observation));
    json_object_object_add(metadata, "sdk_error", json_object_new_int(core->sdk_error));
    json_object_object_add(metadata, "sequence", json_object_new_uint64(sequence));
    json_object_object_add(metadata, "accepted_joints", json_object_new_uint64(core->accepted_joints));
    json_object_object_add(metadata, "rejected_frames", json_object_new_uint64(core->rejected_frames));
    json_object_object_add(robot, "Joint", json_array7(sample->joint));
    json_object_object_add(robot, "PowerState", json_object_new_int(sample->power));
    json_object_object_add(robot, "MoveState", json_object_new_int(sample->move_state));
    json_object_object_add(robot, "DriverError", json_object_new_int(sample->driver_error));
    json_object_object_add(robot, "MoveError", json_object_new_int(sample->move_error));
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
#if defined(JESERVER_V2_PLUGIN)
#include "jeserver_v2_runtime.inc"
#else
static void *run_bridge(void *unused)
{
    (void)unused;
    struct mk2_io io = {.start = sdk_start, .joint = sdk_joint, .stop = sdk_stop, .end = sdk_end};
#if MK2_SIMULATION_ONLY || MK2_PHYSICAL_HANDOVER
    io.stop_run = sdk_stop_run;
#endif
#if MK2_PHYSICAL_CONTROL
    io.recover = sdk_recover;
#endif
    struct mk2_core core;
    if (mk2_core_init(&core, &policy, &io)) return NULL;
#if MK2_PHYSICAL_CONTROL
    /* PC exit/stream loss makes the SDK force-stop (7) within one servo cycle,
     * before any Switch false can arrive, and our own Switch-false RobotStop
     * also leaves 7 latched. Allow ONLY those to return to READY after
     * sustained healthy idle (one RobotStopRecover for our own latch); see
     * mk2_core_tick. */
    core.rearm_after_forced_stop = true;
#endif
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
    fprintf(stderr, "[MK2Control] %s; %s; %s; NO automatic power/clear/gripper operations\n",
            subscriber ? "command receiver configured; manual enable and valid hold required" : "commands LOCKED; state PUB only",
            core.io.recover ? "activation forced-stop recovery enabled" : "no activation recovery",
            core.rearm_after_forced_stop ? "stop/forced-stop rearm after 1 s healthy idle" : "faults latched until restart");
    fflush(stderr);
    uint64_t sequence = 0;
    double next_publish = monotonic_time();
    enum mk2_phase previous = core.phase;
    uint64_t previous_recovers = core.rearm_recovers;
    bool state_failure_logged = false;
    while (!atomic_load(&stopping)) {
        if (!backend_verified()) {
            fprintf(stderr, "[MK2Control] Backend isolation/profile/configuration/lock verification lost; stopping owned servo\n");
            mk2_core_record_state_error(&core, "backend_verification", monotonic_time(), NULL, false, 0);
            mk2_core_stop(&core, MK2_STATE_ERROR);
            break;
        }
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
        if (polled < 0) {
            if (!atomic_load(&stopping)) {
                mk2_core_record_state_error(&core, "transport_poll", monotonic_time(), &sample, false, 0);
                mk2_core_stop(&core, MK2_STATE_ERROR);
            }
            break;
        }
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
                    core.rearm_after_forced_stop = false;
                    core.phase = MK2_FAULT; core.fault = core.sdk_error ? MK2_SDK_ERROR : MK2_BAD_FRAME;
                    break;
                }
            }
        }
        if (core.rearm_recovers != previous_recovers) {
            fprintf(stderr, "[MK2Control] own stop latch (MoveState 7) idle >= 1 s: RobotStopRecover #%llu result=%d; "
                    "waiting for MoveState 0 before rearm\n", (unsigned long long)core.rearm_recovers, core.sdk_error);
            fflush(stderr); previous_recovers = core.rearm_recovers;
        }
        if (core.phase != previous) {
            fprintf(stderr, "[MK2Control] %s -> %s; fault=%s sdk_error=%d\n", mk2_phase_name(previous),
                    mk2_phase_name(core.phase), mk2_fault_name(core.fault), core.sdk_error);
            if (core.phase == MK2_READY && (previous == MK2_STOPPED || previous == MK2_FAULT)) {
                fprintf(stderr, "[MK2Control] rearmed #%llu after healthy idle; next session needs fresh "
                        "preflight, Switch and first hold\n", (unsigned long long)core.rearm_count);
                state_failure_logged = false;
            }
            fflush(stderr); previous = core.phase;
        }
        log_state_failure(&core, &state_failure_logged);
    }
    log_state_failure(&core, &state_failure_logged);
    mk2_core_stop(&core, MK2_UNLOAD); /* Only if this plugin actually acquired servo. */
    if (subscriber) zmq_close(subscriber);
    zmq_close(publisher); zmq_ctx_term(context);
    return NULL;
}
#endif
void PluginMain(void)
{
    if (started || !correct_profile() || !load_policy() || !backend_verified()) {
#if defined(JESERVER_V2_PLUGIN)
        if (!started) atomic_store(&je2_plugin_status, -1);
#endif
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