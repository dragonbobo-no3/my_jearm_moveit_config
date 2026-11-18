#ifndef MK2_CONTROL_CORE_H
#define MK2_CONTROL_CORE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MK2_AXES 7
#define MK2_FRAME_MAX 4096
#define MK2_REARM_SETTLE_SECONDS 1.0
#define MK2_REARM_STATIONARY_RAD 1e-3
enum mk2_phase { MK2_LOCKED, MK2_READY, MK2_WAIT_FIRST, MK2_ACTIVE, MK2_STOPPED, MK2_FAULT, MK2_RELEASING };
enum mk2_fault {
    MK2_NO_FAULT, MK2_BAD_FRAME, MK2_NOT_READY, MK2_BAD_TIME, MK2_LIMIT,
    MK2_FIRST_TARGET, MK2_FOLLOWING, MK2_VELOCITY, MK2_ACCELERATION,
    MK2_WATCHDOG, MK2_SDK_ERROR, MK2_STATE_ERROR, MK2_CLOCK_ERROR, MK2_UNLOAD
};
struct mk2_policy {
    bool allow_commands;
    char command_host[64], device_config_sha256[65];
    int command_port, state_port;
    double watchdog, first_timeout, min_dt, max_dt, clock_skew;
    double first_hold_tolerance, following_tolerance, servo_q, servo_r;
    double lower[MK2_AXES], upper[MK2_AXES], velocity[MK2_AXES], acceleration[MK2_AXES];
};
struct mk2_sample {
    bool valid;
    int power, driver_error, move_error, move_state;
    double joint[MK2_AXES];
};
enum mk2_state_check {
    MK2_CHECK_INVALID_SAMPLE = 1u << 0, MK2_CHECK_NONFINITE = 1u << 1,
    MK2_CHECK_POWER = 1u << 2, MK2_CHECK_DRIVER = 1u << 3,
    MK2_CHECK_MOVE_ERROR = 1u << 4, MK2_CHECK_MOVE_STATE = 1u << 5,
    MK2_CHECK_BOUNDS = 1u << 6, MK2_CHECK_FIRST_NOT_IDLE = 1u << 7,
    MK2_CHECK_COMMAND_TIME = 1u << 8
};
/* Diagnostic only: captured BEFORE stop/end can change the SDK feedback. */
struct mk2_state_failure {
    bool captured, have_sample, have_command_time;
    const char *site; /* Static literal, never a borrowed frame buffer. */
    enum mk2_phase phase;
    unsigned failed_checks;
    double monotonic_time, command_time;
    struct mk2_sample sample;
};
/* Observations only, NOT ownership evidence or a reason to bypass a stop.
 * State 13 is observed in the pinned SDK's ServoJoint implementation, not a
 * public cross-version promise that its execution thread is alive/exited. */
#define MK2_OBSERVED_SDK_SERVO_STATE 13
enum mk2_sdk_call { MK2_SDK_CALL_NONE, MK2_SDK_CALL_RECOVER, MK2_SDK_CALL_START, MK2_SDK_CALL_JOINT };
struct mk2_sdk_call_observation {
    enum mk2_sdk_call call;
    enum mk2_phase phase;
    int result;
    double input_monotonic_time, command_time;
    struct mk2_sample input_sample;
};
struct mk2_servo_observation {
    bool seen_servo_state, first_nonservo_captured, servo_state_seen_before;
    double input_monotonic_time;
    struct mk2_sample first_nonservo_sample;
    struct mk2_sdk_call_observation last_call;
};
/* All mutation is confined to these callbacks. The test implementation has no
 * dependency on libRobotControl or any physical robot. Stop must NOT power off. */
struct mk2_io {
    void *context;
    int (*start)(void *, double q, double r);
    int (*joint)(void *, const double *, double data_time);
    int (*stop)(void *);
    int (*end)(void *);
    /* Simulation or explicitly commissioned physical-handover opt-in ONLY.
     * Default physical builds leave NULL. Never a fault reset. */
    int (*stop_run)(void *);
    /* Clears ONLY a latched forced-stop (observed SDK move_state 7) once, at
     * first-hold servo start. NOT a power cycle, error clear or driver/move
     * fault reset; a genuine fault is still rejected by the first-frame checks.
     * NULL where no activation recovery is wired (tests, sim, handover). */
    int (*recover)(void *);
};
struct mk2_core {
    struct mk2_policy policy;
    struct mk2_io io;
    enum mk2_phase phase;
    enum mk2_fault fault;
    bool owns_servo;
    /* Opt-in (default false). See mk2_core_tick: returns ONLY a clean stop or
     * a sole active-servo forced stop (SDK move_state 7, e.g. PC stream loss)
     * to READY after sustained healthy idle feedback. The only SDK call is one
     * io.recover clearing this plugin's OWN RobotStop latch (stop_latch_ours). */
    bool rearm_after_forced_stop;
    bool stop_latch_ours, rearm_recovered;
    uint64_t rearm_count, rearm_recovers;
    double rearm_since, rearm_joint[MK2_AXES];
    int sdk_error;
    uint64_t accepted_joints, rejected_frames;
    double last_receive, first_receive, first_data_time, last_data_time;
    double last_target[MK2_AXES], last_velocity[MK2_AXES];
    double release_start, release_stable_since, release_joint[MK2_AXES];
    struct mk2_state_failure first_state_error;
    struct mk2_servo_observation servo_observation;
};
int mk2_policy_parse(const char *, size_t, struct mk2_policy *, char *error, size_t);
int mk2_core_init(struct mk2_core *, const struct mk2_policy *, const struct mk2_io *);
bool mk2_core_commands_available(const struct mk2_core *, const struct mk2_sample *);
int mk2_core_handle(struct mk2_core *, const char *, size_t, double now,
                    const struct mk2_sample *);
void mk2_core_tick(struct mk2_core *, double now, const struct mk2_sample *);
void mk2_core_stop(struct mk2_core *, enum mk2_fault);
void mk2_core_record_state_error(struct mk2_core *, const char *site, double now,
                                const struct mk2_sample *, bool have_command_time, double command_time);
const char *mk2_phase_name(enum mk2_phase);
const char *mk2_fault_name(enum mk2_fault);
const char *mk2_sdk_call_name(enum mk2_sdk_call);
#endif