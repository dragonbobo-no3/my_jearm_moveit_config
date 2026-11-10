#ifndef MK2_CONTROL_CORE_H
#define MK2_CONTROL_CORE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MK2_AXES 7
#define MK2_FRAME_MAX 4096
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
/* All mutation is confined to these callbacks. The test implementation has no
 * dependency on libRobotControl or any physical robot. Stop must NOT power off. */
struct mk2_io {
    void *context;
    int (*start)(void *, double q, double r);
    int (*joint)(void *, const double *, double data_time);
    int (*stop)(void *);
    int (*end)(void *);
    /* Optional SIMULATION-ONLY clean release. Never a fault reset. */
    int (*stop_run)(void *);
};
struct mk2_core {
    struct mk2_policy policy;
    struct mk2_io io;
    enum mk2_phase phase;
    enum mk2_fault fault;
    bool owns_servo;
    int sdk_error;
    uint64_t accepted_joints, rejected_frames;
    double last_receive, first_receive, first_data_time, last_data_time;
    double last_target[MK2_AXES], last_velocity[MK2_AXES];
    double release_start, release_stable_since, release_joint[MK2_AXES];
};
int mk2_policy_parse(const char *, size_t, struct mk2_policy *, char *error, size_t);
int mk2_core_init(struct mk2_core *, const struct mk2_policy *, const struct mk2_io *);
bool mk2_core_commands_available(const struct mk2_core *, const struct mk2_sample *);
int mk2_core_handle(struct mk2_core *, const char *, size_t, double now,
                    const struct mk2_sample *);
void mk2_core_tick(struct mk2_core *, double now, const struct mk2_sample *);
void mk2_core_stop(struct mk2_core *, enum mk2_fault);
const char *mk2_phase_name(enum mk2_phase);
const char *mk2_fault_name(enum mk2_fault);
#endif