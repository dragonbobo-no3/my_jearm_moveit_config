/* Standalone SDK SIMULATION probe; NOT a ROS/CMake target or controller plugin.
 * No link-time RobotControl dependency: refuse an unsafe environment BEFORE
 * dlopen/constructors. --init is read-only after initialization. Explicit
 * comparison modes operate ONLY the verified virtual robot; no clear/recover,
 * direct power, legacy plugin, or physical control mode exists.
 */
#define _GNU_SOURCE
#include "Base/RobotStruct.h"
#include <dlfcn.h>
#include <errno.h>
#include <float.h>
#include <limits.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define PROFILE "/tmp/mk2-probe/profile"
static const char expected_global[] =
    "---\nversion:\n  major: 12\n  minor: 2\n  build: 1\nserial: JE\n"
    "description: Isolated SDK initialization probe\n"
    "app_path: /tmp/mk2-probe/profile/app\nsimulation: true\nplugin_list: []\n";

static int fail(const char *reason)
{
    fprintf(stderr, "PROBE_REFUSED: %s; SDK not loaded\n", reason);
    return 77;
}
static int private_namespace(const char *name, const char *environment)
{
    char path[80], actual[128];
    const char *host = getenv(environment);
    if (!host || !*host) return 0;
    snprintf(path, sizeof(path), "/proc/self/ns/%s", name);
    ssize_t length = readlink(path, actual, sizeof(actual) - 1);
    if (length <= 0) return 0;
    actual[length] = '\0';
    if (!strcmp(host, actual)) return 0;
    printf("PROBE_NAMESPACE %s host=%s private=%s\n", name, host, actual);
    return 1;
}
static int safe_environment(void)
{
    static const char *names[] = {"mnt", "ipc", "net", "pid", "user"};
    static const char *variables[] = {"MK2_HOST_MNT", "MK2_HOST_IPC", "MK2_HOST_NET", "MK2_HOST_PID", "MK2_HOST_USER"};
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
        if (!private_namespace(names[i], variables[i])) return fail("missing/private namespace mismatch");
    struct statvfs root;
    struct statfs shm;
    if (statvfs("/", &root) || !(root.f_flag & ST_RDONLY) ||
        statfs("/dev/shm", &shm) || shm.f_type != 0x01021994)
        return fail("host root must be read-only and shared memory private tmpfs");
    if (access("/dev/EtherCAT0", F_OK) == 0 || access("/dev/bus/usb", F_OK) == 0 ||
        access("/home/robot/Work/system/robot_config_new_left/DeviceConfig.yaml", F_OK) == 0)
        return fail("physical devices or physical configuration visible");
    struct if_nameindex *interfaces = if_nameindex();
    if (!interfaces) return fail("cannot verify network isolation");
    int extra_interface = 0;
    for (struct if_nameindex *entry = interfaces; entry->if_index; entry++)
        if (strcmp(entry->if_name, "lo")) extra_interface = 1;
    if_freenameindex(interfaces);
    if (extra_interface) return fail("non-loopback network interface present");
    FILE *status = fopen("/proc/self/status", "r");
    if (!status) return fail("cannot verify dropped capabilities");
    char line[256];
    unsigned long long capabilities = ULLONG_MAX;
    while (fgets(line, sizeof(line), status))
        if (sscanf(line, "CapEff:\t%llx", &capabilities) == 1) break;
    fclose(status);
    if (capabilities != 0) return fail("capabilities were not dropped");
    char config[sizeof(expected_global) + 1];
    FILE *file = fopen(PROFILE "/GlobalConfig.yaml", "rb");
    if (!file) return fail("missing dedicated probe configuration");
    size_t count = fread(config, 1, sizeof(config), file);
    int error = ferror(file);
    fclose(file);
    /* A final LF is optional; no other spelling, field or plugin is allowed. */
    if (error || (count != sizeof(expected_global) - 1 && count != sizeof(expected_global) - 2) ||
        memcmp(config, expected_global, count))
        return fail("configuration must be exact simulation=true and empty plugin list");
    struct rlimit realtime, memory;
    if (getrlimit(RLIMIT_RTPRIO, &realtime) || getrlimit(RLIMIT_MEMLOCK, &memory))
        return fail("cannot inspect realtime resource limits");
    printf("PROBE_ISOLATION_OK root_readonly=1 hardware_hidden=1 plugins=0 capabilities=0 "
           "rtprio_soft=%llu memlock_soft=%llu\n",
           (unsigned long long)realtime.rlim_cur, (unsigned long long)memory.rlim_cur);
    return 0;
}

typedef int (*parser_function)(const char *, command_arg *);
typedef int (*initialize_function)(command_arg *);
typedef int (*getter_function)(void);
typedef int (*indexed_function)(int);
typedef void (*joint_getter_function)(double *, int);
typedef int (*servo_start_function)(double, double, int);
typedef int (*servo_joint_function)(robjoint *, double, int);
typedef int (*teach_joint_function)(int, int);
typedef void (*integer_setter_function)(int);
typedef void (*velocity_setter_function)(double);
typedef uint16_t (*protocol_string_function)(uint8_t *, uint16_t, const char *, const char *);
typedef uint16_t (*protocol_int_function)(uint8_t *, uint16_t, const char *, int);
typedef int (*protocol_check_function)(const uint8_t *, uint16_t);

/* POSIX dlsym function pointers, without an ISO-C object/function cast. */
#define LOAD(target, name) do { \
    void *symbol = dlsym(library, name); \
    _Static_assert(sizeof(target) == sizeof(symbol), "function pointer size"); \
    if (!symbol) { fprintf(stderr, "PROBE_SYMBOL_MISSING %s\n", name); fflush(NULL); _Exit(78); } \
    memcpy(&(target), &symbol, sizeof(target)); \
} while (0)

struct comparison_sdk {
    getter_function simulation, teach_enable, teach_stop, busy;
    indexed_function select_robot, power, state, driver_error, move_error, servo_end, stop_run;
    joint_getter_function actual, target;
    servo_start_function start;
    servo_joint_function joint;
    teach_joint_function teach;
    integer_setter_function move_thread, teach_type;
    velocity_setter_function teach_velocity;
};
static void pause_sample(void)
{
    struct timespec remaining = {.tv_nsec = 10000000};
    while (nanosleep(&remaining, &remaining) && errno == EINTR) {}
}
static int finite_number(double value)
{
    return value >= -DBL_MAX && value <= DBL_MAX;
}
static int dispatch_protocol_jog(void *library, const char *mode)
{
    indexed_function callback, coordinate;
    LOAD(coordinate, "set_robot_teach_coordinate");
    if (coordinate(0)) return -1;
    uint8_t bytes[8192] = {0};
    uint16_t length = 5;
    const int binary = strstr(mode, "--command-") != NULL;
    if (binary) {
        protocol_string_function put_string;
        protocol_int_function put_int;
        protocol_check_function check;
        LOAD(callback, "command_action"); LOAD(put_string, "SetProtocolDataString");
        LOAD(put_int, "SetProtocolDataInt"); LOAD(check, "IsProtocolRight");
        length = put_string(bytes, 0, "function", "robot_teach_move");
        length = put_int(bytes, length, "axis_index", 0);
        length = put_int(bytes, length, "dir", 1);
        if (!length || length > sizeof(bytes)) return -1;
        for (uint16_t offset = 0; offset < length;) {
            const int frame = check(bytes + offset, (uint16_t)(length - offset));
            if (frame <= 0 || frame > length - offset) return -1;
            offset = (uint16_t)(offset + frame);
        }
    } else {
        LOAD(callback, "btnevent_action");
        memcpy(bytes, "tv49!", length);
    }
    printf("PROBE_PROTOCOL_REQUEST callback=%s transport=private_socketpair bytes=%u hex=",
           binary ? "command_action" : "btnevent_action", (unsigned)length);
    for (uint16_t i = 0; i < length; i++) printf("%02x", bytes[i]);
    putchar('\n');
    int pair[2];
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, pair)) return -1;
    const struct timeval timeout = {.tv_sec = 1};
    if (setsockopt(pair[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) ||
        send(pair[0], bytes, length, MSG_NOSIGNAL) != length || shutdown(pair[0], SHUT_WR)) {
        close(pair[0]); close(pair[1]); return -1;
    }
    const int consumed = callback(pair[1]);
    close(pair[0]); close(pair[1]);
    printf("PROBE_PROTOCOL_DISPATCH received=%d expected=%u; NOT a motion return value\n",
           consumed, (unsigned)length);
    return consumed == length ? 0 : -1;
}
static int sample_comparison(const struct comparison_sdk *sdk, const char *stage,
                             int tick, const robjoint *initial)
{
    double actual[ROBOT_MAX_DOF] = {0}, target[ROBOT_MAX_DOF] = {0};
    sdk->actual(actual, 0); sdk->target(target, 0);
    const int state = sdk->state(0), power = sdk->power(0);
    const int driver = sdk->driver_error(0), error = sdk->move_error(0);
    double excursion = 0;
    for (int i = 0; i < 7; i++) {
        if (!finite_number(actual[i]) || !finite_number(target[i]) || !finite_number(initial->angle[i])) return -1;
        double delta = actual[i] - initial->angle[i];
        if (delta < 0) delta = -delta;
        if (delta > excursion) excursion = delta;
    }
    /* Sequential read-only samples, NOT an atomic SDK snapshot. */
    printf("PROBE_SAMPLE stage=%s tick=%d state=%d power=%d driver=%d move_error=%d busy=%d "
           "q0=%.12g target0=%.12g excursion=%.12g\n",
           stage, tick, state, power, driver, error, sdk->busy(), actual[0], target[0], excursion);
    return sdk->simulation() && power == 1 && !driver && !error && state >= 0 && state != 7 && excursion <= .02 ? 0 : -1;
}
static int compare_servo_jog(void *library, const char *mode)
{
    struct comparison_sdk sdk;
    LOAD(sdk.simulation, "GetSystemSimulationState"); LOAD(sdk.select_robot, "set_robot_index");
    LOAD(sdk.teach_enable, "robot_teach_enable"); LOAD(sdk.teach_stop, "robot_teach_stop");
    LOAD(sdk.power, "GetRobotPowerState"); LOAD(sdk.state, "get_robot_move_state");
    LOAD(sdk.driver_error, "IsRobotDriverError"); LOAD(sdk.move_error, "IsRobotMoveError");
    LOAD(sdk.actual, "GetCurrentJoint"); LOAD(sdk.target, "GetCurrentTargetJoint");
    LOAD(sdk.start, "ServoStart"); LOAD(sdk.joint, "ServoJoint"); LOAD(sdk.servo_end, "ServoEnd");
    LOAD(sdk.stop_run, "RobotStopRun"); LOAD(sdk.teach, "robot_teach_joint");
    LOAD(sdk.move_thread, "setMoveThread"); LOAD(sdk.teach_type, "set_teach_move_type");
    LOAD(sdk.teach_velocity, "set_robot_teach_velocity"); LOAD(sdk.busy, "get_robot_busy_state");
    if (!sdk.simulation() || sdk.select_robot(0)) return 79;
    puts("PROBE_VIRTUAL_MOTION_ONLY no physical interfaces or external command source");
    sdk.move_thread(1);
    sdk.teach_type(0); /* Continuous joint jog, bounded below by experiment duration. */
    sdk.teach_velocity(.01);
    const int enabled = sdk.teach_enable();
    printf("PROBE_ENABLE virtual_only=1 result=%d\n", enabled);
    if (enabled) return 79;
    for (int i = 0; i < 50 && sdk.power(0) != 1; i++) pause_sample();
    robjoint hold = {.dof = 7};
    sdk.actual(hold.angle, 0);
    if (sample_comparison(&sdk, "enabled", 0, &hold)) return 79;
    bool started = false, did_jog = false;
    int result = sdk.start(1, .0001, 0), outcome = 0;
    printf("PROBE_SERVO_START result=%d\n", result);
    started = true; /* Even failed Start can have partial effects. */
    if (result) { outcome = 79; goto cleanup; }
    for (int tick = 1; tick <= 20; tick++) {
        result = sdk.joint(&hold, tick * .01, 0);
        if (result || tick == 1 || tick == 20)
            printf("PROBE_HOLD tick=%d result=%d state=%d\n", tick, result, sdk.state(0));
        if (result) { outcome = 79; goto cleanup; }
        pause_sample();
    }
    if (sample_comparison(&sdk, "holding_before_jog", 20, &hold) || sdk.state(0) != 13) {
        outcome = 79; goto cleanup;
    }
    if (strstr(mode, "-end-jog")) {
        if (!strcmp(mode, "--stop-end-jog")) {
            result = sdk.stop_run(0);
            printf("PROBE_PRE_JOG_STOP_RUN result=%d state=%d\n", result, sdk.state(0));
            if (result) { outcome = 79; goto cleanup; }
            for (int tick = 0; tick < 50 && sdk.state(0) != 0; tick++) pause_sample();
        }
        result = sdk.servo_end(0);
        printf("PROBE_PRE_JOG_SERVO_END result=%d state=%d\n", result, sdk.state(0));
        if (result) { outcome = 79; goto cleanup; }
        started = false;
        for (int tick = 0; tick < 10; tick++) pause_sample();
    }
    if (sample_comparison(&sdk, "before_jog_call", 0, &hold)) { outcome = 79; goto cleanup; }
        const int protocol_mode = strstr(mode, "--button-") || strstr(mode, "--command-");
        result = protocol_mode ? dispatch_protocol_jog(library, mode) : sdk.teach(0, 1);
    did_jog = true;
        printf("PROBE_JOG_REQUEST mode=%s result=%d state=%d motion_return=%d\n",
            mode, result, sdk.state(0), !protocol_mode);
        if (protocol_mode && result) { outcome = 79; goto cleanup; }
    int previous_result = INT_MIN;
    for (int tick = 21; tick <= 70; tick++) {
        if (started) {
            result = sdk.joint(&hold, tick * .01, 0);
            if (result != previous_result || tick == 70)
                printf("PROBE_HOLD_AFTER_JOG tick=%d result=%d state=%d\n", tick, result, sdk.state(0));
            previous_result = result;
        }
        if (tick % 5 == 0 && sample_comparison(&sdk, "after_jog", tick, &hold)) {
            outcome = 79; goto cleanup;
        }
        pause_sample();
    }
cleanup:
    /* Explicit experiment cleanup, not a production priority implementation.
     * Do not recover/replay if a request failed or the SDK entered a fault. */
    printf("PROBE_CLEANUP stop_run_result=%d\n", sdk.stop_run(0));
    if (started) printf("PROBE_CLEANUP servo_end_result=%d\n", sdk.servo_end(0));
    for (int tick = 0; tick < 10; tick++) pause_sample();
    printf("PROBE_COMPARE_FINISHED mode=%s outcome=%d jog_called=%d state=%d\n",
           mode, outcome, did_jog, sdk.state(0));
    return outcome;
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (argc != 2 || (strcmp(argv[1], "--check") && strcmp(argv[1], "--init") &&
        strcmp(argv[1], "--hold-jog") && strcmp(argv[1], "--end-jog") && strcmp(argv[1], "--stop-end-jog") &&
        strcmp(argv[1], "--button-jog") && strcmp(argv[1], "--button-end-jog") &&
        strcmp(argv[1], "--command-jog") && strcmp(argv[1], "--command-end-jog")))
        return fail("unknown probe mode");
    int result = safe_environment();
    if (result || !strcmp(argv[1], "--check")) return result;
    /* Cap uninterrupted realtime CPU use inside this disposable process. */
    const struct rlimit realtime_budget = {.rlim_cur = 100000, .rlim_max = 500000};
    if (setrlimit(RLIMIT_RTTIME, &realtime_budget)) return fail("cannot bound realtime CPU use");

    puts("PROBE_LOADING_SDK_AFTER_ISOLATION_CHECK");
    void *library = dlopen("/lib/libRobotControl.so", RTLD_NOW | RTLD_LOCAL);
    if (!library) {
        fprintf(stderr, "PROBE_LOAD_FAILED %s\n", dlerror());
        fflush(NULL); _Exit(78);
    }
    parser_function parse;
    initialize_function initialize;
    getter_function simulation, robot_count;
    indexed_function dof, power, state;
    LOAD(parse, "commandLineParser1");
    LOAD(initialize, "system_initialize");
    LOAD(simulation, "GetSystemSimulationState");
    LOAD(robot_count, "robot_getNUM");
    LOAD(dof, "robot_getDOF");
    LOAD(power, "GetRobotPowerState");
    LOAD(state, "get_robot_move_state");
    command_arg argument = {0};
    result = parse("--path " PROFILE, &argument);
    printf("PROBE_PARSE result=%d\n", result);
    if (result || !argument.path || strcmp(argument.path, PROFILE)) { fflush(NULL); _Exit(78); }
    errno = 0;
    result = initialize(&argument);
    printf("PROBE_INITIALIZE result=%d errno=%d\n", result, errno);
    if (result) { fflush(NULL); _Exit(78); }
    const int simulated = simulation(), count = robot_count();
    if (!simulated || count != 1 || dof(0) != 7) {
        fprintf(stderr, "PROBE_UNEXPECTED_MODEL simulation=%d robots=%d\n", simulated, count);
        fflush(NULL); _Exit(79);
    }
    printf("PROBE_INIT_OK simulation=%d axes=7 power=%d state=%d; NO enable/motion issued\n",
           simulated, power(0), state(0));
    if (strcmp(argv[1], "--init")) {
        result = compare_servo_jog(library, argv[1]);
        fflush(NULL); _Exit(result);
    }
    /* No documented system teardown API. Exit the private PID namespace;
     * never dlclose a library with possibly running SDK threads. */
    fflush(NULL); _Exit(0);
}