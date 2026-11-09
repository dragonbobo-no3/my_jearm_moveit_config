/* Vendor RobotMain plugin: receive-only commissioning from the PC's viewpoint.
 * This module ONLY reads SDK state and publishes it. There is deliberately no
 * command SUB socket, serial/gripper access, power/calibration/motion API call.
 * Build separately on the controller against its vendor SDK (not ROS CMake).
 */
#define _POSIX_C_SOURCE 200809L
#include "HYYRobotInterface.h"
#include <stdatomic.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include <zmq.h>

#ifndef PROFILE_ROOT
#define PROFILE_ROOT "/home/robot/Work/system/robot_config_new_left"
#endif
#ifndef MK2_BACKEND
#define MK2_BACKEND "physical"
#endif

static atomic_bool stopping = ATOMIC_VAR_INIT(false);
static bool started = false;
static pthread_t worker;

static int correct_profile(void)
{
    char args[8192];
    FILE *stream = fopen("/proc/self/cmdline", "rb");
    if (!stream) return 0;
    size_t size = fread(args, 1, sizeof(args) - 1, stream);
    fclose(stream);
    args[size] = '\0';
    for (size_t pos = 0; pos < size;) {
        size_t length = strnlen(args + pos, size - pos);
        if (!strcmp(args + pos, "--path")) {
            size_t next = pos + length + 1;
            return next < size && !strcmp(args + next, PROFILE_ROOT);
        }
        pos += length + 1;
    }
    return 0;
}

static int finite_seven(const double *values)
{
    for (int i = 0; i < 7; ++i) if (!isfinite(values[i])) return 0;
    return 1;
}

static void *publish_state(void *unused)
{
    (void)unused;
    void *context = zmq_ctx_new();
    if (!context) return NULL;
    void *publisher = zmq_socket(context, ZMQ_PUB);
    if (!publisher) { zmq_ctx_term(context); return NULL; }
    int linger = 0, high_water = 2;
    zmq_setsockopt(publisher, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(publisher, ZMQ_SNDHWM, &high_water, sizeof(high_water));
    if (zmq_bind(publisher, "tcp://*:8000") != 0) {
        fprintf(stderr, "[MK2StateOnly] Cannot bind state port: %s\n", zmq_strerror(zmq_errno()));
        zmq_close(publisher);
        zmq_ctx_term(context);
        return NULL;
    }
    fprintf(stderr, "[MK2StateOnly] State PUB :8000 ready; NO command receiver, NO enable/calibration/gripper calls\n");
    fflush(stderr);
    unsigned long long sequence = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (!atomic_load(&stopping)) {
        if (robot_getNUM() == 1 && robot_getDOF(0) == 7) {
            double q[7] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};
            GetCurrentJoint(q, 0);
            if (finite_seven(q)) {
                double velocity[7] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};
                const char *device = get_deviceName(0, NULL);
                const char *robot = device ? get_name_robot_device(device, 0) : NULL;
                int have_velocity = robot && GetGroupVelocity(robot, velocity) >= 0 && finite_seven(velocity);
                int power = GetRobotPowerState(0);
                char message[2048];
                int n = snprintf(message, sizeof(message),
                    "State {\"Bridge\":{\"mode\":\"feedback_only\",\"commands_enabled\":false,\"backend\":\"" MK2_BACKEND "\",\"sequence\":%llu},"
                    "\"Robot0\":{\"PowerState\":%d,\"Joint\":[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]",
                    ++sequence, power, q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
                if (have_velocity && n > 0 && (size_t)n < sizeof(message)) {
                    int count = snprintf(message + n, sizeof(message) - (size_t)n,
                        ",\"JointVelocity\":[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]",
                        velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5], velocity[6]);
                    if (count < 0 || (size_t)count >= sizeof(message) - (size_t)n) n = -1;
                    else n += count;
                }
                if (n > 0 && (size_t)n + 2 < sizeof(message)) {
                    message[n++] = '}'; message[n++] = '}';
                    zmq_send(publisher, message, (size_t)n, ZMQ_DONTWAIT);
                }
            }
        }
        /* Ordinary worker thread, not the servo cycle. Never block servo control
         * on network I/O; no real-time scheduling or SDK cycle modification. */
        next.tv_nsec += 10000000;
        if (next.tv_nsec >= 1000000000) { next.tv_sec++; next.tv_nsec -= 1000000000; }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > next.tv_sec || (now.tv_sec == next.tv_sec && now.tv_nsec > next.tv_nsec)) next = now;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    zmq_close(publisher);
    zmq_ctx_term(context);
    return NULL;
}

void PluginMain(void)
{
    if (started || !correct_profile()) {
        fprintf(stderr, "[MK2StateOnly] Refusing duplicate start or wrong --path\n");
        return;
    }
    atomic_store(&stopping, false);
    if (pthread_create(&worker, NULL, publish_state, NULL) != 0) {
        fprintf(stderr, "[MK2StateOnly] Cannot create state worker\n");
        return;
    }
    started = true;
}

__attribute__((destructor)) static void stop_state_worker(void)
{
    if (started) {
        atomic_store(&stopping, true);
        pthread_join(worker, NULL);
        started = false;
    }
}