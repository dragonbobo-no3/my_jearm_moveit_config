/* Test definitions compiled against vendor DECLARATIONS only. Never link the SDK.
 * Shared by the export-dynamic loader and direct-runtime test executable. */
#ifndef JE2_TEST_FAKE_SDK_H
#define JE2_TEST_FAKE_SDK_H
#include "HYYRobotInterface.h"
#include <assert.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <string.h>

static struct {
    atomic_uint getters, mutations, starts, joints, stops, ends, jogs, jog_stops;
    atomic_int start_result, stop_result, end_result, jog_result, jog_stop_result;
    atomic_int state, driver_error;
} fake;

#ifdef JE2_FAKE_WRITER_TASKS
/* Real kernel task instances, fake SDK only. SDK thread entry resides in this
 * test executable, never in the unloadable plugin. No fabricated exit ACK. */
#include <pthread.h>
#include <sys/prctl.h>
#include <time.h>
#include <errno.h>
static struct {
    pthread_t thread[2];
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    unsigned count, ready;
    bool end, release;
    atomic_bool stuck, duplicate, missing;
    atomic_uint alive, log_initializations, stop_runs;
    atomic_int jog_stop_state;
} fake_writer = {.mutex = PTHREAD_MUTEX_INITIALIZER, .cond = PTHREAD_COND_INITIALIZER};
static void *fake_writer_main(void *arg)
{
    (void)arg;
    assert(prctl(PR_SET_NAME, "_servo_robot0") == 0);
    pthread_mutex_lock(&fake_writer.mutex);
    ++fake_writer.ready; ++fake_writer.alive;
    pthread_cond_broadcast(&fake_writer.cond);
    while (!fake_writer.release && (!fake_writer.end || fake_writer.stuck))
        pthread_cond_wait(&fake_writer.cond, &fake_writer.mutex);
    pthread_mutex_unlock(&fake_writer.mutex);
    /* End has already returned success; actual thread remains alive here. */
    struct timespec delay = {.tv_nsec = 40000000};
    while (nanosleep(&delay, &delay)) assert(errno == EINTR);
    --fake_writer.alive;
    return NULL;
}
static void fake_writer_spawn(void)
{
    if (fake_writer.missing) return;
    pthread_mutex_lock(&fake_writer.mutex);
    assert(!fake_writer.count);
    fake_writer.end = fake_writer.release = false; fake_writer.ready = 0;
    fake_writer.count = fake_writer.duplicate ? 2 : 1;
    for (unsigned i = 0; i < fake_writer.count; ++i)
        assert(!pthread_create(&fake_writer.thread[i], NULL, fake_writer_main, NULL));
    while (fake_writer.ready != fake_writer.count)
        pthread_cond_wait(&fake_writer.cond, &fake_writer.mutex);
    pthread_mutex_unlock(&fake_writer.mutex);
}
static void fake_writer_end(void)
{
    pthread_mutex_lock(&fake_writer.mutex);
    fake_writer.end = true;
    pthread_cond_broadcast(&fake_writer.cond);
    pthread_mutex_unlock(&fake_writer.mutex);
}
static void fake_writer_cleanup(void)
{
    /* Test fixture ONLY, after plugin destruction. Does not acknowledge exit
     * to production code or permit a jog; a stuck thread must have faulted. */
    pthread_mutex_lock(&fake_writer.mutex);
    fake_writer.release = true;
    pthread_cond_broadcast(&fake_writer.cond);
    pthread_mutex_unlock(&fake_writer.mutex);
    for (unsigned i = 0; i < fake_writer.count; ++i) assert(!pthread_join(fake_writer.thread[i], NULL));
    fake_writer.count = fake_writer.ready = 0;
}
int CreateSaveData1(const char *name, int type, int row, int col)
{
    assert(!strcmp(name, "servotest") && type == 1 && row == 100 && col == 14);
    ++fake_writer.log_initializations;
    return 0;
}
int RobotStopRun(int index)
{
    assert(index == 0); ++fake.mutations; ++fake_writer.stop_runs;
    fake.state = 0; return 0; /* Does NOT end the old writer. */
}
#endif

int GetSystemSimulationState(void) { ++fake.getters; return 1; }
int robot_getNUM(void) { ++fake.getters; return 1; }
int robot_getDOF(int index) { assert(index == 0); ++fake.getters; return 7; }
void GetCurrentJoint(double *joint, int index)
{ assert(index == 0); ++fake.getters; memset(joint, 0, 7 * sizeof(*joint)); }
int GetRobotPowerState(int index) { assert(index == 0); ++fake.getters; return 1; }
int IsRobotDriverError(int index)
{ assert(index == 0); ++fake.getters; return fake.driver_error; }
int IsRobotMoveError(int index) { assert(index == 0); ++fake.getters; return 0; }
int get_robot_move_state(int index)
{ assert(index == 0); ++fake.getters; return fake.state; }
const char *get_deviceName(int index, char *name)
{ (void)name; assert(index == 0); ++fake.getters; return "fake_device"; }
const char *get_name_robot_device(const char *device, int index)
{ assert(device && index == 0); ++fake.getters; return "fake_group"; }
int GetGroupVelocity(const char *name, double *velocity)
{ assert(name); ++fake.getters; memset(velocity, 0, 7 * sizeof(*velocity)); return 0; }
void init_robjoint(robjoint *joint, double *data, int dof)
{
    assert(dof == 7); ++fake.mutations;
    memset(joint, 0, sizeof(*joint)); joint->dof = dof;
    memcpy(joint->angle, data, 7 * sizeof(*data));
}
int ServoStart(double q, double r, int index)
{
    assert(q > 0 && r > 0 && index == 0);
    ++fake.mutations; ++fake.starts; fake.state = 13;
#ifdef JE2_FAKE_WRITER_TASKS
    fake_writer_spawn(); /* Including partial failure after writer creation. */
#endif
    return fake.start_result;
}
int ServoJoint(robjoint *joint, double time, int index)
{
    assert(joint->dof == 7 && time > 0 && index == 0);
    ++fake.mutations; ++fake.joints; return 0;
}
int RobotStop(int index)
{
    assert(index == 0); ++fake.mutations; ++fake.stops;
#ifdef JE2_FAKE_WRITER_TASKS
    /* A real forced-stop observation must NOT be cleared by fault cleanup. */
    if (fake.state != 7) fake.state = 0;
#else
    fake.state = 0;
#endif
    return fake.stop_result;
}
int ServoEnd(int index)
{
    assert(index == 0); ++fake.mutations; ++fake.ends; fake.state = 0;
#ifdef JE2_FAKE_WRITER_TASKS
    fake_writer_end();
#endif
    return fake.end_result;
}
int set_robot_index(int index)
{ assert(index == 0); ++fake.mutations; return 0; }
int set_robot_teach_coordinate(int coordinate)
{ assert(coordinate == 0); ++fake.mutations; return 0; }
void setMoveThread(int threaded) { assert(threaded == 1); ++fake.mutations; }
void set_teach_move_type(int type) { assert(type == 0); ++fake.mutations; }
void set_robot_teach_velocity(double velocity)
{ assert(velocity == .01); ++fake.mutations; }
int robot_teach_joint(int axis, int direction)
{
#ifdef JE2_FAKE_WRITER_TASKS
    assert(fake_writer.alive == 0); /* End success cannot release this check. */
#endif
    assert(axis == 0 && direction == 1); ++fake.mutations; ++fake.jogs;
    fake.state = 1; return fake.jog_result;
}
int robot_teach_stop(void)
{
    ++fake.mutations; ++fake.jog_stops;
#ifdef JE2_FAKE_WRITER_TASKS
    fake.state = fake_writer.jog_stop_state;
#else
    fake.state = 0;
#endif
    return fake.jog_stop_result;
}
#endif