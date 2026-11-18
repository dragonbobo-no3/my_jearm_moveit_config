/* Explicit virtual-only experiment, called ONLY after the real loader's full
 * immutable profile/hash/OS guards and SDK model checks. NO plugin is loaded.
 * This does not change policy.allow_commands or commission an SDK adapter.
 */
#include "../jeserver_v2_task_watch.h"
#include <dirent.h>
static void writer_dump_tasks(void)
{
    /* Post-failure diagnostics ONLY; not fed back into the evidence object. */
    DIR *dir = opendir("/proc/self/task");
    if (!dir) return;
    struct dirent *entry;
    unsigned budget = 0;
    while ((entry = readdir(dir)) && ++budget <= JE2_TASK_LIMIT + 2) {
        char *end;
        long id = strtol(entry->d_name, &end, 10);
        if (*end || id <= 0 || id > INT_MAX) continue;
        char path[80], name[80];
        snprintf(path, sizeof(path), "/proc/self/task/%ld/comm", id);
        FILE *file = fopen(path, "r");
        if (!file) continue;
        if (fgets(name, sizeof(name), file)) {
            name[strcspn(name, "\n")] = '\0';
            printf("JE2_WRITER_TASK_DIAGNOSTIC tid=%ld comm=%s NOT_EXIT_PROOF=1\n", id, name);
        }
        fclose(file);
    }
    closedir(dir);
}
static void writer_pause(void)
{
    struct timespec remaining = {.tv_nsec = 10000000};
    while (nanosleep(&remaining, &remaining) && errno == EINTR) {}
}
static int writer_probe(void *sdk)
{
    int (*select_robot)(int), (*enable)(void), (*power)(int), (*state)(int);
    int (*driver)(int), (*error)(int), (*end)(int), (*stop_run)(int), (*simulation)(void);
    int (*start)(double, double, int), (*send_joint)(robjoint *, double, int);
    int (*create_save)(const char *, int, int, int);
    void (*actual)(double *, int), (*move_thread)(int);
    SYMBOL(sdk, select_robot, "set_robot_index"); SYMBOL(sdk, enable, "robot_teach_enable");
    SYMBOL(sdk, power, "GetRobotPowerState"); SYMBOL(sdk, state, "get_robot_move_state");
    SYMBOL(sdk, driver, "IsRobotDriverError"); SYMBOL(sdk, error, "IsRobotMoveError");
    SYMBOL(sdk, start, "ServoStart"); SYMBOL(sdk, send_joint, "ServoJoint");
    SYMBOL(sdk, end, "ServoEnd"); SYMBOL(sdk, stop_run, "RobotStopRun");
    SYMBOL(sdk, actual, "GetCurrentJoint"); SYMBOL(sdk, move_thread, "setMoveThread");
    SYMBOL(sdk, simulation, "GetSystemSimulationState");
    SYMBOL(sdk, create_save, "CreateSaveData1");
    puts("JE2_WRITER_EXPERIMENT VIRTUAL_SERVO_ACTIVITY=1 PLUGIN_LOADED=0 NO_JOG=1 PRODUCTION_READY=0");
    if (simulation() != 1 || select_robot(0)) return 79;
    /* Fixed SDK servo_execution calls RSaveDataFast1("servotest",1,100,14).
     * Its lazy first call creates _saveDataToFile, which inherits Servo comm.
     * Use the PUBLIC Tool/saveData.h API to initialize that exact queue BEFORE
     * any Servo attempt, from this non-Servo thread. Never rename/ignore a
     * candidate, patch the SDK, fabricate an ACK, or delete an active queue.
     * This explicit setup is part of the experiment's applicability contract. */
    int log_ready = create_save("servotest", 1, 100, 14);
    printf("JE2_WRITER_LOG_INITIALIZE result=%d PUBLIC_SDK_API=1 BEFORE_SERVO=1\n", log_ready);
    if (log_ready) return 79;
    move_thread(1);
    int enabled = enable();
    printf("JE2_WRITER_VIRTUAL_ENABLE result=%d\n", enabled);
    if (enabled) return 79;
    for (unsigned tick = 0; tick < 50 && power(0) != 1; ++tick) writer_pause();
    if (power(0) != 1 || state(0) != 0 || driver(0) || error(0)) return 79;
    robjoint hold = {.dof = MK2_AXES};
    actual(hold.angle, 0);
    for (int i = 0; i < MK2_AXES; ++i) if (!finite_value(hold.angle[i])) return 79;
    for (uint64_t epoch = 1; epoch <= 3; ++epoch) {
        struct je2_task_watch watch;
        je2_task_watch_init(&watch);
        bool attempted = false, ended = false, stopped = false;
        int outcome = 79;
        /* Missing baseline or candidate must fail, NOT become an exit ACK. */
        if (je2_task_watch_begin(&watch, "_servo_robot0", epoch)) goto cleanup_writer;
        attempted = true; /* Includes Start partial failure. */
        int result = start(1, .0001, 0);
        printf("JE2_WRITER_START epoch=%llu result=%d\n", (unsigned long long)epoch, result);
        if (result || je2_task_watch_capture(&watch)) goto cleanup_writer;
        printf("JE2_WRITER_CAPTURE epoch=%llu tid=%d pinned_proc_fd=%d kernel_live=1\n",
            (unsigned long long)epoch, (int)watch.tid, watch.task_fd);
        for (unsigned tick = 1; tick <= 20; ++tick) {
            if (simulation() != 1 || power(0) != 1 || state(0) != 13 || driver(0) || error(0) ||
                je2_task_watch_poll(&watch) != JE2_TASK_RUNNING || send_joint(&hold, tick * .01, 0))
                goto cleanup_writer;
            double q[MK2_AXES]; actual(q, 0);
            for (int i = 0; i < MK2_AXES; ++i)
                if (!finite_value(q[i]) || q[i] - hold.angle[i] > 1e-5 || q[i] - hold.angle[i] < -1e-5)
                    goto cleanup_writer;
            writer_pause();
        }
        if (epoch > 1) {
            stopped = true;
            result = stop_run(0);
            printf("JE2_WRITER_STOP_RUN epoch=%llu result=%d\n", (unsigned long long)epoch, result);
            if (result) goto cleanup_writer;
        }
        ended = true;
        result = end(0);
        /* Log SDK return independently; never turn it into task EXITED. */
        printf("JE2_WRITER_END epoch=%llu result=%d NOT_EXIT_PROOF=1\n", (unsigned long long)epoch, result);
        if (result) goto cleanup_writer;
        enum je2_task_state observed = JE2_TASK_RUNNING;
        for (unsigned tick = 0; tick < 100; ++tick) {
            observed = je2_task_watch_poll(&watch);
            if (observed != JE2_TASK_RUNNING) break;
            writer_pause();
        }
        if (observed != JE2_TASK_EXITED) goto cleanup_writer;
        printf("JE2_WRITER_INSTANCE_EXIT epoch=%llu tid=%d kernel_exit=1\n",
            (unsigned long long)epoch, (int)watch.tid);
        /* Fresh observations AFTER exit, not the sample used for End. */
        for (unsigned tick = 0; tick < 12; ++tick) {
            double q[MK2_AXES]; actual(q, 0);
            if (simulation() != 1 || power(0) != 1 || state(0) != 0 || driver(0) || error(0))
                goto cleanup_writer;
            for (int i = 0; i < MK2_AXES; ++i)
                if (!finite_value(q[i]) || q[i] - hold.angle[i] > 1e-5 || q[i] - hold.angle[i] < -1e-5)
                    goto cleanup_writer;
            writer_pause();
        }
        outcome = 0;
cleanup_writer:
        if (outcome) {
            printf("JE2_WRITER_EVIDENCE_FAILURE epoch=%llu phase=%u reason=%s errno=%d scan_tid=%d scan_name=%s\n",
                (unsigned long long)epoch, watch.phase, watch.reason ? watch.reason : "sdk_or_feedback",
                watch.error_number, (int)watch.scan_tid, watch.scan_name);
            writer_dump_tasks();
        }
        if (outcome && attempted) {
            if (!stopped) { stopped = true; (void)stop_run(0); }
            if (!ended) { ended = true; (void)end(0); }
        }
        je2_task_watch_close(&watch);
        if (outcome) {
            printf("JE2_WRITER_EXPERIMENT_FINISHED result=FAIL epoch=%llu NO_HANDOVER=1\n", (unsigned long long)epoch);
            return outcome;
        }
    }
    puts("JE2_WRITER_EXPERIMENT_FINISHED result=PASS epochs=3 KERNEL_INSTANCE_EXIT=1 NO_HANDOVER=1 PRODUCTION_READY=0");
    return 0;
}