/* Opt-in PRIVATE SIMULATION experiment only. Included after SDK helpers.
 * No physical build, environment force switch, thread rename or SDK patch.
 * Exclusive Start interval and public log preinitialization are required.
 */
#include "jeserver_v2_task_watch.h"
struct je2_writer_adapter {
    struct je2_task_watch task;
    bool ready, attempted;
    double exit_time;
};
static int je2_writer_prepare(struct je2_writer_adapter *w)
{
    memset(w, 0, sizeof(*w));
    je2_task_watch_init(&w->task);
    if (!policy.allow_commands) return 0; /* Read-only lifecycle stays inert. */
    if (!backend_verified() || CreateSaveData1("servotest", 1, 100, 14)) return -1;
    w->ready = true;
    return 0;
}
static int je2_writer_start(void *context, double q, double r)
{
    struct je2_writer_adapter *w = context;
    /* No PC reclaim transport yet: one Start attempt per plugin lifetime.
     * Mark before every possible partial effect; never silently re-arm. */
    if (!w->ready || w->attempted) return -1;
    w->attempted = true;
    if (je2_task_watch_begin(&w->task, "_servo_robot0", 1)) return -1;
    int result = sdk_start(NULL, q, r);
    if (result) return result;
    if (je2_task_watch_capture(&w->task)) {
        fprintf(stderr, "[JE2 writer] capture refused: %s errno=%d\n",
            w->task.reason ? w->task.reason : "unknown", w->task.error_number);
        return -1;
    }
    fprintf(stderr, "[JE2 writer] captured live tid=%d epoch=1\n", (int)w->task.tid);
    return 0;
}
static int je2_writer_stop_run(void *context)
{
    /* Vendor RobotStopRun internally invokes recover_robot_move_state after
     * its stop wait. This PRIVATE experiment uses it only for the engine's
     * healthy release path. NEVER reuse it to clear a native jog STOP/state7.
     * No production claim: sampling cannot make that vendor operation atomic
     * with an external safety event. */
    (void)context;
    return RobotStopRun(0);
}
static enum je2_writer je2_writer_observe(void *context)
{
    struct je2_writer_adapter *w = context;
    if (!w->ready || !w->attempted) return JE2_WRITER_UNKNOWN;
    switch (je2_task_watch_poll(&w->task)) {
    case JE2_TASK_RUNNING: return JE2_WRITER_RUNNING;
    case JE2_TASK_EXITED:
        if (!w->exit_time) {
            w->exit_time = monotonic_time();
            fprintf(stderr, "[JE2 writer] kernel instance exited tid=%d epoch=1 time=%.9f\n",
                (int)w->task.tid, w->exit_time);
        }
        return JE2_WRITER_EXITED;
    default: return JE2_WRITER_UNKNOWN;
    }
}