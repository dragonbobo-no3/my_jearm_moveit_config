/* Linux task-instance evidence, NOT an SDK ownership acknowledgement.
 * One caller, one Start attempt per object; initialize before use, close last.
 * Requires exclusive creation of the verified SDK writer during the interval.
 * A name identifies a candidate only; a pinned proc task directory identifies
 * the captured instance. Missing/unobserved/ambiguous candidates never EXIT.
 */
#ifndef JESERVER_V2_TASK_WATCH_H
#define JESERVER_V2_TASK_WATCH_H
#include <stdint.h>
#include <sys/types.h>

#define JE2_TASK_LIMIT 1024
enum je2_task_state { JE2_TASK_UNKNOWN, JE2_TASK_RUNNING, JE2_TASK_EXITED };
struct je2_task_watch {
    int task_fd;
    unsigned phase; /* 0 unused, 1 baseline, 2 captured, 3 exited, 4 unknown */
    uint64_t epoch;
    pid_t tid;
    pid_t baseline[JE2_TASK_LIMIT];
    unsigned baseline_count;
    char name[16];
    const char *reason; /* Diagnostic only; never an exit acknowledgement. */
    int error_number;
    pid_t scan_tid;
    char scan_name[17];
};
void je2_task_watch_init(struct je2_task_watch *watch);
int je2_task_watch_begin(struct je2_task_watch *watch, const char *name, uint64_t epoch);
int je2_task_watch_capture(struct je2_task_watch *watch);
enum je2_task_state je2_task_watch_poll(struct je2_task_watch *watch);
void je2_task_watch_close(struct je2_task_watch *watch);
#endif