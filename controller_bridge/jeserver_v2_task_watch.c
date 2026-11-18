#define _GNU_SOURCE
#include "jeserver_v2_task_watch.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <sys/vfs.h>
#include <unistd.h>

static int unknown(struct je2_task_watch *w)
{
    if (!w->reason) w->reason = "invalid_phase_or_input";
    w->phase = 4;
    return -1;
}
void je2_task_watch_init(struct je2_task_watch *w)
{
    memset(w, 0, sizeof(*w));
    w->task_fd = -1;
}
/* No caller-supplied proc root: fixture directories cannot become evidence. */
static int proc_tasks(void)
{
    int fd = open("/proc/self/task", O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    struct statfs fs;
    if (fd >= 0 && (!fstatfs(fd, &fs) && fs.f_type == 0x9fa0)) return fd;
    if (fd >= 0) close(fd);
    return -1;
}
static int task_name(int fd, char name[17])
{
    int file = openat(fd, "comm", O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (file < 0) return -1;
    ssize_t n;
    do { n = read(file, name, 17); } while (n < 0 && errno == EINTR);
    close(file);
    if (n < 2 || n > 16 || name[n - 1] != '\n') return -1;
    name[n - 1] = '\0';
    return 0;
}
static int baseline_has(const struct je2_task_watch *w, pid_t tid)
{
    for (unsigned i = 0; i < w->baseline_count; ++i)
        if (w->baseline[i] == tid) return 1;
    return 0;
}
static int scan(struct je2_task_watch *w, int capture)
{
    int root = proc_tasks();
    DIR *dir = root < 0 ? NULL : fdopendir(root);
    if (!dir) {
        w->reason = "proc_root"; w->error_number = errno;
        if (root >= 0) close(root);
        return unknown(w);
    }
    unsigned count = 0, matches = 0;
    int failed = 0;
    for (;;) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno) { w->reason = "enumeration"; w->error_number = errno; failed = 1; }
            break;
        }
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        char *end;
        long id = strtol(entry->d_name, &end, 10);
        if (*end || id <= 0 || id > INT_MAX || ++count > JE2_TASK_LIMIT) {
            w->reason = "task_id_or_count"; failed = 1; break;
        }
        w->scan_tid = (pid_t)id;
        w->scan_name[0] = '\0';
        int fd = openat(root, entry->d_name, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        char name[17];
        /* Even a concurrent unrelated exit makes this scan inconclusive. */
        if (fd < 0) {
            w->reason = "task_open"; w->error_number = errno; failed = 1; break;
        }
        if (task_name(fd, name)) {
            w->reason = "task_comm"; w->error_number = errno;
            close(fd); failed = 1; break;
        }
        memcpy(w->scan_name, name, strlen(name) + 1);
        if (!capture) w->baseline[w->baseline_count++] = (pid_t)id;
        if (!strcmp(name, w->name)) {
            ++matches;
            if (!capture || baseline_has(w, (pid_t)id) || matches > 1) {
                w->reason = !capture ? "preexisting_name" :
                    baseline_has(w, (pid_t)id) ? "preexisting_tid" : "duplicate_candidate";
                close(fd); failed = 1; break;
            }
            w->task_fd = fd;
            w->tid = (pid_t)id;
        } else close(fd);
    }
    closedir(dir);
    if (capture && !matches && !failed) w->reason = "no_candidate";
    if (failed || (capture && matches != 1)) return unknown(w);
    w->phase = capture ? 2 : 1;
    return 0;
}
int je2_task_watch_begin(struct je2_task_watch *w, const char *name, uint64_t epoch)
{
    if (w->phase || !name || !*name || strlen(name) > 15 || strchr(name, '\n') || !epoch)
        return unknown(w);
    memcpy(w->name, name, strlen(name) + 1);
    w->epoch = epoch;
    return scan(w, 0);
}
int je2_task_watch_capture(struct je2_task_watch *w)
{
    if (w->phase != 1) return unknown(w);
    if (scan(w, 1)) return -1;
    /* A candidate must actually be observed alive through its pinned handle. */
    if (je2_task_watch_poll(w) != JE2_TASK_RUNNING) {
        if (!w->reason) w->reason = "not_observed_live";
        return unknown(w);
    }
    return 0;
}
enum je2_task_state je2_task_watch_poll(struct je2_task_watch *w)
{
    if (w->phase == 3) return JE2_TASK_EXITED;
    if (w->phase != 2 || w->task_fd < 0) return JE2_TASK_UNKNOWN;
    /* Do not reopen /proc/self/task/<number>: a reused TID is another task.
     * Kernel proc task dentries retain the original struct pid association.
     * On exit, opening stat relative to that pinned directory fails. */
    int fd = openat(w->task_fd, "stat", O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
        if (errno == ENOENT || errno == ESRCH) { w->phase = 3; return JE2_TASK_EXITED; }
        w->reason = "pinned_stat_open"; w->error_number = errno;
        unknown(w);
        return JE2_TASK_UNKNOWN;
    }
    char byte;
    ssize_t n;
    do { n = read(fd, &byte, 1); } while (n < 0 && errno == EINTR);
    int error = errno;
    close(fd);
    if (n == 1) return JE2_TASK_RUNNING;
    if (n < 0 && (error == ENOENT || error == ESRCH)) { w->phase = 3; return JE2_TASK_EXITED; }
    w->reason = "pinned_stat_read"; w->error_number = error;
    unknown(w);
    return JE2_TASK_UNKNOWN;
}
void je2_task_watch_close(struct je2_task_watch *w)
{
    if (w->task_fd >= 0) close(w->task_fd);
    w->task_fd = -1;
    w->phase = 4; /* Reuse requires explicit init, never implicit re-arming. */
}