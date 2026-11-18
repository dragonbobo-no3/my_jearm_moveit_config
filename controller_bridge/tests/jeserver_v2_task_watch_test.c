#define _GNU_SOURCE
#include "jeserver_v2_task_watch.h"
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

struct worker {
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int ready, rename, renamed, stop, tid, proc_fd;
};
static void *run(void *arg)
{
    struct worker *w = arg;
    assert(!prctl(PR_SET_NAME, "_servo_robot0"));
    assert(!pthread_mutex_lock(&w->mutex));
    w->tid = (int)syscall(SYS_gettid);
    w->ready = 1;
    pthread_cond_broadcast(&w->cond);
    while (!w->stop) {
        if (w->rename && !w->renamed) {
            assert(!prctl(PR_SET_NAME, "renamed_writer"));
            w->renamed = 1;
            pthread_cond_broadcast(&w->cond);
        }
        pthread_cond_wait(&w->cond, &w->mutex);
    }
    pthread_mutex_unlock(&w->mutex);
    return NULL;
}
static void start(struct worker *w)
{
    memset(w, 0, sizeof(*w));
    assert(!pthread_mutex_init(&w->mutex, NULL));
    assert(!pthread_cond_init(&w->cond, NULL));
    assert(!pthread_create(&w->thread, NULL, run, w));
    pthread_mutex_lock(&w->mutex);
    while (!w->ready) pthread_cond_wait(&w->cond, &w->mutex);
    char path[80];
    snprintf(path, sizeof(path), "/proc/self/task/%d", w->tid);
    w->proc_fd = open(path, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    assert(w->proc_fd >= 0);
    pthread_mutex_unlock(&w->mutex);
}
static void rename_worker(struct worker *w)
{
    pthread_mutex_lock(&w->mutex);
    w->rename = 1;
    pthread_cond_broadcast(&w->cond);
    while (!w->renamed) pthread_cond_wait(&w->cond, &w->mutex);
    pthread_mutex_unlock(&w->mutex);
}
static void stop(struct worker *w)
{
    pthread_mutex_lock(&w->mutex);
    w->stop = 1;
    pthread_cond_broadcast(&w->cond);
    pthread_mutex_unlock(&w->mutex);
    assert(!pthread_join(w->thread, NULL));
    /* Fixture cleanup must finish kernel task teardown too, otherwise a
     * subsequent baseline correctly refuses this still-visible old task. */
    int gone = 0;
    for (unsigned i = 0; i < 1000 && !gone; ++i) {
        int fd = openat(w->proc_fd, "stat", O_RDONLY | O_CLOEXEC);
        if (fd < 0) {
            assert(errno == ENOENT || errno == ESRCH);
            gone = 1;
        } else {
            char byte;
            ssize_t n = read(fd, &byte, 1);
            if (n < 0 && (errno == ENOENT || errno == ESRCH)) gone = 1;
            else assert(n == 1);
            close(fd);
        }
        if (!gone) {
            struct timespec remaining = {.tv_nsec = 1000000};
            while (nanosleep(&remaining, &remaining) && errno == EINTR) {}
        }
    }
    assert(gone);
    close(w->proc_fd);
    assert(!pthread_mutex_destroy(&w->mutex));
    assert(!pthread_cond_destroy(&w->cond));
}
static void arm(struct je2_task_watch *w, uint64_t epoch)
{
    je2_task_watch_init(w);
    assert(!je2_task_watch_begin(w, "_servo_robot0", epoch));
}
static void await_exit(struct je2_task_watch *w)
{
    /* pthread_join/clear_child_tid may precede final proc task teardown.
     * Wait for the independent observation; never manufacture that ACK. */
    for (unsigned i = 0; i < 1000; ++i) {
        enum je2_task_state state = je2_task_watch_poll(w);
        if (state == JE2_TASK_EXITED) return;
        assert(state == JE2_TASK_RUNNING);
        struct timespec remaining = {.tv_nsec = 1000000};
        while (nanosleep(&remaining, &remaining) && errno == EINTR) {}
    }
    assert(!"task instance did not exit within one second");
}
int main(void)
{
    struct je2_task_watch w;
    struct worker a, b;
    je2_task_watch_init(&w);
    assert(je2_task_watch_poll(&w) == JE2_TASK_UNKNOWN);
    assert(je2_task_watch_capture(&w) == -1);
    je2_task_watch_close(&w);

    arm(&w, 1);
    assert(je2_task_watch_capture(&w) == -1); /* Never seen is NOT exited. */
    assert(je2_task_watch_poll(&w) == JE2_TASK_UNKNOWN);
    start(&a);
    assert(je2_task_watch_capture(&w) == -1); /* Unknown cannot recover. */
    stop(&a);
    je2_task_watch_close(&w);

    arm(&w, 2);
    start(&a);
    assert(!je2_task_watch_capture(&w));
    assert(w.epoch == 2 && w.tid > 0);
    for (unsigned i = 0; i < 10; ++i) assert(je2_task_watch_poll(&w) == JE2_TASK_RUNNING);
    rename_worker(&a);
    assert(je2_task_watch_poll(&w) == JE2_TASK_RUNNING); /* Name loss is not exit. */
    stop(&a);
    await_exit(&w);
    start(&b); /* Same name is a DIFFERENT instance, even after the old exit. */
    assert(je2_task_watch_poll(&w) == JE2_TASK_EXITED);
    stop(&b);
    je2_task_watch_close(&w);

    start(&a);
    je2_task_watch_init(&w);
    assert(je2_task_watch_begin(&w, "_servo_robot0", 3) == -1); /* Preexisting writer. */
    rename_worker(&a);
    assert(je2_task_watch_begin(&w, "_servo_robot0", 4) == -1);
    stop(&a);
    je2_task_watch_close(&w);

    arm(&w, 20);
    start(&a);
    assert(!pthread_detach(a.thread));
    assert(!je2_task_watch_capture(&w));
    pthread_mutex_lock(&a.mutex);
    a.stop = 1;
    pthread_cond_broadcast(&a.cond);
    pthread_mutex_unlock(&a.mutex);
    await_exit(&w); /* SDK-shaped detached thread: no join API involved. */
    close(a.proc_fd);
    assert(!pthread_mutex_destroy(&a.mutex));
    assert(!pthread_cond_destroy(&a.cond));
    je2_task_watch_close(&w);

    start(&a);
    rename_worker(&a);
    arm(&w, 21);
    assert(!pthread_setname_np(a.thread, "_servo_robot0"));
    assert(je2_task_watch_capture(&w) == -1); /* Old task newly adopts name. */
    stop(&a);
    je2_task_watch_close(&w);

    arm(&w, 5);
    start(&a); start(&b);
    assert(je2_task_watch_capture(&w) == -1); /* Ambiguous writers. */
    assert(je2_task_watch_poll(&w) == JE2_TASK_UNKNOWN);
    stop(&a); stop(&b);
    je2_task_watch_close(&w);

    arm(&w, 6);
    start(&a); stop(&a);
    assert(je2_task_watch_capture(&w) == -1); /* Exited before observation. */
    je2_task_watch_close(&w);

    arm(&w, 7);
    start(&a);
    assert(!je2_task_watch_capture(&w));
    assert(!close(w.task_fd)); /* EBADF must not be interpreted as exit. */
    assert(je2_task_watch_poll(&w) == JE2_TASK_UNKNOWN);
    w.task_fd = -1;
    stop(&a);
    je2_task_watch_close(&w);

    je2_task_watch_init(&w);
    assert(je2_task_watch_begin(&w, "_servo_robot0", 0) == -1);
    je2_task_watch_close(&w);
    je2_task_watch_init(&w);
    assert(je2_task_watch_begin(&w, "1234567890123456", 8) == -1);
    je2_task_watch_close(&w);
    je2_task_watch_init(&w);
    assert(je2_task_watch_begin(&w, "", 9) == -1);
    je2_task_watch_close(&w);

    /* Repeated fresh epochs exercise FD cleanup and real task churn. This
     * does not claim that the kernel happened to recycle a numeric TID. */
    for (unsigned i = 0; i < 64; ++i) {
        arm(&w, 100 + i);
        start(&a);
        assert(!je2_task_watch_capture(&w));
        stop(&a);
        await_exit(&w);
        je2_task_watch_close(&w);
    }
    puts("JE2_TASK_WATCH_TEST PASS real_kernel_tasks=1 SDK_LOADED=0 epochs=64");
    return 0;
}