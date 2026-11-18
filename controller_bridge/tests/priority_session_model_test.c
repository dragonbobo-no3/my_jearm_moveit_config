/* Standalone offline policy tests, not a ROS/CMake target. */
#include "priority_session_model.h"
#include <assert.h>
#include <stdio.h>

static const struct priority_evidence complete = {true, true, true, true};
static unsigned tests;
#define TEST_DONE() do { ++tests; } while (0)
static struct priority_session pc_session(void)
{
    struct priority_session s;
    priority_init(&s, 100, 300);
    assert(!priority_admit_pc(&s, s.generation));
    assert(priority_ready(&s, 1, complete));
    assert(priority_claim_pc(&s, 10, s.generation, complete));
    assert(s.generation == 2 && s.owner == PRIORITY_PC);
    return s;
}
static struct priority_session yielding(void)
{
    struct priority_session s = pc_session();
    assert(priority_press(&s, 20, 1, 20, true) == PRIORITY_WAIT);
    assert(s.owner == PRIORITY_YIELDING && s.generation == 3);
    return s;
}
static void old_pc_commands(void)
{
    enum { START, JOINT, RELEASE, STOP, END, ACTIONS };
    unsigned calls[ACTIONS] = {0};
    struct priority_session s = pc_session();
    const uint64_t old = s.generation;
    for (unsigned action = 0; action < ACTIONS; ++action)
        if (priority_admit_pc(&s, old)) ++calls[action];
    assert(priority_press(&s, 20, 1, 20, true) == PRIORITY_WAIT);
    for (unsigned action = 0; action < ACTIONS; ++action) {
        if (priority_admit_pc(&s, old)) ++calls[action];
        assert(calls[action] == 1);
    }
    assert(priority_poll(&s, 30, complete) == PRIORITY_FORWARD);
    for (unsigned action = 0; action < ACTIONS; ++action) {
        if (priority_admit_pc(&s, old)) ++calls[action];
        assert(calls[action] == 1);
    }
    TEST_DONE();
}
static void missing_evidence(void)
{
    for (int missing = 0; missing < 3; ++missing) {
        struct priority_session s = yielding();
        struct priority_evidence e = complete;
        if (missing == 0) e.pc_quiesced = false;
        if (missing == 1) e.writer_exited = false;
        if (missing == 2) e.stationary = false;
        assert(priority_poll(&s, 30, e) == PRIORITY_WAIT);
        assert(s.owner == PRIORITY_YIELDING && !s.pendant_may_move);
        assert(!priority_admit_pc(&s, 2));
        assert(priority_poll(&s, 40, complete) == PRIORITY_FORWARD);
        TEST_DONE();
    }
}
static void release_during_yield(void)
{
    struct priority_session s = yielding();
    assert(!priority_release_pendant(&s, 25));
    assert(priority_poll(&s, 30, complete) == PRIORITY_CANCELLED);
    assert(!s.pendant_may_move && s.owner == PRIORITY_PENDANT);
    assert(priority_poll(&s, 40, complete) == PRIORITY_REJECT);
    assert(!priority_admit_pc(&s, 2));
    TEST_DONE();
}
static void expired_press(void)
{
    struct priority_session s = yielding();
    assert(priority_poll(&s, 121, complete) == PRIORITY_CANCELLED);
    assert(!s.pending && !s.pendant_may_move);
    assert(priority_press(&s, 122, 1, 122, true) == PRIORITY_REJECT);
    TEST_DONE();
}
static void timeout_latches(void)
{
    struct priority_session s = yielding();
    assert(priority_poll(&s, 320, complete) == PRIORITY_FAILED);
    assert(s.owner == PRIORITY_FAULT);
    assert(!priority_ready(&s, 321, complete));
    assert(!priority_claim_pc(&s, 322, s.generation, complete));
    assert(priority_press(&s, 323, 2, 323, true) == PRIORITY_REJECT);
    TEST_DONE();
}
static void unhealthy_latches(void)
{
    struct priority_session s = yielding();
    struct priority_evidence e = complete;
    e.healthy_fresh = false;
    assert(priority_poll(&s, 30, e) == PRIORITY_FAILED);
    assert(priority_poll(&s, 31, complete) == PRIORITY_FAILED);
    assert(!priority_admit_pc(&s, 2) && !priority_admit_pc(&s, 3));
    TEST_DONE();
}
static void once_only(void)
{
    struct priority_session s = yielding();
    assert(priority_poll(&s, 30, complete) == PRIORITY_FORWARD);
    assert(priority_poll(&s, 31, complete) == PRIORITY_REJECT);
    assert(priority_press(&s, 32, 2, 32, true) == PRIORITY_REJECT);
    assert(s.pendant_may_move);
    TEST_DONE();
}
static void new_pc_handshake(void)
{
    struct priority_session s = yielding();
    assert(priority_poll(&s, 30, complete) == PRIORITY_FORWARD);
    assert(!priority_claim_pc(&s, 31, 3, complete));
    assert(priority_release_pendant(&s, 32));
    assert(!priority_claim_pc(&s, 33, 3, complete)); /* Release is not idle evidence. */
    assert(priority_pendant_idle(&s, 34, complete));
    assert(!priority_claim_pc(&s, 35, 2, complete)); /* Pre-yield handshake replay. */
    assert(priority_claim_pc(&s, 36, 3, complete));
    assert(s.generation == 4 && priority_admit_pc(&s, 4));
    assert(!priority_admit_pc(&s, 2) && !priority_admit_pc(&s, 3));
    TEST_DONE();
}
static void malformed_stale_or_future(void)
{
    struct priority_session s = pc_session();
    assert(priority_press(&s, 20, 1, 20, false) == PRIORITY_REJECT);
    assert(priority_press(&s, 21, 0, 21, true) == PRIORITY_REJECT);
    assert(priority_press(&s, 22, 1, 23, true) == PRIORITY_REJECT);
    assert(priority_press(&s, 101, 1, 0, true) == PRIORITY_REJECT);
    assert(s.generation == 2 && priority_admit_pc(&s, 2));
    TEST_DONE();
}
static void repeats_during_barrier(void)
{
    struct priority_session s = yielding();
    assert(priority_press(&s, 21, 1, 21, true) == PRIORITY_REJECT);
    assert(priority_press(&s, 22, 2, 22, true) == PRIORITY_REJECT);
    assert(s.request == 1 && s.request_time == 20 && s.generation == 3);
    TEST_DONE();
}
static void clock_rollback(void)
{
    struct priority_session s = yielding();
    assert(priority_poll(&s, 19, complete) == PRIORITY_FAILED);
    assert(s.owner == PRIORITY_FAULT && !priority_admit_pc(&s, 2));
    TEST_DONE();
}
static void generation_overflow(void)
{
    struct priority_session s = pc_session();
    s.generation = UINT64_MAX;
    assert(priority_press(&s, 20, 1, 20, true) == PRIORITY_FAILED);
    assert(s.owner == PRIORITY_FAULT && !priority_admit_pc(&s, UINT64_MAX));
    TEST_DONE();
}
static void startup_and_no_pc(void)
{
    struct priority_session s;
    priority_init(&s, 0, 300);
    assert(s.owner == PRIORITY_FAULT);
    priority_init(&s, 100, 99);
    assert(s.owner == PRIORITY_FAULT);
    priority_init(&s, 100, 300);
    assert(priority_press(&s, 1, 1, 1, true) == PRIORITY_REJECT);
    struct priority_evidence incomplete = complete;
    incomplete.writer_exited = false;
    assert(!priority_ready(&s, 2, incomplete));
    assert(priority_ready(&s, 3, complete));
    assert(priority_press(&s, 4, 1, 4, true) == PRIORITY_WAIT);
    assert(priority_poll(&s, 5, complete) == PRIORITY_FORWARD);
    assert(!priority_admit_pc(&s, s.generation));
    TEST_DONE();
}
int main(void)
{
    old_pc_commands(); missing_evidence(); release_during_yield(); expired_press();
    timeout_latches(); unhealthy_latches(); once_only(); new_pc_handshake();
    malformed_stale_or_future(); repeats_during_barrier(); clock_rollback();
    generation_overflow(); startup_and_no_pc();
    printf("PASS: %u offline priority-session scenarios; NO SDK/transport/motion\n", tests);
    return 0;
}