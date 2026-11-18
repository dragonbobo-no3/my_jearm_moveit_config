/* Helper integration ONLY, deliberately NOT full plugin initialization.
 * No PluginMain call, namespace spoofing or production startup guard override.
 * Real SDK adapters call the fake definitions; real TCP transport delivers keys. */
#include "../jeserver_v2_plugin.c"
#include "jeserver_v2_fake_sdk.h"
#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>

static struct jeserver_v2 engine;
static void initialize(void)
{
    memset(&fake, 0, sizeof(fake));
    policy = (struct mk2_policy){.allow_commands = true, .watchdog = .2,
        .first_timeout = 1, .min_dt = .005, .max_dt = .05, .clock_skew = .1,
        .first_hold_tolerance = .001, .following_tolerance = .02, .servo_q = 1, .servo_r = .0001};
    for (int i = 0; i < MK2_AXES; ++i) {
        policy.lower[i] = -1; policy.upper[i] = 1;
        policy.velocity[i] = .2; policy.acceleration[i] = .4;
    }
    const struct mk2_io io = {.start = sdk_start, .joint = sdk_joint, .stop = sdk_stop, .end = sdk_end};
    const struct je2_io teach = {.writer = je2_sdk_writer, .jog = je2_sdk_jog, .jog_stop = je2_sdk_jog_stop};
    assert(je2_init(&engine, &policy, &io, &teach) == 0);
    assert(!engine.raw_io.stop_run && je2_sdk_writer(NULL) == JE2_WRITER_UNKNOWN);
}
static int pc(const char *text)
{
    struct je2_feedback f = je2_read_feedback();
    return je2_pc_frame(&engine, 0, text, strlen(text), monotonic_time(), &f);
}
static void tick(void)
{
    struct je2_feedback f = je2_read_feedback();
    je2_tick(&engine, monotonic_time(), &f);
}
static void wait_until(double deadline)
{
    /* Bounded real time, no artificial replacement of the production clock. */
    while (monotonic_time() < deadline) {
        struct timespec delay = {.tv_nsec = 1000000};
        assert(nanosleep(&delay, NULL) == 0 || errno == EINTR);
    }
}
static int connect_button(struct je2_transport *t)
{
    assert(je2_transport_open(t, 0) == 0);
    struct sockaddr_in bound;
    socklen_t size = sizeof(bound);
    assert(getsockname(t->listener_fd, (struct sockaddr *)&bound, &size) == 0);
    assert(bound.sin_addr.s_addr == htonl(INADDR_LOOPBACK));
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    assert(fd >= 0 && connect(fd, (struct sockaddr *)&bound, size) == 0);
    return fd;
}
static void send_button(int fd, const char *text)
{ assert(send(fd, text, strlen(text), MSG_NOSIGNAL) == (ssize_t)strlen(text)); }
static struct je2_event next_event(struct je2_transport *t)
{
    double deadline = monotonic_time() + 1;
    struct je2_event event;
    while (monotonic_time() < deadline) {
        int result = je2_transport_next(t, &event);
        assert(result >= 0);
        if (result) return event;
        struct pollfd fds[2] = {{.fd = t->listener_fd, .events = POLLIN},
                               {.fd = t->client_fd, .events = POLLIN}};
        assert(poll(fds, 2, 2) >= 0 || errno == EINTR);
    }
    assert(!"timed out waiting for real loopback transport");
    return (struct je2_event){0};
}
static void receive_apply(struct je2_transport *t, enum je2_event_type expected)
{
    struct je2_event event = next_event(t);
    assert(event.type == expected);
    je2_apply_button(&engine, &event);
}
static void start_pc(void)
{
    assert(pc("Switch {\"Switch\":true}") == 0);
    assert(pc("Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0,0,0,0,0,0,0]}}") == 0);
    assert(engine.owner == JE2_PC && fake.starts == 1 && fake.joints == 1);
}
static void pc_refuses_unproven_handover(void)
{
    initialize(); start_pc();
    struct je2_transport t; int fd = connect_button(&t);
    unsigned mutations = fake.mutations;
    send_button(fd, "tv49!"); receive_apply(&t, JE2_EVENT_JOG);
    assert(engine.owner == JE2_PC && !engine.pending && engine.legacy_open);
    assert(fake.mutations == mutations && !fake.jogs && !fake.ends && !fake.stops);
    assert(close(fd) == 0); receive_apply(&t, JE2_EVENT_DISCONNECT);
    assert(fake.mutations == mutations);
    je2_transport_close(&t);
    je2_runtime_shutdown(&engine); je2_runtime_shutdown(&engine);
    assert(fake.stops == 1 && fake.ends == 1 && engine.core.fault == MK2_UNLOAD);
    puts("PASS: real TCP jog denied while PC owns servo and writer proof unavailable");
}
static void assert_publication(void)
{
    void *context = zmq_ctx_new(); assert(context);
    void *sender = zmq_socket(context, ZMQ_PAIR), *receiver = zmq_socket(context, ZMQ_PAIR);
    assert(sender && receiver);
    int timeout = 1000, linger = 0;
    assert(zmq_setsockopt(receiver, ZMQ_RCVTIMEO, &timeout, sizeof(timeout)) == 0);
    assert(zmq_setsockopt(sender, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(receiver, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_bind(receiver, "inproc://je2-test-state") == 0);
    assert(zmq_connect(sender, "inproc://je2-test-state") == 0);
    struct je2_feedback f = je2_read_feedback();
    je2_publish(sender, &engine, &f.sample, 123, 12345, NULL);
    char text[MK2_FRAME_MAX + 1];
    for (int i = 0; i < 2; ++i) {
        int n = zmq_recv(receiver, text, sizeof(text) - 1, 0);
        assert(n > 6 && n < (int)sizeof(text)); text[n] = '\0';
        struct json_object *root = json_tokener_parse(text + 6), *field, *value;
        assert(root);
        if (i == 0) {
            assert(!strncmp(text, "State ", 6));
            assert(json_object_object_get_ex(root, "Bridge", &field));
            assert(json_object_object_get_ex(field, "commands_enabled", &value));
            assert(!json_object_get_boolean(value));
            assert(json_object_object_get_ex(field, "simulation_isolated", &value));
            assert(!json_object_get_boolean(value));
        } else {
            assert(!strncmp(text, "Owner ", 6));
            const char *flags[] = {"automatic_priority_ready", "writer_exit_verified", "pc_reclaim_supported"};
            for (size_t j = 0; j < sizeof(flags) / sizeof(flags[0]); ++j) {
                assert(json_object_object_get_ex(root, flags[j], &value));
                assert(!json_object_get_boolean(value));
            }
            assert(json_object_object_get_ex(root, "owner", &value));
            assert(json_object_get_int(value) == (int)engine.owner);
        }
        json_object_put(root);
    }
    assert(zmq_close(sender) == 0 && zmq_close(receiver) == 0 && zmq_ctx_term(context) == 0);
}
static void idle_jog_and_fencing(void)
{
    initialize();
    /* Prior idle ticks cannot qualify a NEW intent. */
    tick(); wait_until(monotonic_time() + .11); tick();
    struct je2_transport t; int fd = connect_button(&t);
    send_button(fd, "tv49!"); receive_apply(&t, JE2_EVENT_JOG);
    assert(engine.pending && !engine.legacy_open && !fake.mutations);
    tick(); assert(!fake.jogs);
    const double stable = engine.stable_at;
    assert(stable > engine.request_at);
    wait_until(stable + .05); tick(); assert(!fake.jogs);
    wait_until(stable + .11); tick();
    assert(fake.jogs == 1 && engine.owner == JE2_PENDANT && engine.jog_live);
    unsigned mutations = fake.mutations;
    const char *late[] = {"Switch {\"Switch\":true}", "Switch {\"Switch\":false}",
        "Release {\"Release\":true}", "Joint garbage"};
    for (size_t i = 0; i < sizeof(late) / sizeof(late[0]); ++i) assert(pc(late[i]) == -1);
    assert(fake.mutations == mutations && engine.dropped_pc == 4);
    assert_publication();
    send_button(fd, "tv33!"); receive_apply(&t, JE2_EVENT_STOP);
    assert(fake.jog_stops == 1 && !fake.stops && !fake.ends);
    je2_runtime_shutdown(&engine); je2_runtime_shutdown(&engine);
    assert(fake.jog_stops == 1 && fake.stops == 1 && !fake.ends);
    assert(close(fd) == 0); je2_transport_close(&t);
    puts("PASS: idle jog needs 100ms fresh post-intent stability; fenced PC and real State/Owner publication");
}
static void cancellation_before_tick(enum je2_event_type cancellation)
{
    initialize(); struct je2_transport t; int fd = connect_button(&t);
    send_button(fd, "tv49!"); receive_apply(&t, JE2_EVENT_JOG);
    assert(engine.pending);
    if (cancellation == JE2_EVENT_STOP) send_button(fd, "tv33!");
    else if (cancellation == JE2_EVENT_REJECT) send_button(fd, "invalid!");
    else { assert(close(fd) == 0); fd = -1; }
    receive_apply(&t, cancellation);
    assert(!engine.pending && !fake.mutations);
    if (cancellation == JE2_EVENT_REJECT) receive_apply(&t, JE2_EVENT_DISCONNECT);
    tick(); wait_until(monotonic_time() + .11); tick();
    assert(!fake.jogs && !fake.mutations && !engine.pending);
    je2_runtime_shutdown(&engine); je2_runtime_shutdown(&engine);
    assert(!fake.mutations);
    if (fd >= 0) assert(close(fd) == 0);
    je2_transport_close(&t);
}
static void cleanup_first_error(void)
{
    initialize(); fake.start_result = -41; fake.stop_result = -42; fake.end_result = -43;
    assert(pc("Switch {\"Switch\":true}") == 0);
    assert(pc("Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0,0,0,0,0,0,0]}}") == -1);
    assert(engine.owner == JE2_FAULT && engine.core.sdk_error == -41);
    assert(engine.cleanup_error == -42 && fake.stops == 1 && fake.ends == 1);
    enum mk2_fault first = engine.core.fault;
    for (int i = 0; i < 8; ++i) je2_runtime_shutdown(&engine);
    assert(engine.core.sdk_error == -41 && engine.core.fault == first && engine.cleanup_error == -42);
    assert(fake.starts == 1 && fake.stops == 1 && fake.ends == 1 && !fake.jogs);
    initialize(); start_pc();
    fake.driver_error = 9; tick();
    assert(engine.core.fault == MK2_STATE_ERROR && fake.stops == 1 && fake.ends == 1);
    je2_runtime_shutdown(&engine);
    assert(engine.core.fault == MK2_STATE_ERROR && fake.stops == 1 && fake.ends == 1);
    puts("PASS: partial-start cleanup exactly once, first SDK/fault/cleanup error retained");
}
static void coalesced_cancel_and_eof(void)
{
    initialize(); struct je2_transport t; int fd = connect_button(&t);
    /* Stop in the same receive batch supersedes older jogs before application. */
    send_button(fd, "tv49!tv49!tv33!");
    receive_apply(&t, JE2_EVENT_STOP);
    tick(); assert(!engine.pending && !fake.mutations && engine.legacy_open);
    assert(close(fd) == 0); receive_apply(&t, JE2_EVENT_DISCONNECT);
    je2_transport_close(&t);

    initialize(); fd = connect_button(&t);
    /* A complete jog buffered at EOF must not become a new live intent. */
    send_button(fd, "tv49!"); assert(shutdown(fd, SHUT_WR) == 0);
    receive_apply(&t, JE2_EVENT_DISCONNECT);
    tick(); assert(!engine.pending && !fake.mutations && engine.legacy_open);
    assert(close(fd) == 0); je2_transport_close(&t);
    puts("PASS: coalesced TCP STOP and jog-at-EOF never initiate pending motion");
}
static void start_idle_motion(void)
{
    initialize();
    const struct je2_event event = {.type = JE2_EVENT_JOG, .axis = 0, .direction = 1};
    je2_apply_button(&engine, &event);
    tick(); wait_until(monotonic_time() + .11); tick();
    assert(engine.jog_live && fake.jogs == 1);
}
static void bounded_jog_runtime(void)
{
    start_idle_motion();
    struct je2_feedback f = je2_read_feedback();
    double now = monotonic_time();
    struct je2_jog_bound bound = {0};
    je2_bound_jog(&engine, &bound, now, &f);
    je2_bound_jog(&engine, &bound, now + .49, &f);
    assert(!fake.jog_stops && engine.owner == JE2_PENDANT);
    je2_bound_jog(&engine, &bound, now + .51, &f);
    assert(engine.owner == JE2_FAULT && engine.core.fault == MK2_WATCHDOG);
    assert(fake.jog_stops == 1 && fake.stops == 1 && !fake.ends);
    je2_runtime_shutdown(&engine);
    assert(fake.jog_stops == 1 && fake.stops == 1);

    start_idle_motion(); f = je2_read_feedback(); now = monotonic_time();
    bound = (struct je2_jog_bound){0};
    je2_bound_jog(&engine, &bound, now, &f);
    f.sample.joint[6] += .021; /* Even unexpected cross-axis motion stops. */
    je2_bound_jog(&engine, &bound, now + .01, &f);
    assert(engine.owner == JE2_FAULT && engine.core.fault == MK2_LIMIT);
    assert(fake.jog_stops == 1 && fake.stops == 1);

    start_idle_motion();
    fake.state = 0; tick(); wait_until(monotonic_time() + .11); tick();
    assert(engine.jog_live && !engine.jog_stop_issued); /* no spontaneous retirement */
    je2_runtime_shutdown(&engine);
    assert(fake.jog_stops == 1 && fake.stops == 1);
    puts("PASS: jog 500ms/0.02rad bounds; idle alone cannot discard continuous jog responsibility");
}
static void event_budget_cancels_pending(void)
{
    initialize(); struct je2_transport t; int fd = connect_button(&t);
    /* 20 complete presses fit in one transport buffer and exceed the runtime
     * event budget. No tick may execute its first accepted pending jog. */
    send_button(fd, "tv49!tv49!tv49!tv49!tv49!tv49!tv49!tv49!tv49!tv49!"
                    "tv49!tv49!tv49!tv49!tv49!tv49!tv49!tv49!tv49!tv49!");
    assert(je2_drain_buttons(&engine, &t) == 0);
    assert(!engine.pending && !fake.mutations);
    assert(je2_drain_buttons(&engine, &t) == 0);
    tick(); wait_until(monotonic_time() + .11); tick();
    assert(!fake.jogs && !engine.pending);
    assert(close(fd) == 0); je2_transport_close(&t);
    je2_runtime_shutdown(&engine);
    puts("PASS: saturated button event budget cancels pending motion before barrier/tick");
}
#if JE2_SIM_SESSION_EXPERIMENT
#include "jeserver_v2_session_test.inc"
#endif
int main(void)
{
    /* Host refusal itself must not even query fake SDK getters. */
    assert(!getenv("MK2_HOST_MNT"));
    assert(!je2_simulation_guard() && !fake.getters && !fake.mutations);
    pc_refuses_unproven_handover(); idle_jog_and_fencing();
    cancellation_before_tick(JE2_EVENT_STOP);
    cancellation_before_tick(JE2_EVENT_DISCONNECT);
    cancellation_before_tick(JE2_EVENT_REJECT);
    puts("PASS: TCP STOP/disconnect/reject cancels pending jog BEFORE engine tick");
    coalesced_cancel_and_eof();
    cleanup_first_error();
    bounded_jog_runtime(); event_budget_cancels_pending();
#if JE2_SIM_SESSION_EXPERIMENT
    session_tests();
#endif
    assert(!started && JEServerV2Status() == 0 && JEServerV2ButtonPort() == 0);
    puts("PASS: runtime helper integration only; PluginMain/SDK initialization NEVER invoked");
    return 0;
}