#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "../jeserver_v2_transport.h"
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/* Ordinary loopback ephemeral sockets only; no SDK, controller or threads.
 * Every test wait is bounded poll, never a sleep or unbounded busy loop. */
static void flags_ok(int fd)
{
    assert(fcntl(fd, F_GETFL) & O_NONBLOCK);
    assert(fcntl(fd, F_GETFD) & FD_CLOEXEC);
}
static void wait_fd(int fd, short events)
{
    struct pollfd p = {.fd = fd, .events = events};
    assert(poll(&p, 1, 1000) == 1);
    assert(!(p.revents & POLLNVAL));
}
static int connect_client(const struct je2_transport *t)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    assert(fd >= 0);
    assert(fcntl(fd, F_SETFL, O_NONBLOCK) == 0);
    assert(fcntl(fd, F_SETFD, FD_CLOEXEC) == 0);
    struct sockaddr_in address = {.sin_family = AF_INET,
        .sin_addr = {.s_addr = htonl(INADDR_LOOPBACK)}, .sin_port = htons(t->bound_port)};
    int result = connect(fd, (struct sockaddr *)&address, sizeof(address));
    assert(result == 0 || (result == -1 && errno == EINPROGRESS));
    wait_fd(fd, POLLOUT);
    int error = -1;
    socklen_t size = sizeof(error);
    assert(getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &size) == 0 && !error);
    flags_ok(fd);
    return fd;
}
static void send_bytes(int fd, const char *text, size_t length)
{
    /* All test payloads fit in a freshly writable ordinary TCP send buffer. */
    wait_fd(fd, POLLOUT);
    assert(send(fd, text, length, 0) == (ssize_t)length);
}
static void send_text(int fd, const char *text) { send_bytes(fd, text, strlen(text)); }
static struct je2_event next_event(struct je2_transport *t)
{
    struct je2_event event;
    for (int attempt = 0; attempt < 8; ++attempt) {
        int result = je2_transport_next(t, &event);
        assert(result >= 0);
        if (result == 1) return event;
        struct pollfd fds[] = {{.fd = t->listener_fd, .events = POLLIN},
            {.fd = t->client_fd, .events = POLLIN}};
        assert(poll(fds, 2, 1000) > 0);
    }
    assert(!"event did not arrive within bounded service attempts");
    return event;
}
static void expect_none(struct je2_transport *t)
{
    struct je2_event event = {.type = JE2_EVENT_REJECT, .axis = 99, .direction = 99,
        .connection_id = UINT64_MAX};
    assert(je2_transport_next(t, &event) == 0);
    assert(event.type == JE2_EVENT_NONE && event.axis == -1 && !event.direction && !event.connection_id);
}
static void open_transport(struct je2_transport *t)
{
    assert(je2_transport_open(t, 0) == 0 && t->bound_port);
    flags_ok(t->listener_fd);
    struct sockaddr_in address;
    socklen_t size = sizeof(address);
    assert(getsockname(t->listener_fd, (struct sockaddr *)&address, &size) == 0);
    assert(address.sin_family == AF_INET && address.sin_addr.s_addr == htonl(INADDR_LOOPBACK));
    assert(ntohs(address.sin_port) == t->bound_port);
    expect_none(t);
}
static int accept_client(struct je2_transport *t)
{
    int fd = connect_client(t);
    wait_fd(t->listener_fd, POLLIN);
    expect_none(t);
    assert(t->client_fd >= 0 && t->connection_id);
    flags_ok(t->client_fd);
    return fd;
}
static void expect_event(struct je2_transport *t, enum je2_event_type type, uint64_t id)
{
    struct je2_event event = next_event(t);
    assert(event.type == type && event.connection_id == id);
    assert(event.axis == -1 && !event.direction);
}
static void expect_jog(struct je2_transport *t, uint64_t id, int axis, int direction)
{
    struct je2_event event = next_event(t);
    assert(event.type == JE2_EVENT_JOG && event.connection_id == id);
    assert(event.axis == axis && event.direction == direction);
}
static void finish(struct je2_transport *t, int fd)
{
    if (fd >= 0) assert(close(fd) == 0);
    int listener = t->listener_fd, client = t->client_fd;
    je2_transport_close(t);
    if (listener >= 0) assert(fcntl(listener, F_GETFD) == -1 && errno == EBADF);
    if (client >= 0) assert(fcntl(client, F_GETFD) == -1 && errno == EBADF);
    assert(t->listener_fd == -1 && t->client_fd == -1 && !t->bound_port && !t->used);
    je2_transport_close(t);
}
static void partial_and_concatenated(void)
{
    struct je2_transport t; open_transport(&t);
    int fd = accept_client(&t);
    uint64_t id = t.connection_id;
    send_text(fd, "tv4"); wait_fd(t.client_fd, POLLIN); expect_none(&t);
    assert(t.used == 3);
    send_text(fd, "9!"); expect_jog(&t, id, 0, 1); expect_none(&t);
    send_text(fd, "tv50!tv113!tv117!");
    expect_jog(&t, id, 1, 1); expect_jog(&t, id, 0, -1); expect_jog(&t, id, 6, -1);
    expect_none(&t); finish(&t, fd);
}
static void all_fourteen_jogs(void)
{
    struct je2_transport t; open_transport(&t);
    int fd = accept_client(&t);
    const unsigned negative[] = {113, 119, 101, 114, 116, 121, 117};
    for (int direction = -1; direction <= 1; direction += 2) {
        for (int axis = 0; axis < 7; ++axis) {
            char packet[16];
            int size = snprintf(packet, sizeof(packet), "tv%u!", direction == 1 ? 49U + (unsigned)axis : negative[axis]);
            send_bytes(fd, packet, (size_t)size);
            expect_jog(&t, t.connection_id, axis, direction);
        }
    }
    finish(&t, fd);
}
static void stop_priority(void)
{
    struct je2_transport t; open_transport(&t);
    int fd = accept_client(&t);
    uint64_t id = t.connection_id;
    send_text(fd, "tv49!tv50!tv33!");
    expect_event(&t, JE2_EVENT_STOP, id); expect_none(&t);
    send_text(fd, "tv49!tv33!tv50!tv33!tv51!tv1");
    expect_event(&t, JE2_EVENT_STOP, id); expect_jog(&t, id, 2, 1); expect_none(&t);
    send_text(fd, "13!"); expect_jog(&t, id, 0, -1);
    /* A stop newly received on next() also cancels already-buffered jogs. */
    send_text(fd, "tv49!tv50!"); expect_jog(&t, id, 0, 1);
    assert(t.used == 5);
    send_text(fd, "tv33!"); wait_fd(t.client_fd, POLLIN);
    expect_event(&t, JE2_EVENT_STOP, id); expect_none(&t);
    finish(&t, fd);
}
static void rejected_input(void)
{
    static const char *const bad[] = {
        "bad!", "tv999!", "tv049!", "tv!", "tv-49!", "tv49!!", "tv49!tv99!",
        "tv49!tv33!unknown!", "tv1234", "garbage without delimiter", "tv49\n!"
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        struct je2_transport t; open_transport(&t);
        int fd = accept_client(&t);
        uint64_t id = t.connection_id;
        send_text(fd, "tv49!"); expect_jog(&t, id, 0, 1);
        send_text(fd, bad[i]);
        expect_event(&t, JE2_EVENT_REJECT, id);
        assert(t.client_fd == -1 && !t.used);
        /* A waiting replacement cannot bypass cancellation, even on the
         * invocation immediately following REJECT. */
        int replacement = connect_client(&t);
        send_text(replacement, "tv50!");
        expect_event(&t, JE2_EVENT_DISCONNECT, id);
        expect_jog(&t, id + 1, 1, 1); expect_none(&t);
        assert(close(fd) == 0); finish(&t, replacement);
    }
    struct je2_transport t; open_transport(&t);
    int fd = accept_client(&t);
    uint64_t id = t.connection_id;
    char oversized[512]; memset(oversized, '1', sizeof(oversized));
    send_bytes(fd, oversized, sizeof(oversized));
    expect_event(&t, JE2_EVENT_REJECT, id);
    expect_event(&t, JE2_EVENT_DISCONNECT, id); expect_none(&t);
    finish(&t, fd);
}
static void receive_window_boundaries(void)
{
    struct je2_transport t; open_transport(&t);
    int fd = accept_client(&t);
    uint64_t id = t.connection_id;
    char window[JE2_TRANSPORT_BUFFER_SIZE];
    /* Exactly 128 bytes: valid concatenated frames plus a split suffix, not
     * an oversized individual frame. STOP cancels all 24 preceding jogs. */
    for (size_t i = 0; i < 24; ++i) memcpy(window + i * 5, "tv49!", 5);
    memcpy(window + 120, "tv33!tv4", 8);
    send_bytes(fd, window, sizeof(window));
    expect_event(&t, JE2_EVENT_STOP, id); assert(t.used == 3); expect_none(&t);
    send_text(fd, "9!"); expect_jog(&t, id, 0, 1);
    /* Embedded NUL is data, never a C-string terminator/bypass. */
    const char binary[] = {'t', 'v', '4', '9', '\0', '!'};
    send_bytes(fd, binary, sizeof(binary));
    expect_event(&t, JE2_EVENT_REJECT, id);
    expect_event(&t, JE2_EVENT_DISCONNECT, id); expect_none(&t);
    assert(close(fd) == 0);
    fd = accept_client(&t); id = t.connection_id;
    /* EOF cancels queued complete jogs too, not just a live delivered jog. */
    send_text(fd, "tv49!tv50!"); expect_jog(&t, id, 0, 1); assert(t.used == 5);
    assert(shutdown(fd, SHUT_WR) == 0); wait_fd(t.client_fd, POLLIN);
    expect_event(&t, JE2_EVENT_DISCONNECT, id); expect_none(&t);
    finish(&t, fd);
}
static void disconnects(void)
{
    struct je2_transport t; open_transport(&t);
    int fd = accept_client(&t);
    uint64_t id = t.connection_id;
    assert(close(fd) == 0);
    expect_event(&t, JE2_EVENT_DISCONNECT, id); expect_none(&t); expect_none(&t);
    fd = accept_client(&t); assert(t.connection_id == id + 1); id = t.connection_id;
    send_text(fd, "tv4"); wait_fd(t.client_fd, POLLIN); expect_none(&t);
    assert(shutdown(fd, SHUT_WR) == 0);
    expect_event(&t, JE2_EVENT_REJECT, id);
    expect_event(&t, JE2_EVENT_DISCONNECT, id); expect_none(&t);
    assert(close(fd) == 0);
    fd = accept_client(&t); assert(t.connection_id == id + 1); id = t.connection_id;
    send_text(fd, "tv49!"); expect_jog(&t, id, 0, 1);
    struct linger reset = {.l_onoff = 1, .l_linger = 0};
    assert(setsockopt(fd, SOL_SOCKET, SO_LINGER, &reset, sizeof(reset)) == 0);
    assert(close(fd) == 0);
    expect_event(&t, JE2_EVENT_DISCONNECT, id); expect_none(&t);
    finish(&t, -1);
}
static void second_client(void)
{
    struct je2_transport t; open_transport(&t);
    int first = accept_client(&t);
    uint64_t id = t.connection_id;
    int active = t.client_fd;
    int second = connect_client(&t);
    send_text(second, "tv117!");
    wait_fd(t.listener_fd, POLLIN); expect_none(&t);
    assert(t.client_fd == active && t.connection_id == id && t.last_connection_id == id);
    wait_fd(second, POLLIN);
    char byte;
    ssize_t count = recv(second, &byte, 1, 0);
    assert(count == 0 || (count == -1 && errno == ECONNRESET));
    assert(close(second) == 0);
    send_text(first, "tv49!"); expect_jog(&t, id, 0, 1);
    assert(close(first) == 0); expect_event(&t, JE2_EVENT_DISCONNECT, id);
    first = accept_client(&t); assert(t.connection_id == id + 1);
    send_text(first, "tv113!"); expect_jog(&t, id + 1, 0, -1);
    finish(&t, first);
}
static void failures_and_cleanup(void)
{
    struct je2_transport t; open_transport(&t);
    struct je2_transport occupied;
    assert(je2_transport_open(&occupied, t.bound_port) == -1);
    assert(occupied.listener_fd == -1 && occupied.client_fd == -1);
    je2_transport_close(&occupied);
    int fd = accept_client(&t);
    send_text(fd, "tv49!"); expect_jog(&t, t.connection_id, 0, 1);
    /* Inject a listener failure without leaving another descriptor owned by
     * the fixture. The next service must fail closed and remain fatal. */
    assert(shutdown(t.listener_fd, SHUT_RDWR) == 0);
    struct je2_event event;
    assert(je2_transport_next(&t, &event) == -1 && t.fatal);
    assert(t.listener_fd == -1 && t.client_fd == -1);
    assert(je2_transport_next(&t, &event) == -1);
    finish(&t, fd);
    open_transport(&t);
    t.last_connection_id = UINT64_MAX - 1;
    fd = accept_client(&t); assert(t.connection_id == UINT64_MAX);
    assert(close(fd) == 0); expect_event(&t, JE2_EVENT_DISCONNECT, UINT64_MAX);
    fd = connect_client(&t); wait_fd(t.listener_fd, POLLIN);
    assert(je2_transport_next(&t, &event) == -1 && t.fatal);
    assert(t.last_connection_id == UINT64_MAX && t.client_fd == -1 && t.listener_fd == -1);
    finish(&t, fd);
    /* Close with both descriptors and a buffered partial frame. */
    open_transport(&t); fd = accept_client(&t);
    send_text(fd, "tv4"); wait_fd(t.client_fd, POLLIN); expect_none(&t);
    finish(&t, fd);
    /* Close while cancellation is latched is explicit teardown, not an event. */
    open_transport(&t); fd = accept_client(&t); send_text(fd, "bad!");
    expect_event(&t, JE2_EVENT_REJECT, t.connection_id);
    assert(t.pending_disconnect); finish(&t, fd); assert(!t.pending_disconnect);
    /* The required cancellation survives a listener failure between events. */
    open_transport(&t); fd = accept_client(&t);
    uint64_t id = t.connection_id;
    send_text(fd, "tv999!"); expect_event(&t, JE2_EVENT_REJECT, id);
    assert(shutdown(t.listener_fd, SHUT_RDWR) == 0);
    expect_event(&t, JE2_EVENT_DISCONNECT, id);
    assert(je2_transport_next(&t, &event) == -1 && t.fatal);
    finish(&t, fd);
}
int main(void)
{
    partial_and_concatenated(); all_fourteen_jogs(); stop_priority();
    rejected_input(); receive_window_boundaries(); disconnects(); second_client(); failures_and_cleanup();
    puts("PASS: 8 transport suites (14 jogs, framing, STOP priority, reject/cancel, 128-byte window, disconnect, single client, fail-closed cleanup)");
    return 0;
}