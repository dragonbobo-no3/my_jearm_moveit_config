#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "jeserver_v2_transport.h"
#include "jeserver_v2.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int descriptor_flags(int fd)
{
    int status = fcntl(fd, F_GETFL);
    int flags = fcntl(fd, F_GETFD);
    if (status < 0 || flags < 0 || fcntl(fd, F_SETFL, status | O_NONBLOCK) < 0 ||
        fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0) return -1;
    return 0;
}
void je2_transport_close(struct je2_transport *t)
{
    if (!t) return;
    if (t->opened) {
        if (t->client_fd >= 0) (void)close(t->client_fd);
        if (t->listener_fd >= 0) (void)close(t->listener_fd);
    }
    t->listener_fd = t->client_fd = -1;
    t->opened = false;
    t->bound_port = 0;
    t->connection_id = t->pending_disconnect = 0;
    t->used = 0;
}
static int server_failure(struct je2_transport *t)
{
    int error = errno;
    je2_transport_close(t);
    t->fatal = true;
    errno = error;
    return -1;
}
int je2_transport_open(struct je2_transport *t, uint16_t port)
{
    if (!t) { errno = EINVAL; return -1; }
    memset(t, 0, sizeof(*t));
    t->listener_fd = t->client_fd = -1;
    t->opened = true;
    t->listener_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (t->listener_fd < 0 || descriptor_flags(t->listener_fd)) return server_failure(t);
    struct sockaddr_in address = {.sin_family = AF_INET,
        .sin_addr = {.s_addr = htonl(INADDR_LOOPBACK)}, .sin_port = htons(port)};
    if (bind(t->listener_fd, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        listen(t->listener_fd, 8) < 0) return server_failure(t);
    socklen_t size = sizeof(address);
    if (getsockname(t->listener_fd, (struct sockaddr *)&address, &size) < 0)
        return server_failure(t);
    t->bound_port = ntohs(address.sin_port);
    return 0;
}
static int emit(struct je2_event *event, enum je2_event_type type, uint64_t id)
{
    *event = (struct je2_event){.type = type, .axis = -1, .connection_id = id};
    return 1;
}
static int disconnect_client(struct je2_transport *t, struct je2_event *event, bool reject)
{
    uint64_t id = t->connection_id;
    (void)close(t->client_fd);
    t->client_fd = -1;
    t->connection_id = 0;
    t->used = 0;
    if (reject) t->pending_disconnect = id;
    return emit(event, reject ? JE2_EVENT_REJECT : JE2_EVENT_DISCONNECT, id);
}
/* A delimiter-free suffix can only be a proper prefix of a strict button.
 * Unknown numeric codes are rejected by the real decoder when complete. */
static bool partial_button(const char *text, size_t size)
{
    if (!size) return true;
    if (size > 5 || text[0] != 't') return false;
    if (size > 1 && text[1] != 'v') return false;
    if (size > 3 && text[2] == '0') return false;
    for (size_t i = 2; i < size; ++i)
        if (text[i] < '0' || text[i] > '9') return false;
    return true;
}
int je2_transport_next(struct je2_transport *t, struct je2_event *event)
{
    if (!t || !event) { errno = EINVAL; return -1; }
    *event = (struct je2_event){.type = JE2_EVENT_NONE, .axis = -1};
    /* Cancellation cannot be lost to a new accept, a listener fault or an
     * empty read after the previous invocation returned REJECT. */
    if (t->pending_disconnect) {
        uint64_t id = t->pending_disconnect;
        t->pending_disconnect = 0;
        return emit(event, JE2_EVENT_DISCONNECT, id);
    }
    if (t->fatal || !t->opened) { errno = EBADF; return -1; }
    struct pollfd listener = {.fd = t->listener_fd, .events = POLLIN};
    int ready = poll(&listener, 1, 0);
    if (ready < 0) {
        if (errno == EINTR) return 0;
        return server_failure(t);
    }
    if (listener.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        errno = EIO; return server_failure(t);
    }
    if (listener.revents & POLLIN) {
        for (int attempt = 0; attempt < 8; ++attempt) {
            int fd = accept(t->listener_fd, NULL, NULL);
            if (fd < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                if (errno == EINTR || errno == ECONNABORTED) continue;
                return server_failure(t);
            }
            if (t->client_fd >= 0) { (void)close(fd); continue; }
            if (descriptor_flags(fd)) {
                int error = errno;
                (void)close(fd); errno = error;
                return server_failure(t);
            }
            if (t->last_connection_id == UINT64_MAX) {
                (void)close(fd); errno = EOVERFLOW;
                return server_failure(t);
            }
            t->client_fd = fd;
            t->connection_id = ++t->last_connection_id;
        }
    }
    if (t->client_fd < 0) return 0;
    struct pollfd client = {.fd = t->client_fd, .events = POLLIN};
    ready = poll(&client, 1, 0);
    if (ready < 0) {
        if (errno == EINTR) return 0;
        return server_failure(t);
    }
    if (client.revents & (POLLERR | POLLNVAL)) return disconnect_client(t, event, false);
    bool eof = false;
    if (client.revents & (POLLIN | POLLHUP)) {
        for (int attempt = 0; attempt < 4 && t->used < sizeof(t->buffer); ++attempt) {
            ssize_t count = recv(t->client_fd, t->buffer + t->used, sizeof(t->buffer) - t->used, 0);
            if (count > 0) { t->used += (size_t)count; continue; }
            if (!count) { eof = true; break; }
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            return disconnect_client(t, event, false);
        }
    }
    size_t begin = 0, last_stop = 0;
    int axis = -1, direction = 0;
    for (size_t i = 0; i < t->used; ++i) {
        if (t->buffer[i] != '!') continue;
        int decoded = je2_decode_button(t->buffer + begin, i + 1 - begin, &axis, &direction);
        if (decoded != 1 && decoded != 2) return disconnect_client(t, event, true);
        if (decoded == 2) last_stop = i + 1;
        begin = i + 1;
    }
    if (!partial_button(t->buffer + begin, t->used - begin) || (eof && begin != t->used))
        return disconnect_client(t, event, true);
    if (eof) return disconnect_client(t, event, false);
    if (last_stop) {
        memmove(t->buffer, t->buffer + last_stop, t->used - last_stop);
        t->used -= last_stop;
        return emit(event, JE2_EVENT_STOP, t->connection_id);
    }
    if (!begin) return 0;
    char *end = memchr(t->buffer, '!', t->used);
    size_t length = (size_t)(end - t->buffer) + 1;
    (void)je2_decode_button(t->buffer, length, &axis, &direction);
    memmove(t->buffer, t->buffer + length, t->used - length);
    t->used -= length;
    emit(event, JE2_EVENT_JOG, t->connection_id);
    event->axis = axis; event->direction = direction;
    return 1;
}