#ifndef JESERVER_V2_TRANSPORT_H
#define JESERVER_V2_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* SIMULATION-ONLY text BUTTON ingress prototype, not the full pendant
 * 6666/6667/6668 protocol and NOT production-UI compatible. No SDK calls.
 * One serialized executor must own open/next/close and consume every event.
 * STOP and DISCONNECT must cancel pending intent/motion in that executor;
 * REJECT is diagnostic, NEVER a substitute for consuming DISCONNECT.
 */
enum je2_event_type {
    JE2_EVENT_NONE, JE2_EVENT_JOG, JE2_EVENT_STOP,
    JE2_EVENT_DISCONNECT, JE2_EVENT_REJECT
};
struct je2_event {
    enum je2_event_type type;
    int axis, direction; /* axis=-1, direction=0 except for JOG */
    uint64_t connection_id;
};

#define JE2_TRANSPORT_BUFFER_SIZE 128
struct je2_transport {
    /* Descriptors are exposed for an outer poll loop, but owned here. */
    int listener_fd, client_fd;
    uint16_t bound_port;
    /* IDs are server-assigned, nonzero, monotonic within one open lifetime.
     * Overflow is fatal, never wraps. These remaining fields are private. */
    uint64_t last_connection_id, connection_id, pending_disconnect;
    char buffer[JE2_TRANSPORT_BUFFER_SIZE];
    size_t used;
    bool opened, fatal;
};

/* Initializes *t (even on failure); do not open an already-open object.
 * Binds IPv4 INADDR_LOOPBACK only. port=0 selects an ordinary ephemeral port,
 * reported in bound_port. All owned descriptors are nonblocking/CLOEXEC.
 * Returns 0 on success, -1 on failure. No runtime simulation attestation is
 * performed here: a future adapter must enforce simulation-only use itself. */
int je2_transport_open(struct je2_transport *t, uint16_t port);
/* 1=one event, 0=none (event is NONE), -1=latched fatal server error.
 * Never waits/sleeps: poll timeout zero, <=8 accept and <=4 recv attempts.
 * Only one active client; competitors are closed without events or takeover.
 * Valid split frames may wait for more bytes. Impossible/overlong prefixes,
 * malformed/unknown complete frames or EOF with a partial frame produce
 * REJECT, then a latched DISCONNECT before any later client can produce JOG.
 * Every complete frame in the <=128-byte receive window is validated before
 * returning JOG. The LAST buffered STOP discards itself and all earlier
 * commands; STOP is returned first, later commands remain in stream order.
 * This cannot prioritize a STOP still outside that bounded receive window.
 * Active-client I/O errors/EOF discard queued jogs and emit DISCONNECT once.
 * On -1 the caller must cancel motion (no further events are guaranteed).
 */
int je2_transport_next(struct je2_transport *t, struct je2_event *event);
/* Idempotent after initialization/open, including failed open. Discards events
 * and closes all owned resources; caller must cancel motion BEFORE close. */
void je2_transport_close(struct je2_transport *t);

#endif