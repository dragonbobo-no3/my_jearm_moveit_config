#ifndef JE2_SESSION_CLIENT_H
#define JE2_SESSION_CLIENT_H
#include "jeserver_v2.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Portable client binding component for a future ROS adapter. NO sockets,
 * SDK calls, trajectory storage, auto retry, clear/reset or shared globals.
 * Caller serializes observe/output/send-result and clears ROS goal queues on
 * terminal state. Public tokens are correlation IDs, NOT authentication. */
enum je2_client_phase { JE2_CLIENT_OBSERVING, JE2_CLIENT_CLAIMING, JE2_CLIENT_BOUND,
    JE2_CLIENT_SWITCHING, JE2_CLIENT_HOLD_READY, JE2_CLIENT_HOLDING,
    JE2_CLIENT_ACTIVE, JE2_CLIENT_RETIRED, JE2_CLIENT_FAULT,
    JE2_CLIENT_DRAINING, JE2_CLIENT_CLOSING, JE2_CLIENT_CLOSED };
/* Capture by value when accepting a NEW goal, not at callback execution time.
 * Numeric generation alone is insufficient across server/client restarts. */
struct je2_client_lease {
    char boot[33], client[33];
    uint64_t generation;
};
struct je2_client {
    enum je2_client_phase phase;
    char boot[33], client[33];
    uint64_t generation, observation, wire_sequence, acknowledged;
    double now, received_at, deadline, next_claim;
    double close_deadline;
    double first_target_at, first_data_time, last_data_time;
    int owner, native_phase;
    bool seen, release_supported;
    double joint[MK2_AXES];
    /* Exact formatted first hold, NOT subsequently observed feedback. Used by
     * higher-level validators to seed command finite differences at zero speed. */
    double first_target[MK2_AXES];
};
int je2_client_init(struct je2_client *);
/* 0=new valid observation, 1=duplicate (does NOT refresh freshness), -1=fail.
 * A boot/owner/generation change retires an engaged session with no stop frame. */
int je2_client_observe(struct je2_client *, const char *, size_t, double now);
int je2_client_tick(struct je2_client *, double now);
/* Explicit claim operation may be polled for the SAME request (>=20ms apart)
 * within its fixed 500ms deadline. No autonomous sends or deadline extension. */
int je2_client_claim(struct je2_client *, double now, char *, size_t);
int je2_client_switch(struct je2_client *, double now, char *, size_t);
/* Initial hold is generated ONLY from post-Switch-ACK snapshot feedback.
 * No API accepts an old queued trajectory or caller-supplied initial target. */
int je2_client_hold(struct je2_client *, double now, double dt, char *, size_t);
int je2_client_get_lease(const struct je2_client *, struct je2_client_lease *);
/* Explicit current-session target, no queue, retiming or retransmission.
 * data_time is cumulative, starting at the initial hold's time, NOT per-frame dt.
 * >0: one formatted frame, 0: one command still awaiting ACK (nothing queued),
 * -1: refused. A stale lease is dropped BEFORE clock/target validation and does
 * not fault the current session. Current-session invalid time/finite values
 * fault. Caller must validate trajectory limits (e.g. existing ROS validator);
 * this protocol component is NOT a planner/limit validator. Server guards remain
 * authoritative. At most one outstanding target, fixed 100ms ACK deadline.
 * Serialize formatting with immediate send; report any send failure, no retry. */
int je2_client_target(struct je2_client *, const struct je2_client_lease *, double now,
                      double data_time, const double joint[MK2_AXES], char *, size_t);
/* Explicit experimental close, NOT ROS goal success or stop provenance.
 * Requires validated release-receipt fields in a fresh Owner publication;
 * older servers without those fields are refused before formatting Release.
 * First call freezes target output, waits for any outstanding target's ACK
 * without extending its deadline, then formats canonical Release exactly once.
 * Polling returns 0 while waiting; >0 is the only Release frame, -1 is refusal.
 * Fixed total deadline 500ms, release ACK 100ms, no retries/auto recovery.
 * CLOSED is terminal and requires a request-correlated engine barrier receipt;
 * generation change without that receipt retires, never sends cleanup. */
int je2_client_close(struct je2_client *, const struct je2_client_lease *, double now, char *, size_t);
void je2_client_send_failed(struct je2_client *);
void je2_client_retire(struct je2_client *);
#ifdef __cplusplus
}
#endif
#endif