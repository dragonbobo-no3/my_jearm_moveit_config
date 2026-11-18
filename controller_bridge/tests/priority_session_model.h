/* RESEARCH MODEL ONLY. No SDK, transport, locks, motion or deployment code.
 * A future adapter must serialize EVERY SDK-mutating entry with this policy,
 * authenticate/validate messages and supply independently established evidence.
 * Existing legacy Switch/Joint messages do NOT carry the required generation.
 * Never infer writer exit from move_state != 13, StopRun/End return, or this model.
 */
#ifndef MK2_PRIORITY_SESSION_MODEL_H
#define MK2_PRIORITY_SESSION_MODEL_H
#include <stdbool.h>
#include <stdint.h>

enum priority_owner { PRIORITY_UNAVAILABLE, PRIORITY_PENDANT, PRIORITY_PC, PRIORITY_YIELDING, PRIORITY_FAULT };
enum priority_result { PRIORITY_REJECT, PRIORITY_WAIT, PRIORITY_FORWARD, PRIORITY_CANCELLED, PRIORITY_FAILED };
struct priority_evidence {
    bool healthy_fresh, pc_quiesced, writer_exited, stationary;
};
struct priority_session {
    enum priority_owner owner;
    uint64_t generation, request, request_time, yield_time, last_time;
    uint64_t request_ttl, yield_timeout;
    bool pending, pendant_may_move;
};
static inline void priority_fault(struct priority_session *s)
{
    s->owner = PRIORITY_FAULT;
    s->pending = false;
    /* No further admission, including commands bearing this generation. */
}
static inline bool priority_clock(struct priority_session *s, uint64_t now)
{
    if (now < s->last_time) { priority_fault(s); return false; }
    s->last_time = now;
    return true;
}
static inline bool priority_next_generation(struct priority_session *s)
{
    if (s->generation == UINT64_MAX) { priority_fault(s); return false; }
    ++s->generation;
    return true;
}
static inline bool priority_complete(struct priority_evidence e)
{
    return e.healthy_fresh && e.pc_quiesced && e.writer_exited && e.stationary;
}
static inline void priority_init(struct priority_session *s, uint64_t ttl, uint64_t timeout)
{
    *s = (struct priority_session){.generation = 1, .request_ttl = ttl, .yield_timeout = timeout};
    if (!ttl || timeout < ttl) priority_fault(s);
}
static inline bool priority_ready(struct priority_session *s, uint64_t now, struct priority_evidence e)
{
    if (!priority_clock(s, now) || s->owner != PRIORITY_UNAVAILABLE || !priority_complete(e)) return false;
    s->owner = PRIORITY_PENDANT;
    return true;
}
/* A NEW PC handshake, not a legacy Switch replay. Generation is server-issued. */
static inline bool priority_claim_pc(struct priority_session *s, uint64_t now,
                                      uint64_t observed_generation, struct priority_evidence e)
{
    if (!priority_clock(s, now) || s->owner != PRIORITY_PENDANT || s->pendant_may_move || s->pending ||
        observed_generation != s->generation || !priority_complete(e)) return false;
    if (!priority_next_generation(s)) return false;
    s->owner = PRIORITY_PC;
    return true;
}
/* Applies identically to Start, Joint, Release, Stop and End. Stale cleanup
 * MUST NOT call an SDK-mutating function on behalf of the retired session. */
static inline bool priority_admit_pc(const struct priority_session *s, uint64_t generation)
{
    return s->owner == PRIORITY_PC && generation == s->generation;
}
/* Validated joint-jog only; request id and receive timestamp assigned by adapter.
 * Reject repeats/new presses during a barrier rather than silently queueing motion. */
static inline enum priority_result priority_press(struct priority_session *s, uint64_t now,
                                                   uint64_t request, uint64_t received, bool valid)
{
    if (!priority_clock(s, now)) return PRIORITY_FAILED;
    if (!valid || !request || request <= s->request || received > now || now - received > s->request_ttl ||
        (s->owner != PRIORITY_PC && s->owner != PRIORITY_PENDANT) || s->pendant_may_move) return PRIORITY_REJECT;
    s->request = request; s->request_time = received; s->pending = true;
    if (s->owner == PRIORITY_PC) {
        /* Invalidate the old writer BEFORE initiating any asynchronous yield. */
        if (!priority_next_generation(s)) return PRIORITY_FAILED;
        s->owner = PRIORITY_YIELDING;
        s->yield_time = now;
    }
    return PRIORITY_WAIT;
}
/* A validated pendant stop/release cancels any queued jog. Return true only if
 * forwarding a native pendant stop is appropriate; no automatic PC re-acquire. */
static inline bool priority_release_pendant(struct priority_session *s, uint64_t now)
{
    if (!priority_clock(s, now)) return false;
    s->pending = false;
    return s->owner == PRIORITY_PENDANT && s->pendant_may_move;
}
static inline enum priority_result priority_poll(struct priority_session *s, uint64_t now,
                                                 struct priority_evidence e)
{
    if (!priority_clock(s, now) || s->owner == PRIORITY_FAULT) return PRIORITY_FAILED;
    if (s->owner != PRIORITY_YIELDING && !(s->owner == PRIORITY_PENDANT && s->pending)) return PRIORITY_REJECT;
    if (!e.healthy_fresh) { priority_fault(s); return PRIORITY_FAILED; }
    if (s->owner == PRIORITY_YIELDING && now - s->yield_time >= s->yield_timeout) {
        priority_fault(s); return PRIORITY_FAILED;
    }
    if (s->pending && now - s->request_time > s->request_ttl) s->pending = false;
    if (!priority_complete(e)) {
        return PRIORITY_WAIT;
    }
    s->owner = PRIORITY_PENDANT;
    if (!s->pending) return PRIORITY_CANCELLED;
    s->pending = false;
    s->pendant_may_move = true;
    return PRIORITY_FORWARD; /* Exactly once, only for this still-fresh press. */
}
/* After a forwarded jog/stop, only independently qualified idle evidence lets
 * a NEW PC handshake proceed; a button release by itself is insufficient. */
static inline bool priority_pendant_idle(struct priority_session *s, uint64_t now,
                                         struct priority_evidence e)
{
    if (!priority_clock(s, now) || s->owner != PRIORITY_PENDANT || !priority_complete(e)) return false;
    s->pendant_may_move = false;
    return true;
}
#endif