#ifndef JESERVER_V2_H
#define JESERVER_V2_H
#include "mk2_control_core.h"

/* Controller-side engine; no second SDK instance and no SDK symbol injection.
 * Transport callbacks must enqueue events, NEVER invoke mutation concurrently.
 * All calls below (including state reads) require one serialized executor.
 * The network/plugin adapter is NOT implemented by this engine.
 */
enum je2_owner { JE2_AVAILABLE, JE2_PC, JE2_YIELDING, JE2_EXIT_BARRIER, JE2_PENDANT, JE2_FAULT };
enum je2_writer { JE2_WRITER_UNKNOWN, JE2_WRITER_RUNNING, JE2_WRITER_EXITED };
struct je2_feedback { struct mk2_sample sample; double acquired_at; };
struct je2_io {
    void *context;
    /* Must refer to the specific writer instance captured on ServoStart.
     * Neither state!=13 nor ServoEnd()==0 is independent exit evidence. */
    enum je2_writer (*writer)(void *);
    int (*jog)(void *, int axis, int direction);
    int (*jog_stop)(void *);
};
struct jeserver_v2 {
    struct mk2_core core;
    struct mk2_io raw_io;
    struct je2_io teach;
    enum je2_owner owner;
    uint64_t generation, dropped_pc;
    bool legacy_open, pending, had_writer, jog_live, stop_issued, end_issued, jog_stop_issued, idle_qualified;
    int cleanup_error;
    int axis, direction;
    double last_now, request_at, release_at, stable_at, last_feedback_at, barrier_after;
    double stable_joint[MK2_AXES];
};
int je2_init(struct jeserver_v2 *, const struct mk2_policy *, const struct mk2_io *, const struct je2_io *);
/* generation==0 is the unchanged initial legacy Switch/Joint session ONLY.
 * After any yield/release it is permanently sealed. New PC clients must claim
 * a published generation, then attach the returned generation to every event. */
int je2_pc_frame(struct jeserver_v2 *, uint64_t generation, const char *, size_t,
                 double now, const struct je2_feedback *);
int je2_claim_pc(struct jeserver_v2 *, uint64_t observed_generation, double now,
                 const struct je2_feedback *);
/* Complete validated joint-jog intent from EXISTING pendant keys. No takeover
 * button/service required. Call stop/disconnect even while a yield is pending. */
int je2_pendant_jog(struct jeserver_v2 *, int axis, int direction, double received_at,
                     double now, const struct je2_feedback *);
int je2_pendant_stop(struct jeserver_v2 *, double now);
void je2_tick(struct jeserver_v2 *, double now, const struct je2_feedback *);
/* Trusted executor failure/unload, never a request from a retired PC session. */
void je2_abort(struct jeserver_v2 *, enum mk2_fault reason);
/* Only complete single text button packets; caller must assemble TCP streams.
 * 1=jog, 2=stop, 0=unsupported, -1=malformed. No SDK call or guessed aliases. */
int je2_decode_button(const char *, size_t, int *axis, int *direction);
#endif