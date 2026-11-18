#ifndef JESERVER_V2_SESSION_H
#define JESERVER_V2_SESSION_H
#include <stdbool.h>
#include <stdint.h>
/* Executor-owned experimental transport state, NOT authentication or stop
 * provenance. A new kernel-random boot token is required on EVERY load. */
struct je2_session {
    char boot[33], client[33];
    uint64_t generation, last_sequence;
    /* Accepted canonical Release from ACTIVE in this exact session. Zero is
     * no request; consumption alone must never populate this receipt. */
    uint64_t release_sequence;
    bool discarding_parts;
};
#endif