#include "../je2_session_client.h"
#include <assert.h>
#include <json-c/json.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *boot = "abcdefabcdefabcdefabcdefabcdefab";
static struct json_object *snapshot(const struct je2_client *c, uint64_t seq,
                                    uint64_t gen, uint64_t ack, int owner, int phase)
{
    struct json_object *o = json_object_new_object(), *f = json_object_new_object(), *q = json_object_new_array();
    assert(o && f && q);
#define STR(k, v) json_object_object_add(o, k, json_object_new_string(v))
#define NUM(k, v) json_object_object_add(o, k, json_object_new_uint64(v))
#define BOOL(k, v) json_object_object_add(o, k, json_object_new_boolean(v))
    STR("implementation", "JEServer_v2"); STR("session_boot", boot);
    STR("session_client", gen == 1 ? "" : c->client);
    BOOL("experimental", true); BOOL("loopback_only", true); BOOL("session_transport_enabled", true);
    BOOL("automatic_priority_ready", false); BOOL("writer_exit_verified", false); BOOL("pc_reclaim_supported", false);
    NUM("snapshot_version", 1); NUM("snapshot_sequence", seq); NUM("generation", gen);
    NUM("session_generation", gen == 1 ? 0 : gen); NUM("session_last_sequence", ack);
    NUM("session_release_sequence", 0); NUM("session_release_phase", 0);
    NUM("owner", owner); NUM("snapshot_phase", phase); NUM("snapshot_fault", 0); NUM("snapshot_sdk_error", 0);
#undef STR
#undef NUM
#undef BOOL
    json_object_object_add(f, "valid", json_object_new_boolean(true));
    json_object_object_add(f, "PowerState", json_object_new_int(1));
    json_object_object_add(f, "MoveState", json_object_new_int(phase == MK2_ACTIVE ? 13 : 0));
    json_object_object_add(f, "DriverError", json_object_new_int(0));
    json_object_object_add(f, "MoveError", json_object_new_int(0));
    for (int i = 0; i < MK2_AXES; ++i) json_object_array_add(q, json_object_new_double(i * .01));
    json_object_object_add(f, "Joint", q); json_object_object_add(o, "snapshot_feedback", f);
    return o;
}
static int observe(struct je2_client *c, struct json_object *o, double now)
{
    char wire[MK2_FRAME_MAX];
    int n = snprintf(wire, sizeof(wire), "Owner %s", json_object_to_json_string_ext(o, JSON_C_TO_STRING_PLAIN));
    assert(n > 0 && (size_t)n < sizeof(wire));
    int result = je2_client_observe(c, wire, (size_t)n, now);
    json_object_put(o); return result;
}
static void bind(struct je2_client *c)
{
    char wire[MK2_FRAME_MAX]; assert(je2_client_init(c) == 0);
    assert(observe(c, snapshot(c, 1, 1, 0, JE2_AVAILABLE, MK2_READY), .01) == 0);
    assert(je2_client_claim(c, .02, wire, sizeof(wire)) > 0);
    assert(!strncmp(wire, "JE2C1 abcdef", 12));
    assert(je2_client_switch(c, .025, wire, sizeof(wire)) == -1 && !wire[0]);
    assert(observe(c, snapshot(c, 2, 2, 0, JE2_AVAILABLE, MK2_READY), .03) == 0);
    assert(c->phase == JE2_CLIENT_BOUND && !c->wire_sequence);
}
static void activated(struct je2_client *c)
{
    char wire[MK2_FRAME_MAX]; bind(c);
    assert(je2_client_switch(c, .04, wire, sizeof(wire)) > 0);
    assert(c->wire_sequence == 1 && strstr(wire, "Switch {\"Switch\":true}"));
    assert(je2_client_hold(c, .045, .01, wire, sizeof(wire)) == -1 && !wire[0]);
    assert(observe(c, snapshot(c, 3, 2, 1, JE2_PC, MK2_WAIT_FIRST), .05) == 0);
    assert(je2_client_hold(c, .06, .01, wire, sizeof(wire)) > 0);
    assert(c->wire_sequence == 2 && strstr(wire, "Joint "));
    assert(observe(c, snapshot(c, 4, 2, 2, JE2_PC, MK2_ACTIVE), .07) == 0);
    assert(c->phase == JE2_CLIENT_ACTIVE);
    assert(je2_client_hold(c, .08, .01, wire, sizeof(wire)) == -1 && !wire[0]);
}
static void lifecycle_and_retirement(void)
{
    struct je2_client c; activated(&c);
    assert(observe(&c, snapshot(&c, 5, 3, 2, JE2_YIELDING, MK2_RELEASING), .09) == -1);
    assert(c.phase == JE2_CLIENT_RETIRED);
    assert(observe(&c, snapshot(&c, 6, 2, 2, JE2_PC, MK2_ACTIVE), .1) == -1);
    char wire[4096] = "old target";
    assert(je2_client_claim(&c, .11, wire, sizeof(wire)) == -1 && !wire[0]);
    assert(je2_client_hold(&c, .12, .01, wire, sizeof(wire)) == -1 && !wire[0]);
    activated(&c);
    struct json_object *o = snapshot(&c, 1, 2, 2, JE2_PC, MK2_ACTIVE);
    json_object_object_add(o, "session_boot", json_object_new_string("11111111111111111111111111111111"));
    assert(observe(&c, o, .09) == -1 && c.phase == JE2_CLIENT_RETIRED);
    puts("PASS: explicit claim/Switch/fresh hold ACKs; yield/restart permanently retire and emit no cleanup or replay");
}
static void deadlines_and_send_failure(void)
{
    struct je2_client c; char wire[4096], original[4096];
    assert(je2_client_init(&c) == 0);
    assert(observe(&c, snapshot(&c, 1, 1, 0, JE2_AVAILABLE, MK2_READY), .01) == 0);
    assert(je2_client_claim(&c, .02, original, sizeof(original)) > 0);
    double deadline = c.deadline;
    assert(je2_client_claim(&c, .025, wire, sizeof(wire)) == 0 && !wire[0]);
    assert(je2_client_claim(&c, .05, wire, sizeof(wire)) > 0 && !strcmp(wire, original));
    assert(c.deadline == deadline);
    for (int i = 2; i <= 6; ++i)
        assert(observe(&c, snapshot(&c, (uint64_t)i, 1, 0, JE2_AVAILABLE, MK2_READY), .05 + .08 * (i - 1)) == 0);
    assert(je2_client_claim(&c, .53, wire, sizeof(wire)) == -1 && c.phase == JE2_CLIENT_FAULT);
    activated(&c);
    assert(observe(&c, snapshot(&c, 4, 2, 2, JE2_PC, MK2_ACTIVE), .15) == 1);
    assert(c.received_at == .07);
    assert(je2_client_tick(&c, .23) == -1 && c.phase == JE2_CLIENT_FAULT);
    bind(&c); je2_client_send_failed(&c);
    assert(je2_client_switch(&c, .04, wire, sizeof(wire)) == -1 && !wire[0]);
    bind(&c); assert(je2_client_tick(&c, NAN) == -1);
    bind(&c); assert(je2_client_tick(&c, .01) == -1);
    puts("PASS: fixed claim deadline, duplicate snapshots do not refresh freshness, clock/send failures latch");
}
static void bad_ack_and_feedback(void)
{
    for (int which = 0; which < 14; ++which) {
        struct je2_client c; bind(&c);
        struct json_object *o = snapshot(&c, 3, 2, 0, JE2_AVAILABLE, MK2_READY), *f = NULL;
        assert(json_object_object_get_ex(o, "snapshot_feedback", &f));
        switch (which) {
        case 0: json_object_object_add(o, "session_client", json_object_new_string("22222222222222222222222222222222")); break;
        case 1: json_object_object_add(o, "session_last_sequence", json_object_new_int(1)); break;
        case 2: json_object_object_add(o, "snapshot_phase", json_object_new_int(MK2_ACTIVE)); break;
        case 3: json_object_object_add(o, "snapshot_fault", json_object_new_int(1)); break;
        case 4: json_object_object_add(o, "snapshot_sdk_error", json_object_new_int(-1)); break;
        case 5: json_object_object_add(f, "MoveState", json_object_new_int(7)); break;
        case 6: json_object_object_add(f, "MoveState", json_object_new_int(-1)); break;
        case 7: json_object_object_add(f, "PowerState", json_object_new_int(0)); break;
        case 8: json_object_object_add(f, "DriverError", json_object_new_int(1)); break;
        case 9: json_object_object_add(f, "MoveError", json_object_new_int(1)); break;
        case 10: json_object_object_add(f, "Joint", json_object_new_array()); break;
        case 11: json_object_object_add(o, "session_transport_enabled", json_object_new_boolean(false)); break;
        case 12: json_object_object_add(o, "generation", json_object_new_double(2)); break;
        default: json_object_object_add(f, "valid", json_object_new_boolean(false)); break;
        }
        assert(observe(&c, o, .04) == -1);
        assert(c.phase == JE2_CLIENT_RETIRED || c.phase == JE2_CLIENT_FAULT);
        char out[4096]; assert(je2_client_switch(&c, .05, out, sizeof(out)) == -1 && !out[0]);
    }
    puts("PASS: 14 ACK/schema/health failures refuse binding and subsequent output");
}
static void malformed_and_duplicate_keys(void)
{
    const char *suffix[] = {",\"generation\":2}", ",\"snapshot_sequence\":3}",
        ",\"snapshot_feedback\":{}}", ",\"session_boot\":\"11111111111111111111111111111111\"}",
        "} trailing", ",\"generation\":18446744073709551616}"};
    for (size_t i = 0; i < sizeof(suffix) / sizeof(suffix[0]); ++i) {
        struct je2_client c; bind(&c);
        struct json_object *o = snapshot(&c, 3, 2, 0, JE2_AVAILABLE, MK2_READY);
        const char *body = json_object_to_json_string_ext(o, JSON_C_TO_STRING_PLAIN);
        char wire[4096]; int n = snprintf(wire, sizeof(wire), "Owner %.*s%s", (int)strlen(body) - 1, body, suffix[i]);
        assert(n > 0 && je2_client_observe(&c, wire, (size_t)n, .04) == -1);
        assert(c.phase == JE2_CLIENT_FAULT); json_object_put(o);
    }
    struct je2_client c; bind(&c); char short_buffer[2] = "x";
    assert(je2_client_switch(&c, .04, short_buffer, sizeof(short_buffer)) == -1 && !short_buffer[0]);
    assert(c.phase == JE2_CLIENT_FAULT && !c.wire_sequence);
    puts("PASS: duplicate keys/trailing data/overflow and truncated output fail closed");
}
static void structured_numeric_fields(void)
{
    const char *keys[] = {"snapshot_version", "snapshot_sequence", "generation", "session_generation",
        "session_last_sequence", "owner", "snapshot_phase", "snapshot_fault", "snapshot_sdk_error",
        "PowerState", "MoveState", "DriverError", "MoveError"};
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        struct je2_client c; bind(&c);
        struct json_object *o = snapshot(&c, 3, 2, 0, JE2_AVAILABLE, MK2_READY), *f = NULL, *v = NULL;
        assert(json_object_object_get_ex(o, "snapshot_feedback", &f));
        struct json_object *from = i < 9 ? o : f, *to = i < 9 ? f : o;
        assert(json_object_object_get_ex(from, keys[i], &v));
        json_object_get(v); json_object_object_del(from, keys[i]); json_object_object_add(to, keys[i], v);
        assert(observe(&c, o, .04) == -1 && c.phase == JE2_CLIENT_FAULT);
    }
    const char *bad[] = {"18446744073709551616", "-1", "02", "2.0", "2e0", "\"2\"", "true", "null"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        struct je2_client c; bind(&c);
        struct json_object *o = snapshot(&c, 3, 2, 0, JE2_AVAILABLE, MK2_READY);
        const char *body = json_object_to_json_string_ext(o, JSON_C_TO_STRING_PLAIN);
        const char *number = strstr(body, "\"generation\":2"); assert(number); number += strlen("\"generation\":");
        char wire[4096]; int n = snprintf(wire, sizeof(wire), "Owner %.*s%s%s", (int)(number - body), body, bad[i], number + 1);
        assert(n > 0 && je2_client_observe(&c, wire, (size_t)n, .04) == -1);
        assert(c.phase == JE2_CLIENT_FAULT); json_object_put(o);
    }
    puts("PASS: all 13 numeric fields require correct nesting; eight noncanonical/overflow values fail without duplicate keys");
}
static void fresh_hold_and_stale_claim(void)
{
    struct je2_client c; char wire[4096]; bind(&c);
    assert(c.joint[0] == 0);
    assert(je2_client_switch(&c, .04, wire, sizeof(wire)) > 0);
    struct json_object *o = snapshot(&c, 3, 2, 1, JE2_PC, MK2_WAIT_FIRST), *f = NULL, *q = NULL;
    assert(json_object_object_get_ex(o, "snapshot_feedback", &f));
    assert(json_object_object_get_ex(f, "Joint", &q));
    assert(json_object_array_put_idx(q, 0, json_object_new_double(.125)) == 0);
    assert(observe(&c, o, .05) == 0 && c.phase == JE2_CLIENT_HOLD_READY);
    /* An older pre-Switch pose must not overwrite the post-ACK hold source. */
    assert(observe(&c, snapshot(&c, 2, 2, 0, JE2_AVAILABLE, MK2_READY), .055) == 1);
    assert(c.joint[0] == .125 && c.received_at == .05);
    assert(je2_client_hold(&c, .06, .01, wire, sizeof(wire)) > 0);
    const char *body = strstr(wire, "Joint "); assert(body);
    o = json_tokener_parse(body + 6); assert(o);
    assert(json_object_object_get_ex(o, "Robot0", &f) && json_object_object_get_ex(f, "joint", &q));
    assert(json_object_get_double(json_object_array_get_idx(q, 0)) == .125); json_object_put(o);
    assert(c.first_target[0] == .125);
    assert(observe(&c, snapshot(&c, 4, 2, 2, JE2_PC, MK2_ACTIVE), .07) == 0);
    assert(c.joint[0] == 0 && c.first_target[0] == .125);
    for (int stage = 0; stage < 3; ++stage) {
        if (stage == 0) {
            assert(je2_client_init(&c) == 0);
            assert(observe(&c, snapshot(&c, 1, 1, 0, JE2_AVAILABLE, MK2_READY), .01) == 0);
            assert(je2_client_claim(&c, .02, wire, sizeof(wire)) > 0);
        } else if (stage == 1) bind(&c); else activated(&c);
        o = snapshot(&c, 20, 2, stage == 2 ? 2 : 0, stage == 2 ? JE2_PC : JE2_AVAILABLE,
                     stage == 2 ? MK2_ACTIVE : MK2_READY);
        json_object_object_add(o, "session_boot", json_object_new_string("11111111111111111111111111111111"));
        assert(observe(&c, o, .1) == -1 && c.phase == JE2_CLIENT_RETIRED);
        assert(observe(&c, snapshot(&c, 21, 2, 1, JE2_PC, MK2_WAIT_FIRST), .11) == -1);
        assert(je2_client_switch(&c, .12, wire, sizeof(wire)) == -1 && !wire[0]);
        assert(je2_client_hold(&c, .13, .01, wire, sizeof(wire)) == -1 && !wire[0]);
    }
    assert(je2_client_init(&c) == 0);
    assert(observe(&c, snapshot(&c, 1, 1, 0, JE2_AVAILABLE, MK2_READY), .01) == 0);
    assert(je2_client_claim(&c, .02, wire, sizeof(wire)) > 0);
    o = snapshot(&c, 2, 2, 0, JE2_AVAILABLE, MK2_READY);
    json_object_object_add(o, "session_client", json_object_new_string("22222222222222222222222222222222"));
    assert(observe(&c, o, .03) == -1 && c.phase == JE2_CLIENT_RETIRED);
    puts("PASS: first hold uses changed post-Switch pose; restart at claim/bound/active and foreign claim ACK cannot re-enter");
}
static void streaming_targets(void)
{
    struct je2_client c; struct je2_client_lease lease; activated(&c);
    assert(je2_client_get_lease(&c, &lease) == 0);
    double q[MK2_AXES] = {.001}; char wire[4096];
    assert(je2_client_target(&c, &lease, .09, .04, q, wire, sizeof(wire)) > 0);
    assert(c.wire_sequence == 3 && c.acknowledged == 2 && c.last_data_time == .04);
    assert(strstr(wire, " 3 Joint "));
    double deadline = c.deadline;
    q[0] = .002;
    assert(je2_client_target(&c, &lease, .10, .05, q, wire, sizeof(wire)) == 0 && !wire[0]);
    assert(c.wire_sequence == 3 && c.last_data_time == .04 && c.deadline == deadline);
    assert(observe(&c, snapshot(&c, 5, 2, 3, JE2_PC, MK2_ACTIVE), .105) == 0);
    /* Fresh callback supplies the new goal, not the backpressured one. */
    q[0] = .003;
    assert(je2_client_target(&c, &lease, .11, .06, q, wire, sizeof(wire)) > 0);
    assert(c.wire_sequence == 4 && strstr(wire, " 4 Joint "));
    struct json_object *o = json_tokener_parse(strstr(wire, "Joint ") + 6), *robot = NULL, *joints = NULL;
    assert(o && json_object_object_get_ex(o, "Robot0", &robot));
    assert(json_object_object_get_ex(robot, "joint", &joints));
    assert(json_object_get_double(json_object_array_get_idx(joints, 0)) == .003); json_object_put(o);
    assert(observe(&c, snapshot(&c, 6, 2, 4, JE2_PC, MK2_ACTIVE), .12) == 0);
    assert(c.phase == JE2_CLIENT_ACTIVE && c.acknowledged == 4);
    assert(observe(&c, snapshot(&c, 7, 3, 4, JE2_YIELDING, MK2_RELEASING), .13) == -1);
    assert(c.phase == JE2_CLIENT_RETIRED);
    assert(je2_client_target(&c, &lease, .14, .09, q, wire, sizeof(wire)) == -1 && !wire[0]);
    assert(je2_client_get_lease(&c, &lease) == -1 && lease.generation == 0 && !lease.boot[0]);
    puts("PASS: consecutive target ACKs, one outstanding frame, no queued backpressure target, yield closes stream");
}
static void stale_target_leases(void)
{
    struct je2_client c; struct je2_client_lease lease; activated(&c);
    assert(je2_client_get_lease(&c, &lease) == 0);
    char wire[4096];
    for (int i = 0; i < 3; ++i) {
        struct je2_client_lease old = lease;
        if (i == 0) old.boot[0] = old.boot[0] == 'a' ? 'b' : 'a';
        if (i == 1) old.client[0] = old.client[0] == 'a' ? 'b' : 'a';
        if (i == 2) --old.generation;
        struct je2_client before = c;
        assert(je2_client_target(&c, &old, NAN, NAN, NULL, wire, sizeof(wire)) == -1 && !wire[0]);
        assert(!memcmp(&c, &before, sizeof(c)));
    }
    struct je2_client replacement; activated(&replacement);
    assert(strcmp(replacement.client, c.client)); /* Per-init random correlation ID. */
    struct je2_client before = replacement;
    assert(je2_client_target(&replacement, &lease, NAN, NAN, NULL, wire, sizeof(wire)) == -1 && !wire[0]);
    assert(!memcmp(&replacement, &before, sizeof(replacement)));
    puts("PASS: boot/client/generation stale callbacks rejected before bad clock/data, new client remains unchanged");
}
static void stream_faults(void)
{
    for (int i = 0; i < 11; ++i) {
        struct je2_client c; struct je2_client_lease lease; activated(&c);
        assert(je2_client_get_lease(&c, &lease) == 0);
        double q[MK2_AXES] = {0}, data_time = .04, t = .09; char wire[4096] = "old";
        switch (i) {
        case 0: data_time = NAN; break;
        case 1: data_time = .01; break; /* repeated cumulative time */
        case 2: data_time = .1; break; /* excessive per-frame interval */
        case 3: data_time = .001; break;
        case 4: q[0] = NAN; break;
        case 5: q[6] = INFINITY; break;
        case 6: t = .01; break; /* backwards clock */
        case 7: c.wire_sequence = c.acknowledged = UINT64_MAX; break;
        case 8: break; /* tiny output buffer */
        case 9: t = .16; data_time = .02; break; /* old timestamp, fresh feedback */
        default: break; /* null target */
        }
        uint64_t sequence = c.wire_sequence;
        assert(je2_client_target(&c, &lease, t, data_time, i == 10 ? NULL : q,
                                 wire, i == 8 ? 2 : sizeof(wire)) == -1 && !wire[0]);
        assert(c.phase == JE2_CLIENT_FAULT && c.wire_sequence == sequence);
    }
    struct je2_client c; struct je2_client_lease lease; activated(&c);
    assert(je2_client_get_lease(&c, &lease) == 0);
    double q[MK2_AXES] = {0}; char wire[4096];
    assert(je2_client_target(&c, &lease, .09, .04, q, wire, sizeof(wire)) > 0);
    double deadline = c.deadline;
    assert(observe(&c, snapshot(&c, 5, 2, 2, JE2_PC, MK2_ACTIVE), .15) == 0);
    assert(observe(&c, snapshot(&c, 6, 2, 2, JE2_PC, MK2_ACTIVE), .18) == 0);
    assert(c.deadline == deadline && c.acknowledged == 2);
    assert(je2_client_tick(&c, .191) == -1 && c.phase == JE2_CLIENT_FAULT);
    assert(observe(&c, snapshot(&c, 7, 2, 3, JE2_PC, MK2_ACTIVE), .192) == -1);
    assert(je2_client_target(&c, &lease, .20, .15, q, wire, sizeof(wire)) == -1 && !wire[0]);
    for (int i = 0; i < 3; ++i) {
        activated(&c); assert(je2_client_get_lease(&c, &lease) == 0);
        assert(je2_client_target(&c, &lease, .09, .04, q, wire, sizeof(wire)) > 0);
        if (i == 0) je2_client_send_failed(&c);
        else {
            assert(observe(&c, snapshot(&c, 5, 2, 3, JE2_PC, MK2_ACTIVE), .10) == 0);
            assert(observe(&c, snapshot(&c, 6, 2, i == 1 ? 4 : 2, JE2_PC, MK2_ACTIVE), .11) == -1);
        }
        assert(c.phase == JE2_CLIENT_FAULT);
    }
    puts("PASS: 11 bad stream inputs, ACK stalls despite healthy snapshots, late ACK/send failure/future or regressive ACK latch");
}
static struct json_object *release_snapshot(const struct je2_client *c, uint64_t sequence, int phase)
{
    struct json_object *o = snapshot(c, sequence, 3, 3,
        phase == 2 ? JE2_PENDANT : JE2_EXIT_BARRIER, MK2_READY);
    json_object_object_add(o, "session_generation", json_object_new_uint64(2));
    json_object_object_add(o, "session_release_sequence", json_object_new_uint64(3));
    json_object_object_add(o, "session_release_phase", json_object_new_int(phase));
    return o;
}
static void close_receipts(void)
{
    struct je2_client c; struct je2_client_lease lease; char wire[MK2_FRAME_MAX];
    activated(&c); assert(je2_client_get_lease(&c, &lease) == 0);
    struct json_object *legacy = snapshot(&c, 5, 2, 2, JE2_PC, MK2_ACTIVE);
    json_object_object_del(legacy, "session_release_sequence");
    json_object_object_del(legacy, "session_release_phase");
    assert(observe(&c, legacy, .085) == 0 && !c.release_supported);
    assert(je2_client_close(&c, &lease, .09, wire, sizeof(wire)) == -1 && !wire[0]);
    assert(c.phase == JE2_CLIENT_FAULT && c.wire_sequence == 2);
    activated(&c); assert(je2_client_get_lease(&c, &lease) == 0);
    for (int i = 0; i < 3; ++i) {
        struct je2_client_lease old = lease;
        if (i == 0) ++old.generation;
        else if (i == 1) old.boot[0] = '0';
        else old.client[0] = old.client[0] == '0' ? '1' : '0';
        const struct je2_client before = c;
        assert(je2_client_close(&c, &old, NAN, NULL, 0) == -1);
        assert(!memcmp(&before, &c, sizeof(c)));
    }
    assert(je2_client_close(&c, &lease, .09, wire, sizeof(wire)) > 0);
    assert(strstr(wire, "Release {\"Release\":true}") && c.phase == JE2_CLIENT_CLOSING);
    assert(observe(&c, release_snapshot(&c, 5, 1), .1) == 0 && c.phase == JE2_CLIENT_CLOSING);
    assert(je2_client_close(&c, &lease, .11, wire, sizeof(wire)) == 0 && !wire[0]);
    assert(observe(&c, release_snapshot(&c, 5, 2), .12) == 1 && c.phase == JE2_CLIENT_CLOSING);
    assert(observe(&c, release_snapshot(&c, 6, 2), .2) == 0 && c.phase == JE2_CLIENT_CLOSED);
    assert(je2_client_close(&c, &lease, .21, wire, sizeof(wire)) == 0 && !wire[0]);
    assert(je2_client_target(&c, &lease, .22, .02, c.joint, wire, sizeof(wire)) == -1 && !wire[0]);
    for (int kind = 0; kind < 14; ++kind) {
        activated(&c); assert(je2_client_get_lease(&c, &lease) == 0);
        assert(je2_client_close(&c, &lease, .09, wire, sizeof(wire)) > 0);
        struct json_object *o = release_snapshot(&c, 5, 2), *f = NULL;
        assert(json_object_object_get_ex(o, "snapshot_feedback", &f));
        switch (kind) {
        case 0: json_object_object_del(o, "session_release_phase"); break;
        case 1: json_object_object_add(o, "session_release_phase", json_object_new_string("2")); break;
        case 2: json_object_object_add(o, "session_release_phase", json_object_new_double(2)); break;
        case 3: json_object_object_add(o, "session_release_phase", json_object_new_int(3)); break;
        case 4: json_object_object_del(o, "session_release_sequence"); break;
        case 5: json_object_object_add(o, "session_release_sequence", json_object_new_double(3)); break;
        case 6: json_object_object_add(o, "session_release_sequence", json_object_new_int(2)); break;
        case 7: json_object_object_add(o, "generation", json_object_new_int(4)); break;
        case 8: json_object_object_add(o, "session_generation", json_object_new_int(3)); break;
        case 9: json_object_object_add(o, "session_client", json_object_new_string("00000000000000000000000000000000")); break;
        case 10: json_object_object_add(o, "owner", json_object_new_int(JE2_YIELDING)); break;
        case 11: json_object_object_add(f, "MoveState", json_object_new_int(7)); break;
        case 12: json_object_object_add(o, "session_release_phase", json_object_new_int(0)); break;
        default:
            json_object_object_del(o, "session_release_phase");
            json_object_object_add(f, "session_release_phase", json_object_new_int(2)); break;
        }
        assert(observe(&c, o, .1) == -1);
        assert(c.phase == JE2_CLIENT_FAULT || c.phase == JE2_CLIENT_RETIRED);
        assert(je2_client_close(&c, &lease, .11, wire, sizeof(wire)) == -1 && !wire[0]);
    }
    for (int kind = 0; kind < 3; ++kind) {
        activated(&c); assert(je2_client_get_lease(&c, &lease) == 0);
        assert(je2_client_close(&c, &lease, .09, wire, sizeof(wire)) > 0);
        struct json_object *o = release_snapshot(&c, 5, 2);
        if (kind) json_object_object_del(o, "session_release_phase");
        const char *body = json_object_to_json_string_ext(o, JSON_C_TO_STRING_PLAIN);
        int n = snprintf(wire, sizeof(wire), "Owner {\"session_release_phase\":%s,%s",
            kind == 0 ? "2" : kind == 1 ? "18446744073709551616" : "2e0", body + 1);
        assert(n > 0 && (size_t)n < sizeof(wire));
        assert(je2_client_observe(&c, wire, (size_t)n, .1) == -1 && c.phase == JE2_CLIENT_FAULT);
        json_object_put(o);
    }
    puts("PASS: close receipt identity/schema/duplicates/overflow, stale close leases, terminal CLOSED, no replay");
}
int main(void)
{
    lifecycle_and_retirement(); deadlines_and_send_failure(); bad_ack_and_feedback(); malformed_and_duplicate_keys();
    structured_numeric_fields(); fresh_hold_and_stale_claim();
    streaming_targets(); stale_target_leases(); stream_faults();
    close_receipts();
    puts("PASS: standalone client component only; no ROS activation, network, SDK or deployment");
    return 0;
}