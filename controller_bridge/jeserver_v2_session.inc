/* Included only in the explicit private simulation session experiment.
 * Fixed ASCII envelope avoids ambiguous JSON metadata/duplicate keys:
 * JE2C1 <boot32hex> <observed-generation> <client32hex>
 * JE2F1 <boot32hex> <claimed-generation> <client32hex> <sequence> <core-frame>
 * No legacy fallback in this build; unknown/malformed envelopes are dropped
 * before SDK reads or engine callbacks. Tokens are public correlation IDs. */
#include <sys/random.h>

static int je2_session_init(struct je2_session *s)
{
    unsigned char bytes[16];
    memset(s, 0, sizeof(*s));
    if (getrandom(bytes, sizeof(bytes), GRND_NONBLOCK) != (ssize_t)sizeof(bytes)) return -1;
    static const char hex[] = "0123456789abcdef";
    for (size_t i = 0; i < sizeof(bytes); ++i) {
        s->boot[2 * i] = hex[bytes[i] >> 4];
        s->boot[2 * i + 1] = hex[bytes[i] & 15];
    }
    return 0; /* Never derive identity from time, PID or an environment override. */
}
static bool je2_session_hex(const char **p, const char *end, char out[33])
{
    if (end - *p < 32) return false;
    for (int i = 0; i < 32; ++i) {
        char c = (*p)[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    memcpy(out, *p, 32); out[32] = '\0'; *p += 32;
    return true;
}
static bool je2_session_space(const char **p, const char *end)
{
    if (*p == end || **p != ' ') return false;
    ++*p; return true;
}
static bool je2_session_uint(const char **p, const char *end, uint64_t *out)
{
    if (*p == end || **p < '1' || **p > '9') return false;
    uint64_t n = 0;
    while (*p < end && **p >= '0' && **p <= '9') {
        unsigned digit = (unsigned)(**p - '0');
        if (n > (UINT64_MAX - digit) / 10) return false;
        n = n * 10 + digit; ++*p;
    }
    *out = n; return true;
}
static int je2_session_drop(struct jeserver_v2 *v)
{
    if (v->dropped_pc != UINT64_MAX) ++v->dropped_pc;
    return -1;
}
static int je2_session_receive(struct jeserver_v2 *v, struct je2_session *s,
                               const char *text, size_t length, bool more)
{
    /* Discard multipart one part per executor turn: no unbounded drain, no
     * old multipart cleanup against the new owner, no tail-as-new-message. */
    if (s->discarding_parts || more) {
        s->discarding_parts = more;
        return je2_session_drop(v);
    }
    if (!s->boot[0] || !text || length < 6 || length > MK2_FRAME_MAX || memchr(text, '\0', length))
        return je2_session_drop(v);
    bool claim = !memcmp(text, "JE2C1 ", 6);
    if (!claim && memcmp(text, "JE2F1 ", 6)) return je2_session_drop(v);
    const char *p = text + 6, *end = text + length;
    char boot[33], client[33];
    uint64_t generation, sequence = 0;
    if (!je2_session_hex(&p, end, boot) || !je2_session_space(&p, end) ||
        !je2_session_uint(&p, end, &generation) || !je2_session_space(&p, end) ||
        !je2_session_hex(&p, end, client)) return je2_session_drop(v);
    if (claim ? p != end : (!je2_session_space(&p, end) ||
        !je2_session_uint(&p, end, &sequence) || !je2_session_space(&p, end) || p == end))
        return je2_session_drop(v);
    if (strcmp(boot, s->boot) || generation != v->generation) return je2_session_drop(v);
    if (claim) {
        if ((v->owner != JE2_AVAILABLE && v->owner != JE2_PENDANT) || !strcmp(client, s->client))
            return je2_session_drop(v);
        struct je2_feedback f = je2_read_feedback();
        int result = je2_claim_pc(v, generation, monotonic_time(), &f);
        if (!result) {
            memcpy(s->client, client, sizeof(s->client));
            s->generation = v->generation; s->last_sequence = 0; s->release_sequence = 0;
        }
        return result; /* Owner boot/client/generation is the asynchronous ACK. */
    }
    if (generation != s->generation || strcmp(client, s->client) ||
        (v->owner != JE2_AVAILABLE && v->owner != JE2_PC) || sequence <= s->last_sequence)
        return je2_session_drop(v);
    /* A current session with a missing command must not resume across a gap.
     * Old/replayed envelopes above never enter this cleanup path. */
    if (s->last_sequence == UINT64_MAX || sequence != s->last_sequence + 1) {
        je2_abort(v, MK2_BAD_FRAME);
        return -1;
    }
    s->last_sequence = sequence;
    struct je2_feedback f = je2_read_feedback();
    static const char release[] = "Release {\"Release\":true}";
    const bool closing = v->core.phase == MK2_ACTIVE && v->core.owns_servo &&
        (size_t)(end - p) == sizeof(release) - 1 && !memcmp(p, release, sizeof(release) - 1);
    const int result = je2_pc_frame(v, generation, p, (size_t)(end - p), monotonic_time(), &f);
    if (!result && closing && generation != UINT64_MAX && v->generation == generation + 1 &&
        v->owner == JE2_YIELDING && v->core.phase == MK2_RELEASING && !v->core.fault && !v->core.sdk_error)
        s->release_sequence = sequence;
    return result;
}