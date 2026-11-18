/* Runtime helper tests: fake SDK ONLY. Full ZMQ/PluginMain coverage is separate. */
static const char *client_a = "11111111111111111111111111111111";
static const char *client_b = "22222222222222222222222222222222";
static int session_packet(struct je2_session *s, bool claim, const char *boot,
                          uint64_t generation, const char *client, uint64_t seq,
                          const char *payload, bool more)
{
    char text[MK2_FRAME_MAX + 128];
    int n = claim ? snprintf(text, sizeof(text), "JE2C1 %s %llu %s", boot,
        (unsigned long long)generation, client) :
        snprintf(text, sizeof(text), "JE2F1 %s %llu %s %llu %s", boot,
        (unsigned long long)generation, client, (unsigned long long)seq, payload);
    assert(n > 0 && n < (int)sizeof(text));
    return je2_session_receive(&engine, s, text, (size_t)n, more);
}
static void session_claim(struct je2_session *s, const char *client)
{
    uint64_t observed = engine.generation;
    assert(session_packet(s, true, s->boot, observed, client, 0, NULL, false) == -1);
    wait_until(monotonic_time() + .11);
    assert(session_packet(s, true, s->boot, observed, client, 0, NULL, false) == 0);
    assert(engine.generation == observed + 1 && !engine.legacy_open);
    assert(s->generation == engine.generation && !strcmp(s->client, client) && !s->last_sequence);
    assert(!fake.mutations); /* Neither claim nor ACK starts a servo. */
}
static void session_start(struct je2_session *s)
{
    initialize(); assert(je2_session_init(s) == 0); session_claim(s, client_a);
    assert(session_packet(s, false, s->boot, s->generation, client_a, 1,
        "Switch {\"Switch\":true}", false) == 0);
    assert(!fake.mutations);
    assert(session_packet(s, false, s->boot, s->generation, client_a, 2,
        "Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0,0,0,0,0,0,0]}}", false) == 0);
    assert(fake.starts == 1 && fake.joints == 1);
}
static void session_envelope_guards(void)
{
    struct je2_session s, other;
    initialize(); assert(je2_session_init(&s) == 0 && je2_session_init(&other) == 0);
    assert(strlen(s.boot) == 32 && strcmp(s.boot, other.boot));
    char text[300];
    const char *invalid[] = {"Switch {\"Switch\":true}", "JE2C1", "JE2C1 ",
        "JE2C2 ", " JE2C1 ", "JE2F1 \n", "Owner {}"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
        assert(je2_session_receive(&engine, &s, invalid[i], strlen(invalid[i]), false) == -1);
    const char *numbers[] = {"0", "01", "-1", "+1", "1.0", "1e0", "18446744073709551616"};
    for (size_t i = 0; i < sizeof(numbers) / sizeof(numbers[0]); ++i) {
        int n = snprintf(text, sizeof(text), "JE2C1 %s %s %s", s.boot, numbers[i], client_a);
        assert(je2_session_receive(&engine, &s, text, (size_t)n, false) == -1);
    }
    int n = snprintf(text, sizeof(text), "JE2C1 %s 1 %s\n", s.boot, client_a);
    assert(je2_session_receive(&engine, &s, text, (size_t)n, false) == -1);
    text[n - 1] = '\0'; /* Even an embedded NUL in a length-delimited message. */
    assert(je2_session_receive(&engine, &s, text, (size_t)n, false) == -1);
    assert(session_packet(&s, true, other.boot, 1, client_a, 0, NULL, false) == -1);
    assert(session_packet(&s, true, s.boot, UINT64_MAX, client_a, 0, NULL, false) == -1);
    assert(session_packet(&s, false, s.boot, 1, client_a, 1, "Switch {}", false) == -1);
    char oversized[MK2_FRAME_MAX + 1]; memset(oversized, 'x', sizeof(oversized));
    assert(je2_session_receive(&engine, &s, oversized, sizeof(oversized), false) == -1);
    assert(!fake.getters && !fake.mutations && engine.generation == 1);
    puts("PASS: canonical session envelope rejects legacy/overflow/ambiguous/stale/unclaimed packets before SDK reads");
}
static void session_replay_and_gap(void)
{
    struct je2_session s, restarted;
    session_start(&s);
    unsigned mutations = fake.mutations, getters = fake.getters;
    const char *stop = "Switch {\"Switch\":false}";
    assert(session_packet(&s, false, s.boot, s.generation, client_b, 3, stop, false) == -1);
    assert(session_packet(&s, false, s.boot, s.generation - 1, client_a, 3, stop, false) == -1);
    assert(session_packet(&s, false, s.boot, s.generation, client_a, 2, stop, false) == -1);
    assert(session_packet(&s, true, s.boot, s.generation - 1, client_a, 0, NULL, false) == -1);
    assert(session_packet(&s, false, s.boot, s.generation, client_a, 3, stop, true) == -1);
    assert(session_packet(&s, false, s.boot, s.generation, client_a, 3, stop, false) == -1);
    assert(!s.discarding_parts && s.last_sequence == 2);
    assert(fake.mutations == mutations && fake.getters == getters && engine.owner == JE2_PC);
    /* The next complete message reaches the core after the final part.
     * A duplicate enable is rejected by the core, not a sticky drain latch. */
    assert(session_packet(&s, false, s.boot, s.generation, client_a, 3,
        "Switch {\"Switch\":true}", false) == -1);
    assert(s.last_sequence == 3 && fake.getters > getters && fake.mutations == mutations);
    assert(session_packet(&s, false, s.boot, s.generation, client_a, 5, stop, false) == -1);
    assert(engine.owner == JE2_FAULT && engine.core.fault == MK2_BAD_FRAME);
    assert(fake.stops == 1 && fake.ends == 1);
    je2_runtime_shutdown(&engine);
    assert(fake.stops == 1 && fake.ends == 1);

    /* Restart reuses low engine generations, but never the old boot identity. */
    char old_boot[33]; memcpy(old_boot, s.boot, sizeof(old_boot));
    initialize(); assert(je2_session_init(&restarted) == 0);
    assert(strcmp(old_boot, restarted.boot));
    assert(session_packet(&restarted, true, old_boot, 1, client_a, 0, NULL, false) == -1);
    assert(!fake.getters && !fake.mutations);
    session_claim(&restarted, client_b);
    getters = fake.getters;
    assert(session_packet(&restarted, false, old_boot, restarted.generation, client_a, 1, stop, false) == -1);
    assert(fake.getters == getters && !fake.mutations && !restarted.last_sequence);
    puts("PASS: boot/client/generation/replay/multipart fences; current-session sequence gap latches once");
}
static void session_no_recovery_or_stale_hold(void)
{
    struct je2_session s;
    initialize(); assert(je2_session_init(&s) == 0); session_claim(&s, client_a);
    assert(session_packet(&s, false, s.boot, s.generation, client_a, 1,
        "Switch {\"Switch\":true}", false) == 0);
    assert(session_packet(&s, false, s.boot, s.generation, client_a, 2,
        "Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0.01,0,0,0,0,0,0]}}", false) == -1);
    assert(engine.owner == JE2_FAULT && engine.core.fault == MK2_FIRST_TARGET && !fake.starts && !fake.joints);
    assert(session_packet(&s, true, s.boot, engine.generation, client_b, 0, NULL, false) == -1);

    initialize(); assert(je2_session_init(&s) == 0); session_claim(&s, client_a);
    fake.state = 7;
    assert(session_packet(&s, true, s.boot, engine.generation, client_b, 0, NULL, false) == -1);
    assert(engine.owner == JE2_FAULT && engine.core.fault == MK2_STATE_ERROR);
    fake.state = 0;
    assert(session_packet(&s, true, s.boot, engine.generation, client_b, 0, NULL, false) == -1);
    assert(!fake.mutations && !s.last_sequence);
    puts("PASS: session claim cannot restore state7/fault, execute without fresh hold or replay a stale first target");
}
static void session_tests(void)
{
    session_envelope_guards(); session_replay_and_gap(); session_no_recovery_or_stale_hold();
}