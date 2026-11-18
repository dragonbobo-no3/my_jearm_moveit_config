/* Real ZMQ, real PluginMain and kernel writer tasks; FAKE SDK, never hardware. */
static const char *sid_a = "11111111111111111111111111111111";
static const char *sid_b = "22222222222222222222222222222222";
static const char *hold = "Joint {\"Robot0\":{\"time\":0.01,\"joint\":[0,0,0,0,0,0,0]}}";
static void session_send(struct session *s, const char *boot, uint64_t generation,
                         const char *client, uint64_t seq, const char *payload, int flags)
{
    char text[1024];
    int n = payload ? snprintf(text, sizeof(text), "JE2F1 %s %llu %s %llu %s", boot,
        (unsigned long long)generation, client, (unsigned long long)seq, payload) :
        snprintf(text, sizeof(text), "JE2C1 %s %llu %s", boot, (unsigned long long)generation, client);
    assert(n > 0 && n < (int)sizeof(text));
    assert(zmq_send(s->pub, text, (size_t)n, flags) == n);
}
static void session_claim_ack(struct session *s, const char *client)
{
    uint64_t observed = s->generation;
    unsigned mutations = fake.mutations;
    double deadline = now() + 1, next = 0;
    /* Bounded TEST retries cover slow PUB joins and the fresh idle interval.
     * No client reconnect/retry policy is being installed in production. */
    do {
        if (now() >= next) {
            session_send(s, s->boot, observed, client, 0, NULL, 0); next = now() + .02;
        }
        receive(s);
    } while (now() < deadline && !(s->generation == observed + 1 &&
        s->session_generation == s->generation && !strcmp(s->client, client)));
    assert(s->generation == observed + 1 && s->session_generation == s->generation);
    assert(!strcmp(s->client, client) && s->last_sequence == 0 && s->owner == JE2_AVAILABLE);
    assert(fake.mutations == mutations);
}
static void session_switch_ack(struct session *s, const char *client)
{
    double deadline = now() + .5, next = 0;
    do {
        if (now() >= next) {
            session_send(s, s->boot, s->session_generation, client, 1, "Switch {\"Switch\":true}", 0);
            next = now() + .02;
        }
        receive(s);
    } while (now() < deadline && !(s->last_sequence == 1 && s->owner == JE2_PC));
    assert(s->last_sequence == 1 && s->owner == JE2_PC);
}
static void session_wait_drops(struct session *s, uint64_t target)
{
    double deadline = now() + .12;
    while (now() < deadline && s->dropped < target) receive(s);
    assert(s->dropped >= target);
}
static void session_client_case(const char *library, int reject_case)
{
    struct session s; open_session(&s, library);
    struct je2_client client; assert(je2_client_init(&client) == 0); s.pc_client = &client;
    char wire[MK2_FRAME_MAX];
    double deadline = now() + 1;
    while (now() < deadline && !client.seen) receive(&s);
    assert(client.seen && client.phase == JE2_CLIENT_OBSERVING);
    /* The real reusable client consumes the server's single identity+sample
     * snapshot. No fabricated claim ACK or rewritten native guard fields. */
    deadline = now() + .5;
    do {
        int n = je2_client_claim(&client, now(), wire, sizeof(wire));
        assert(n >= 0);
        if (n > 0) assert(zmq_send(s.pub, wire, (size_t)n, 0) == n);
        receive(&s);
    } while (now() < deadline && client.phase == JE2_CLIENT_CLAIMING);
    assert(client.phase == JE2_CLIENT_BOUND && fake.mutations == 0);
    int n = je2_client_switch(&client, now(), wire, sizeof(wire));
    assert(n > 0 && zmq_send(s.pub, wire, (size_t)n, 0) == n);
    deadline = now() + .15;
    while (now() < deadline && client.phase == JE2_CLIENT_SWITCHING) receive(&s);
    assert(client.phase == JE2_CLIENT_HOLD_READY && !fake.starts);
    n = je2_client_hold(&client, now(), .01, wire, sizeof(wire));
    assert(n > 0 && zmq_send(s.pub, wire, (size_t)n, 0) == n);
    deadline = now() + .15;
    while (now() < deadline && client.phase == JE2_CLIENT_HOLDING) receive(&s);
    assert(client.phase == JE2_CLIENT_ACTIVE && fake.starts == 1 && fake.joints == 1);
    struct je2_client_lease lease; assert(je2_client_get_lease(&client, &lease) == 0);
    double target[MK2_AXES] = {0};
    for (unsigned i = 0; i < (reject_case ? 1u : 25u); ++i) {
        /* Only the fake test generates this tiny analytic trajectory. The
         * protocol component neither generates nor retimes caller targets. */
        deadline = now() + .1;
        while (now() < deadline && now() - client.first_target_at + client.first_data_time - client.last_data_time < .008)
            receive(&s);
        double sent = now(), elapsed = sent - client.first_target_at;
        double data_time = client.first_data_time + elapsed;
        double dt = data_time - client.last_data_time;
        target[0] = .0001 * elapsed * elapsed;
        if (reject_case == 1) target[0] = 2;       /* position limit */
        if (reject_case == 2) target[0] = .03;     /* following error */
        if (reject_case == 3) target[0] = .3 * dt; /* velocity */
        if (reject_case == 4) target[0] = .1 * dt; /* acceleration */
        n = je2_client_target(&client, &lease, sent, data_time, target, wire, sizeof(wire));
        assert(n > 0);
        /* Case 5 deliberately loses the formatted command, as a PUB transport
         * can silently do. Real healthy Owner publications continue unchanged. */
        if (reject_case != 5) assert(zmq_send(s.pub, wire, (size_t)n, 0) == n);
        deadline = now() + (reject_case == 5 ? .14 : .1);
        while (now() < deadline && client.phase == JE2_CLIENT_ACTIVE && client.acknowledged != client.wire_sequence)
            receive(&s);
        if (reject_case == 5) {
            assert(client.phase == JE2_CLIENT_FAULT && client.acknowledged == 2 && client.wire_sequence == 3);
            assert(s.owner == JE2_PC && !s.snapshot_fault && fake.joints == 1 && !fake.stops);
            assert(je2_client_target(&client, &lease, now(), data_time + .01, target, wire, sizeof(wire)) == -1 && !wire[0]);
            close_session(&s, library);
            puts("PASS: lost outgoing target times out locally despite real healthy Owner stream, no retransmission");
            return;
        }
        if (reject_case) {
            const int faults[] = {0, MK2_LIMIT, MK2_FOLLOWING, MK2_VELOCITY, MK2_ACCELERATION};
            assert(client.phase == JE2_CLIENT_FAULT && s.snapshot_fault == faults[reject_case]);
            assert(fake.joints == 1 && fake.starts == 1 && fake.stops == 1 && fake.ends == 1);
            assert(je2_client_target(&client, &lease, now(), data_time + .01, target, wire, sizeof(wire)) == -1 && !wire[0]);
            close_session(&s, library);
            puts("PASS: client envelope preserves server position/following/velocity/acceleration rejection before SDK target call");
            return;
        }
        assert(client.phase == JE2_CLIENT_ACTIVE && client.acknowledged == client.wire_sequence);
        assert(fake.joints == i + 2 && fake.starts == 1 && !fake.stops);
    }
    char last_wire[MK2_FRAME_MAX]; memcpy(last_wire, wire, (size_t)n + 1);
    int last_size = n;
    int fd = button(); send_button(fd, "tv49!");
    deadline = now() + .15;
    while (now() < deadline && client.phase == JE2_CLIENT_ACTIVE) receive(&s);
    assert(client.phase == JE2_CLIENT_RETIRED);
    assert(je2_client_claim(&client, now(), wire, sizeof(wire)) == -1 && !wire[0]);
    assert(je2_client_hold(&client, now(), .01, wire, sizeof(wire)) == -1 && !wire[0]);
    assert(je2_client_target(&client, &lease, now(), client.last_data_time + .01, target, wire, sizeof(wire)) == -1 && !wire[0]);
    await_owner(&s, JE2_PENDANT); await_counter(&s, &fake.jogs, 1);
    uint64_t dropped = s.dropped;
    assert(zmq_send(s.pub, last_wire, (size_t)last_size, 0) == last_size);
    session_wait_drops(&s, dropped + 1); /* A delayed already-formatted frame is also fenced at server. */
    assert(fake.joints == 26 && fake.ends == 1 && !fake.stops);
    close_session(&s, library); close_after_unload(fd);
    puts("PASS: client + real ZMQ first hold and 25 timed targets, native yield closes stream and late serialized frame is fenced");
}
static void session_cases(const char *library)
{
    struct session s; open_session(&s, library);
    char old_boot[33]; memcpy(old_boot, s.boot, sizeof(old_boot));
    session_claim_ack(&s, sid_a); session_switch_ack(&s, sid_a);
    assert(!fake.starts && !fake.joints);
    uint64_t old_generation = s.session_generation;
    session_send(&s, s.boot, old_generation, sid_a, 2, hold, 0);
    await_counter(&s, &fake.joints, 1);
    int fd = button(); send_button(fd, "tv49!");
    await_owner(&s, JE2_YIELDING); await_owner(&s, JE2_PENDANT);
    await_counter(&s, &fake.jogs, 1);
    assert(fake.starts == 1 && fake.ends == 1 && fake_writer.alive == 0);
    send_button(fd, "tv33!"); await_counter(&s, &fake.jog_stops, 1);
    session_claim_ack(&s, sid_b);
    assert(s.generation == old_generation + 2 && !fake.stops && fake.joints == 1);
    session_switch_ack(&s, sid_b);
    uint64_t dropped = s.dropped;
    const char *late[] = {hold, "Switch {\"Switch\":false}", "Release {\"Release\":true}", "Joint broken"};
    for (size_t i = 0; i < 4; ++i)
        session_send(&s, s.boot, old_generation, sid_a, 3 + i, late[i], 0);
    assert(zmq_send(s.pub, late[1], strlen(late[1]), 0) >= 0);
    session_send(&s, s.boot, old_generation, sid_a, 10, late[1], ZMQ_SNDMORE);
    /* Even a valid NEW-session command in an old multipart tail is discarded. */
    session_send(&s, s.boot, s.session_generation, sid_b, 2, late[1], 0);
    session_wait_drops(&s, dropped + 7);
    assert(s.owner == JE2_PC && s.last_sequence == 1 && fake.starts == 1 && fake.joints == 1 && !fake.stops);
    /* A complete command AFTER the multipart tail must still be delivered.
     * Release cancels the unfulfilled new intent without a second Start. */
    session_send(&s, s.boot, s.session_generation, sid_b, 2, "Release {\"Release\":true}", 0);
    await_owner(&s, JE2_EXIT_BARRIER);
    assert(s.last_sequence == 2 && fake.starts == 1 && fake.joints == 1 && !fake.stops);
    /* No second ServoStart is attempted: writer adapter remains once-per-load.
     * Claim/Switch ACK alone is NOT a certified second-servo cycle. */
    close_session(&s, library); close_after_unload(fd);
    puts("PASS: full session PC->writer exit->pendant STOP0->new claim/Switch; stale goals/stop/release/multipart fenced");

    open_session(&s, library);
    assert(strcmp(s.boot, old_boot));
    session_claim_ack(&s, sid_a); session_switch_ack(&s, sid_a);
    dropped = s.dropped;
    session_send(&s, old_boot, s.session_generation, sid_a, 2, hold, 0);
    session_wait_drops(&s, dropped + 1);
    assert(!fake.mutations && s.last_sequence == 1);
    /* Same client and numeric generation after reload; only boot differs. */
    session_send(&s, s.boot, s.session_generation, sid_a, 2, hold, 0);
    await_counter(&s, &fake.joints, 1);
    dropped = s.dropped;
    session_send(&s, s.boot, s.session_generation, sid_a, 2, hold, 0);
    session_wait_drops(&s, dropped + 1);
    assert(fake.joints == 1 && !fake.stops);
    session_send(&s, s.boot, s.session_generation, sid_a, 4, hold, 0);
    await_owner(&s, JE2_FAULT);
    assert(fake.stops == 1 && fake.ends == 1 && fake.joints == 1);
    close_session(&s, library);
    puts("PASS: actual DSO reload changes boot token; duplicate target drops, missing current sequence faults once");

    open_session(&s, library); fake_writer.jog_stop_state = 7;
    fd = button(); send_button(fd, "tv49!");
    await_counter(&s, &fake.jogs, 1); await_owner(&s, JE2_PENDANT);
    send_button(fd, "tv33!"); await_owner(&s, JE2_FAULT);
    uint64_t generation = s.generation; dropped = s.dropped;
    session_send(&s, s.boot, generation, sid_b, 0, NULL, 0);
    session_wait_drops(&s, dropped + 1);
    assert(s.generation == generation && !s.client[0] && !fake.starts && !fake.joints);
    fake.state = 0; /* Fake later healthy feedback MUST NOT clear the latch. */
    dropped = s.dropped;
    session_send(&s, s.boot, generation, sid_b, 0, NULL, 0);
    session_wait_drops(&s, dropped + 1);
    assert(s.owner == JE2_FAULT && !s.client[0] && fake.jogs == 1 && fake.jog_stops == 1);
    close_session(&s, library); close_after_unload(fd);
    puts("PASS: full worker native STOP7 and later healthy feedback both refuse new-session claim, no recovery");
    for (int i = 0; i <= 5; ++i) session_client_case(library, i);
}