/* Shared by the plugin publisher and SDK-free protocol integration tests.
 * Formatting ONLY: no SDK, sockets, clocks, mutation or ownership inference.
 * Keep the non-session wire schema unchanged. */
static int je2_format_owner(char *out, size_t capacity, const struct jeserver_v2 *v,
                            const struct mk2_sample *sample, uint64_t sequence,
                            unsigned port, const struct je2_session *session,
                            bool writer_enabled, bool instance_exited, double exit_time)
{
    if (!out || !capacity) return -1;
    out[0] = '\0';
    char frame[1024];
    int n = snprintf(frame, sizeof(frame),
        "Owner {\"implementation\":\"JEServer_v2\",\"experimental\":true,"
        "\"owner\":%d,\"generation\":%llu,\"dropped_pc\":%llu,"
        "\"button_port\":%u,\"loopback_only\":true,"
        "\"automatic_priority_ready\":false,\"writer_exit_verified\":false,"
        "\"pc_reclaim_supported\":false,\"writer_observer_enabled\":%s,"
        "\"writer_instance_exited\":%s,\"writer_exit_time\":%.9f,"
        "\"session_transport_enabled\":%s,\"session_boot\":\"%s\",\"session_client\":\"%s\","
        "\"session_generation\":%llu,\"session_last_sequence\":%llu}", (int)v->owner,
        (unsigned long long)v->generation, (unsigned long long)v->dropped_pc, port,
        writer_enabled ? "true" : "false", instance_exited ? "true" : "false", exit_time,
        session ? "true" : "false", session ? session->boot : "", session ? session->client : "",
        (unsigned long long)(session ? session->generation : 0),
        (unsigned long long)(session ? session->last_sequence : 0));
    if (n <= 0 || (size_t)n >= sizeof(frame)) return -1;
#if JE2_SIM_SESSION_EXPERIMENT
    /* Same executor publication for identity and sequential feedback. Not an
     * atomic vendor snapshot or a certificate of stop origin. */
    if (session) {
        if (!sample) return -1;
        struct json_object *owner = json_tokener_parse(frame + 6);
        if (!owner) return -1;
        struct json_object *feedback = json_object_new_object();
        struct json_object *joints = json_object_new_array();
        if (!feedback || !joints) {
            if (feedback) json_object_put(feedback);
            if (joints) json_object_put(joints);
            json_object_put(owner); return -1;
        }
        json_object_object_add(owner, "snapshot_version", json_object_new_int(1));
        json_object_object_add(owner, "snapshot_sequence", json_object_new_uint64(sequence));
        json_object_object_add(owner, "snapshot_phase", json_object_new_int(v->core.phase));
        json_object_object_add(owner, "snapshot_fault", json_object_new_int(v->core.fault));
        json_object_object_add(owner, "snapshot_sdk_error", json_object_new_int(v->core.sdk_error));
        /* Request-correlated engine progress, not stop provenance, an atomic
         * SDK sample, or permission to start again. Phase 2 is possible only
         * AFTER the existing instance-exit and post-exit stationary barrier.
         * A later owner/generation/claim invalidates this live receipt. */
        int release_phase = 0;
        if (session->release_sequence && session->generation != UINT64_MAX &&
            v->generation == session->generation + 1 && !v->core.fault && !v->core.sdk_error && !v->cleanup_error) {
            if ((v->owner == JE2_YIELDING && v->core.phase == MK2_RELEASING) ||
                (v->owner == JE2_EXIT_BARRIER && v->core.phase == MK2_READY)) release_phase = 1;
            else if (v->owner == JE2_PENDANT && v->core.phase == MK2_READY && v->end_issued &&
                !v->had_writer && !v->core.owns_servo && !v->pending && !v->jog_live && v->idle_qualified)
                release_phase = 2;
        }
        json_object_object_add(owner, "session_release_sequence", json_object_new_uint64(session->release_sequence));
        json_object_object_add(owner, "session_release_phase", json_object_new_int(release_phase));
        json_object_object_add(feedback, "valid", json_object_new_boolean(sample->valid));
        json_object_object_add(feedback, "PowerState", json_object_new_int(sample->power));
        json_object_object_add(feedback, "MoveState", json_object_new_int(sample->move_state));
        json_object_object_add(feedback, "DriverError", json_object_new_int(sample->driver_error));
        json_object_object_add(feedback, "MoveError", json_object_new_int(sample->move_error));
        for (int i = 0; i < MK2_AXES; ++i)
            json_object_array_add(joints, isfinite(sample->joint[i]) ? json_object_new_double(sample->joint[i]) : NULL);
        json_object_object_add(feedback, "Joint", joints);
        json_object_object_add(owner, "snapshot_feedback", feedback);
        const char *body = json_object_to_json_string_ext(owner, JSON_C_TO_STRING_PLAIN);
        n = body ? snprintf(out, capacity, "Owner %s", body) : -1;
        json_object_put(owner);
        if (n <= 0 || (size_t)n >= capacity || n >= MK2_FRAME_MAX) { out[0] = '\0'; return -1; }
        return n;
    }
#else
    (void)sample; (void)sequence;
#endif
    if ((size_t)n >= capacity) return -1;
    memcpy(out, frame, (size_t)n + 1);
    return n;
}