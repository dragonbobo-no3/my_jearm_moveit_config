/* Explicit native-API semantics experiment, NOT bridge handover certification.
 * Compiled only into the dedicated binary, after all existing immutable
 * five-namespace/hash/model guards. No plugin, ROS, Servo or external client.
 * Intentionally observes the SDK's next-jog recovery in PRIVATE SIMULATION.
 * Never use these state=7 expectations as production permission to continue.
 */
struct teach_api {
    int (*select_robot)(int), (*coordinate)(int), (*enable)(void);
    int (*simulation)(void), (*power)(int), (*state)(int), (*driver)(int), (*error)(int);
    int (*jog)(int, int), (*stop)(void), (*force_stop)(int);
    void (*velocity)(double), (*move_type)(int), (*move_thread)(int), (*actual)(double *, int);
};
struct teach_sample {
    double at, joint[MK2_AXES];
    int simulation, power, state, driver, error;
};
static struct teach_sample teach_read(const struct teach_api *a)
{
    struct teach_sample s = {.at = now(), .simulation = a->simulation(),
        .power = a->power(0), .state = a->state(0), .driver = a->driver(0), .error = a->error(0)};
    a->actual(s.joint, 0);
    return s;
}
static bool teach_bounded(const struct teach_sample *s, const double *origin, double since)
{
    if (!finite_value(s->at) || s->at < since || s->simulation != 1 || s->power != 1 ||
        s->driver || s->error || s->state < 0) return false;
    for (int i = 0; i < MK2_AXES; ++i) {
        double delta = s->joint[i] - origin[i];
        if (!finite_value(s->joint[i]) || delta >= .02 || delta <= -.02 ||
            (i && (delta > 1e-5 || delta < -1e-5))) return false;
    }
    return true;
}
static void teach_report(const char *stage, unsigned epoch, const struct teach_sample *s)
{
    printf("JE2_TEACH_SAMPLE stage=%s epoch=%u monotonic=%.9f power=%d state=%d driver=%d error=%d j1=%.17g\n",
        stage, epoch, s->at, s->power, s->state, s->driver, s->error, s->joint[0]);
}
static bool teach_stationary_seven(const struct teach_api *a, const double *origin,
                                   double since, unsigned epoch)
{
    double stable_at = -1, previous = since, reference[MK2_AXES] = {0};
    for (unsigned tick = 0; tick < 50; ++tick) {
        struct teach_sample s = teach_read(a);
        if (!teach_bounded(&s, origin, previous) || s.at - since >= .5) return false;
        previous = s.at;
        /* 7 is the observation under test, NOT an idle/ownership ACK. */
        if (s.state == 7) {
            if (stable_at < 0) { stable_at = s.at; memcpy(reference, s.joint, sizeof(reference)); }
            for (int i = 0; i < MK2_AXES; ++i)
                if (s.joint[i] - reference[i] > 1e-5 || s.joint[i] - reference[i] < -1e-5) return false;
            if (s.at - stable_at >= .1) {
                teach_report("stop7_observed_NOT_idle_ack", epoch, &s);
                return true;
            }
        } else stable_at = -1;
        writer_pause();
    }
    return false;
}
enum teach_stop_source { TEACH_NATIVE_STOP, TEACH_EXTERNAL_STOP, TEACH_OVERLAP_STOP };
static int teach_experiment(const struct teach_api *a, enum teach_stop_source source)
{
    bool attempted = false, stopped = false;
    unsigned epochs = 0;
    int result = 79;
    struct teach_sample initial = teach_read(a);
    puts("JE2_TEACH_EXPERIMENT VIRTUAL_ONLY=1 PLUGIN_LOADED=0 SERVO_STARTED=0 EXPLICIT_RECOVERY_CALLS=0 SDK_INTERNAL_RECOVERY_POSSIBLE=1 PRODUCTION_READY=0");
    printf("JE2_STOP_SOURCE mode=%s INJECTED_BY_TEST=1 HARDWARE_ESTOP_TEST=0\n",
        source == TEACH_NATIVE_STOP ? "native_teach" : source == TEACH_EXTERNAL_STOP ? "external_RobotStop" : "teach_then_RobotStop");
    if (initial.simulation != 1 || initial.power != 0 || initial.state != 0 || initial.driver || initial.error ||
        !finite_value(initial.at) || initial.at < 0) goto done;
    for (int i = 0; i < MK2_AXES; ++i) if (!finite_value(initial.joint[i])) goto done;
    if (a->select_robot(0) || a->coordinate(0)) goto done;
    a->move_thread(1); a->move_type(0); a->velocity(.01);
    int enabled = a->enable();
    printf("JE2_TEACH_VIRTUAL_ENABLE result=%d\n", enabled);
    if (enabled) goto done;
    struct teach_sample s = teach_read(a);
    for (unsigned tick = 0; tick < 50 && s.power != 1; ++tick) { writer_pause(); s = teach_read(a); }
    if (!teach_bounded(&s, initial.joint, initial.at) || s.state != 0) goto done;
    /* External/overlap tests stop after ONE jog: never exercise recovery or
     * retry after the test's force-stop, including when our own STOP preceded it. */
    const unsigned required_epochs = source == TEACH_NATIVE_STOP ? 2 : 1;
    for (unsigned epoch = 1; epoch <= required_epochs; ++epoch) {
        s = teach_read(a);
        if (!teach_bounded(&s, initial.joint, initial.at) || s.state != (epoch == 1 ? 0 : 7)) goto done;
        teach_report("before_native_jog", epoch, &s);
        const double before = s.joint[0], start_time = s.at;
        attempted = true; stopped = false;
        int rc = a->jog(0, 1); /* Native API itself may recover. No explicit reset. */
        printf("JE2_TEACH_JOG epoch=%u result=%d\n", epoch, rc);
        if (rc) goto done;
        bool moved = false;
        for (unsigned tick = 0; tick < 20; ++tick) {
            s = teach_read(a);
            if (!teach_bounded(&s, initial.joint, start_time) || s.at - start_time >= .2 || s.state == 7) goto done;
            if (s.joint[0] - before > 1e-6) { moved = true; break; }
            writer_pause();
        }
        if (!moved) goto done;
        teach_report("actual_virtual_displacement", epoch, &s);
        stopped = true;
        if (source != TEACH_EXTERNAL_STOP) {
            rc = a->stop();
            printf("JE2_TEACH_STOP epoch=%u result=%d\n", epoch, rc);
            if (rc) goto done;
        }
        if (source != TEACH_NATIVE_STOP) {
            rc = a->force_stop(0);
            printf("JE2_EXTERNAL_STOP epoch=%u result=%d AFTER_OWN_TEACH_STOP=%d NO_SUBSEQUENT_JOG=1\n",
                epoch, rc, source == TEACH_OVERLAP_STOP);
        }
        if (rc || !teach_stationary_seven(a, initial.joint, s.at, epoch)) goto done;
        ++epochs;
    }
    result = 0;
done:
    if (attempted && !stopped) {
        int rc = a->stop();
        printf("JE2_TEACH_FAILURE_STOP result=%d NO_RETRY_OR_RECOVERY=1\n", rc);
    }
    s = teach_read(a);
    teach_report("final", epochs, &s);
    if (source != TEACH_NATIVE_STOP)
        printf("JE2_STOP_SOURCE_OBSERVATION power=%d state=%d driver=%d error=%d STOP_ORIGIN_NOT_ENCODED_IN_THESE_FIELDS=1 NO_RESUME_ATTEMPT=1\n",
            s.power, s.state, s.driver, s.error);
    printf("JE2_TEACH_FINISHED result=%s completed_epochs=%u SEMANTICS_ONLY=1 HANDOVER_CERTIFIED=0 SDK_RETAINED_UNTIL_PRIVATE_PROCESS_EXIT=1\n",
        result ? "FAIL" : "PASS", epochs);
    return result;
}
static int teach_probe(void *sdk, enum teach_stop_source source)
{
    struct teach_api a = {0};
    SYMBOL(sdk, a.select_robot, "set_robot_index"); SYMBOL(sdk, a.coordinate, "set_robot_teach_coordinate");
    SYMBOL(sdk, a.enable, "robot_teach_enable"); SYMBOL(sdk, a.simulation, "GetSystemSimulationState");
    SYMBOL(sdk, a.power, "GetRobotPowerState"); SYMBOL(sdk, a.state, "get_robot_move_state");
    SYMBOL(sdk, a.driver, "IsRobotDriverError"); SYMBOL(sdk, a.error, "IsRobotMoveError");
    SYMBOL(sdk, a.jog, "robot_teach_joint"); SYMBOL(sdk, a.stop, "robot_teach_stop");
    SYMBOL(sdk, a.velocity, "set_robot_teach_velocity"); SYMBOL(sdk, a.move_type, "set_teach_move_type");
    SYMBOL(sdk, a.move_thread, "setMoveThread"); SYMBOL(sdk, a.actual, "GetCurrentJoint");
    if (source != TEACH_NATIVE_STOP) SYMBOL(sdk, a.force_stop, "RobotStop");
    return teach_experiment(&a, source);
}