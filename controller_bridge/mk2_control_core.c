/* Original, SDK-independent command validator/state machine. */
#include "mk2_control_core.h"
#include <arpa/inet.h>
#include <ctype.h>
#include <json-c/json.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static struct json_object *strict_json(const char *text, size_t length)
{
    if (!text || !length || length > MK2_FRAME_MAX || memchr(text, '\0', length)) return NULL;
    struct json_tokener *parser = json_tokener_new_ex(12);
    if (!parser) return NULL;
    json_tokener_set_flags(parser, JSON_TOKENER_STRICT);
    struct json_object *value = json_tokener_parse_ex(parser, text, (int)length);
    enum json_tokener_error error = json_tokener_get_error(parser);
    size_t end = json_tokener_get_parse_end(parser);
    while (end < length && isspace((unsigned char)text[end])) end++;
    if (error != json_tokener_success || end != length ||
        !value || json_object_get_type(value) != json_type_object) {
        if (value) json_object_put(value);
        value = NULL;
    }
    json_tokener_free(parser);
    return value;
}

static struct json_object *field(struct json_object *object, const char *key)
{
    struct json_object *value = NULL;
    if (!object || !json_object_object_get_ex(object, key, &value)) return NULL;
    return value;
}
static bool number(struct json_object *value, double *output)
{
    if (!value || (json_object_get_type(value) != json_type_int &&
                   json_object_get_type(value) != json_type_double)) return false;
    *output = json_object_get_double(value);
    return isfinite(*output);
}
static bool array7(struct json_object *value, double *output)
{
    if (!value || json_object_get_type(value) != json_type_array ||
        json_object_array_length(value) != MK2_AXES) return false;
    for (size_t i = 0; i < MK2_AXES; i++)
        if (!number(json_object_array_get_idx(value, i), output + i)) return false;
    return true;
}
static bool finite_sample(const struct mk2_sample *sample)
{
    if (!sample || !sample->valid) return false;
    for (int i = 0; i < MK2_AXES; i++) if (!isfinite(sample->joint[i])) return false;
    return true;
}
static bool powered_healthy(const struct mk2_sample *sample)
{
    return finite_sample(sample) && sample->power == 1 && sample->driver_error == 0 &&
           sample->move_error == 0 && sample->move_state >= 0 && sample->move_state != 7;
}
static bool policy_valid(const struct mk2_policy *p)
{
    if (!p || !isfinite(p->watchdog) || p->watchdog <= 0 || p->watchdog > 0.5 ||
        !isfinite(p->first_timeout) || p->first_timeout < p->watchdog || p->first_timeout > 5 ||
        !isfinite(p->min_dt) || !isfinite(p->max_dt) || p->min_dt < 0.001 ||
        p->max_dt < p->min_dt || p->max_dt > 0.05 ||
        !isfinite(p->clock_skew) || p->clock_skew <= 0 || p->clock_skew > 0.25 ||
        !isfinite(p->first_hold_tolerance) || p->first_hold_tolerance <= 0 ||
        p->first_hold_tolerance > 0.002 || !isfinite(p->following_tolerance) ||
        p->following_tolerance < p->first_hold_tolerance || p->following_tolerance > 0.05 ||
        !isfinite(p->servo_q) || !isfinite(p->servo_r) || p->servo_q <= 0 || p->servo_r <= 0)
        return false;
    for (int i = 0; i < MK2_AXES; i++)
        if (!isfinite(p->lower[i]) || !isfinite(p->upper[i]) || p->lower[i] >= p->upper[i] ||
            !isfinite(p->velocity[i]) || p->velocity[i] <= 0 || p->velocity[i] > 0.3 ||
            !isfinite(p->acceleration[i]) || p->acceleration[i] <= 0 || p->acceleration[i] > 0.5)
            return false;
    return true;
}
int mk2_policy_parse(const char *text, size_t size, struct mk2_policy *p, char *error, size_t error_size)
{
    struct json_object *root = strict_json(text, size);
    struct mk2_policy parsed = {0};
    struct json_object *value, *limits;
    struct in_addr address;
    bool valid = root && json_object_object_length(root) == 16;
    double version = 0, port = 0;
    if (valid) valid = number(field(root, "version"), &version) && version == 1;
    value = field(root, "allow_commands");
    if (valid) valid = value && json_object_get_type(value) == json_type_boolean;
    if (valid) parsed.allow_commands = json_object_get_boolean(value);
    value = field(root, "command_host");
    if (valid) valid = value && json_object_get_type(value) == json_type_string &&
                       strlen(json_object_get_string(value)) < sizeof(parsed.command_host) &&
                       inet_pton(AF_INET, json_object_get_string(value), &address) == 1;
    if (valid) snprintf(parsed.command_host, sizeof(parsed.command_host), "%s", json_object_get_string(value));
    if (valid) valid = number(field(root, "command_port"), &port) && port >= 1024 && port <= 65535 && floor(port) == port;
    if (valid) parsed.command_port = (int)port;
    if (valid) valid = number(field(root, "state_port"), &port) && port >= 1024 && port <= 65535 && floor(port) == port;
    if (valid) parsed.state_port = (int)port;
    value = field(root, "device_config_sha256");
    if (valid) valid = value && json_object_get_type(value) == json_type_string && strlen(json_object_get_string(value)) == 64;
    if (valid) {
        snprintf(parsed.device_config_sha256, sizeof(parsed.device_config_sha256), "%s", json_object_get_string(value));
        for (int i = 0; i < 64; i++) if (!isxdigit((unsigned char)parsed.device_config_sha256[i])) valid = false;
    }
#define READ_NUMBER(key, member) if (valid) valid = number(field(root, key), &parsed.member)
    READ_NUMBER("watchdog_seconds", watchdog);
    READ_NUMBER("first_command_timeout", first_timeout);
    READ_NUMBER("min_dt", min_dt);
    READ_NUMBER("max_dt", max_dt);
    READ_NUMBER("clock_skew", clock_skew);
    READ_NUMBER("first_hold_tolerance", first_hold_tolerance);
    READ_NUMBER("following_tolerance", following_tolerance);
    READ_NUMBER("servo_q", servo_q);
    READ_NUMBER("servo_r", servo_r);
#undef READ_NUMBER
    limits = field(root, "limits");
    if (valid) valid = limits && json_object_get_type(limits) == json_type_object && json_object_object_length(limits) == 4;
    if (valid) valid = array7(field(limits, "lower"), parsed.lower) && array7(field(limits, "upper"), parsed.upper) &&
                       array7(field(limits, "velocity"), parsed.velocity) && array7(field(limits, "acceleration"), parsed.acceleration);
    valid = valid && policy_valid(&parsed);
    if (root) json_object_put(root);
    if (!valid || !p) {
        if (error && error_size) snprintf(error, error_size, "Invalid control policy/schema or commissioning caps");
        return -1;
    }
    *p = parsed;
    return 0;
}

int mk2_core_init(struct mk2_core *core, const struct mk2_policy *policy, const struct mk2_io *io)
{
    if (!core || !policy_valid(policy) || !io || !io->start || !io->joint || !io->stop || !io->end) return -1;
    memset(core, 0, sizeof(*core));
    core->policy = *policy;
    core->io = *io;
    core->phase = policy->allow_commands ? MK2_READY : MK2_LOCKED;
    core->rearm_since = -1;
    return 0;
}
static bool sample_in_bounds(const struct mk2_core *core, const struct mk2_sample *sample)
{
    if (!powered_healthy(sample)) return false;
    for (int i = 0; i < MK2_AXES; i++)
        if (sample->joint[i] < core->policy.lower[i] || sample->joint[i] > core->policy.upper[i]) return false;
    return true;
}
bool mk2_core_commands_available(const struct mk2_core *core, const struct mk2_sample *sample)
{
    return core->policy.allow_commands && sample_in_bounds(core, sample) &&
           (core->phase == MK2_ACTIVE ||
            ((core->phase == MK2_READY || core->phase == MK2_WAIT_FIRST) && sample->move_state == 0));
}
void mk2_core_record_state_error(struct mk2_core *core, const char *site, double now,
                                const struct mk2_sample *sample, bool have_command_time, double command_time)
{
    if (core->first_state_error.captured || core->fault != MK2_NO_FAULT) return;
    struct mk2_state_failure *failure = &core->first_state_error;
    failure->captured = true;
    failure->site = site;
    failure->phase = core->phase;
    failure->monotonic_time = now;
    failure->have_sample = sample != NULL;
    failure->have_command_time = have_command_time;
    failure->command_time = command_time;
    if (sample) {
        failure->sample = *sample;
        if (!sample->valid) failure->failed_checks |= MK2_CHECK_INVALID_SAMPLE;
        if (sample->power != 1) failure->failed_checks |= MK2_CHECK_POWER;
        if (sample->driver_error != 0) failure->failed_checks |= MK2_CHECK_DRIVER;
        if (sample->move_error != 0) failure->failed_checks |= MK2_CHECK_MOVE_ERROR;
        if (sample->move_state < 0 || sample->move_state == 7) failure->failed_checks |= MK2_CHECK_MOVE_STATE;
        if (core->phase == MK2_WAIT_FIRST && sample->move_state != 0)
            failure->failed_checks |= MK2_CHECK_FIRST_NOT_IDLE;
        for (int i = 0; i < MK2_AXES; i++) {
            if (!isfinite(sample->joint[i])) failure->failed_checks |= MK2_CHECK_NONFINITE;
            else if (sample->joint[i] < core->policy.lower[i] || sample->joint[i] > core->policy.upper[i])
                failure->failed_checks |= MK2_CHECK_BOUNDS;
        }
    }
    if (have_command_time && command_time <= 0) failure->failed_checks |= MK2_CHECK_COMMAND_TIME;
}
void mk2_core_stop(struct mk2_core *core, enum mk2_fault reason)
{
    bool latched = false;
    if (core->owns_servo) {
        int stop = core->io.stop(core->io.context);
        int end = core->io.end(core->io.context);
        if (!core->sdk_error) core->sdk_error = stop ? stop : end;
        core->owns_servo = false;
        latched = stop == 0;
    }
    core->fault = core->sdk_error ? MK2_SDK_ERROR : reason;
    core->phase = core->fault == MK2_NO_FAULT ? MK2_STOPPED : MK2_FAULT;
    core->rearm_since = -1;
    core->stop_latch_ours = latched;
    core->rearm_recovered = false;
}
/* Eligible ONLY for a clean stop, or a fault whose first cause, observed by the
 * active-servo tick, was the SDK forced stop (move_state 7) and/or power loss
 * (pendant disable), with no other failed check. Re-enable is the operator's
 * action; rearm itself still needs sustained powered healthy idle. Driver/move
 * errors, bounds, watchdog, timing, limits, frames and SDK call failures stay
 * latched until RobotMain restart. */
static bool rearm_cause_ok(const struct mk2_core *core)
{
    if (!core->rearm_after_forced_stop || !core->policy.allow_commands ||
        core->owns_servo || core->sdk_error) return false;
    if (core->phase == MK2_STOPPED) return core->fault == MK2_NO_FAULT;
    const struct mk2_state_failure *f = &core->first_state_error;
    return core->phase == MK2_FAULT && core->fault == MK2_STATE_ERROR && f->captured &&
           f->have_sample && f->site && !strcmp(f->site, "control_tick") &&
           f->phase == MK2_ACTIVE && f->failed_checks &&
           !(f->failed_checks & ~(MK2_CHECK_MOVE_STATE | MK2_CHECK_POWER));
}
/* Healthy, powered, in-bounds feedback whose only abnormality is the SDK stop
 * latch (move_state 7). */
static bool stop_latched_idle(const struct mk2_core *core, const struct mk2_sample *sample)
{
    if (!sample || sample->move_state != 7) return false;
    struct mk2_sample unlatched = *sample;
    unlatched.move_state = 0;
    return sample_in_bounds(core, &unlatched);
}
/* READY moves nothing: a new session still needs the PC preflight, Switch, a
 * first hold at the measured pose and all first-frame checks, exactly as after
 * a RobotMain restart. Our own RobotStop(0) leaves move_state 7 latched; that
 * latch (and only while it has been continuously ours: no idle, power, error or
 * bounds change since the stop) is cleared by ONE io.recover after the same
 * stationary interval, then the normal move_state 0 interval is required. */
static void tick_rearm(struct mk2_core *core, double now, const struct mk2_sample *sample)
{
    bool latched = stop_latched_idle(core, sample);
    bool idle = sample_in_bounds(core, sample) && sample->move_state == 0;
    if (idle || (!latched && !sample_in_bounds(core, sample)) ||
        (core->first_state_error.captured && (core->first_state_error.failed_checks & MK2_CHECK_POWER)))
        core->stop_latch_ours = false; /* a 7 after power loss may be the pendant's */
    bool recoverable = latched && core->stop_latch_ours && core->io.recover && !core->rearm_recovered;
    if (!rearm_cause_ok(core) || !isfinite(now) || !(idle || recoverable)) {
        core->rearm_since = -1;
        return;
    }
    bool stationary = core->rearm_since >= 0 && now >= core->rearm_since;
    for (int i = 0; stationary && i < MK2_AXES; i++)
        if (fabs(sample->joint[i] - core->rearm_joint[i]) > MK2_REARM_STATIONARY_RAD) stationary = false;
    if (!stationary) {
        memcpy(core->rearm_joint, sample->joint, sizeof(core->rearm_joint));
        core->rearm_since = now;
        return;
    }
    if (now - core->rearm_since < MK2_REARM_SETTLE_SECONDS) return;
    if (latched) {
        core->rearm_recovered = true;
        core->rearm_since = -1;
        core->rearm_recovers++;
        core->sdk_error = core->io.recover(core->io.context);
        if (core->sdk_error) mk2_core_stop(core, MK2_SDK_ERROR); /* latched for good */
        return;
    }
    core->phase = MK2_READY;
    core->fault = MK2_NO_FAULT;
    core->first_state_error = (struct mk2_state_failure){0};
    core->servo_observation = (struct mk2_servo_observation){0};
    core->rearm_since = -1;
    core->rearm_count++;
}
static int reject(struct mk2_core *core, enum mk2_fault reason)
{
    core->rejected_frames++;
    if (core->owns_servo || core->phase == MK2_WAIT_FIRST) mk2_core_stop(core, reason);
    return -1;
}
static void observe_servo_sample(struct mk2_core *core, double now, const struct mk2_sample *sample)
{
    if (core->phase != MK2_ACTIVE || !core->owns_servo || !sample) return;
    struct mk2_servo_observation *observation = &core->servo_observation;
    if (finite_sample(sample) && sample->move_state == MK2_OBSERVED_SDK_SERVO_STATE)
        observation->seen_servo_state = true;
    if (observation->seen_servo_state && sample->move_state != MK2_OBSERVED_SDK_SERVO_STATE &&
        !observation->first_nonservo_captured) {
        observation->first_nonservo_captured = true;
        observation->servo_state_seen_before = observation->seen_servo_state;
        observation->input_monotonic_time = now;
        observation->first_nonservo_sample = *sample;
    }
}
static void observe_sdk_call(struct mk2_core *core, enum mk2_sdk_call call, int result,
                             double now, double command_time, const struct mk2_sample *input)
{
    core->servo_observation.last_call = (struct mk2_sdk_call_observation){
        .call = call, .phase = core->phase, .result = result,
        .input_monotonic_time = now, .command_time = command_time, .input_sample = *input
    };
}
void mk2_core_tick(struct mk2_core *core, double now, const struct mk2_sample *sample)
{
    /* Memory copies only, before Stop/End can alter the SDK input. This does
     * not classify a pendant event or change any guard/phase below. */
    observe_servo_sample(core, now, sample);
    if (core->phase == MK2_RELEASING) {
        /* SDK StopRun is not StopRecover. Verify a fresh stationary interval
         * before closing servo; no new Joint/Switch intent during release. */
        if (!isfinite(now) || now < core->last_receive) {
            mk2_core_stop(core, MK2_CLOCK_ERROR); return;
        }
        core->last_receive = now;
        if (!sample_in_bounds(core, sample)) {
            mk2_core_record_state_error(core, "release_tick", now, sample, false, 0);
            mk2_core_stop(core, MK2_STATE_ERROR); return;
        }
        if (now - core->release_start > core->policy.first_timeout) {
            mk2_core_stop(core, MK2_WATCHDOG); return;
        }
        bool stable = sample->move_state == 0 && core->release_stable_since >= 0;
        for (int i = 0; stable && i < MK2_AXES; i++)
            if (fabs(sample->joint[i] - core->release_joint[i]) > 1e-5) stable = false;
        if (!stable) {
            memcpy(core->release_joint, sample->joint, sizeof(core->release_joint));
            core->release_stable_since = sample->move_state == 0 ? now : -1;
        } else if (now - core->release_stable_since >= 0.1) {
            core->sdk_error = core->io.end(core->io.context);
            if (core->sdk_error) mk2_core_stop(core, MK2_SDK_ERROR);
            else { core->owns_servo = false; core->phase = MK2_READY; }
        }
        return;
    }
    if (!core->owns_servo && (core->phase == MK2_STOPPED || core->phase == MK2_FAULT)) {
        tick_rearm(core, now, sample);
        return;
    }
    if (!core->owns_servo && core->phase != MK2_WAIT_FIRST) return;
    if (!isfinite(now) || now < core->last_receive) mk2_core_stop(core, MK2_CLOCK_ERROR);
    else if (!sample_in_bounds(core, sample) || (core->phase == MK2_WAIT_FIRST && sample->move_state != 0)) {
        mk2_core_record_state_error(core, "control_tick", now, sample, false, 0);
        mk2_core_stop(core, MK2_STATE_ERROR);
    }
    else if (now - core->last_receive > (core->phase == MK2_WAIT_FIRST ? core->policy.first_timeout : core->policy.watchdog))
        mk2_core_stop(core, MK2_WATCHDOG);
}
/* Protocol field names must be literal ASCII, exactly once. Combined with exact
 * object/field counts below this rejects duplicate keys and escaped key aliases
 * instead of relying on json-c's otherwise last-key-wins behavior. */
static bool once(const char *text, size_t length, const char *literal)
{
    size_t n = strlen(literal), count = 0;
    for (size_t i = 0; i + n <= length; i++) if (!memcmp(text + i, literal, n)) count++;
    return count == 1;
}
int mk2_core_handle(struct mk2_core *core, const char *frame, size_t length, double now, const struct mk2_sample *sample)
{
    mk2_core_tick(core, now, sample);
    if (!frame || length > MK2_FRAME_MAX || !length || memchr(frame, '\0', length) || memchr(frame, '\\', length))
        return reject(core, MK2_BAD_FRAME);
    if (!isfinite(now) || now < 0) return reject(core, MK2_CLOCK_ERROR);
    size_t prefix = length > 8 && !memcmp(frame, "Release ", 8) ? 8 :
                    length > 7 && !memcmp(frame, "Switch ", 7) ? 7 :
                    length > 6 && !memcmp(frame, "Joint ", 6) ? 6 : 0;
    if (!prefix) return reject(core, MK2_BAD_FRAME);
    struct json_object *object = strict_json(frame + prefix, length - prefix);
    if (!object) return reject(core, MK2_BAD_FRAME);
    struct json_object *value;
    int result = -1;
    if (prefix == 8) {
        value = field(object, "Release");
        if (json_object_object_length(object) != 1 || !value ||
            json_object_get_type(value) != json_type_boolean || !json_object_get_boolean(value) ||
            !once(frame, length, "\"Release\"")) result = reject(core, MK2_BAD_FRAME);
        else if (!core->io.stop_run || core->fault != MK2_NO_FAULT || core->sdk_error ||
                 (core->phase != MK2_ACTIVE && core->phase != MK2_WAIT_FIRST) ||
                 !sample_in_bounds(core, sample)) core->rejected_frames++;
        else if (!core->owns_servo) {
            /* Cancel an unfulfilled intent, with no SDK mutation. */
            core->phase = MK2_READY; result = 0;
        } else {
            core->sdk_error = core->io.stop_run(core->io.context);
            if (core->sdk_error) mk2_core_stop(core, MK2_SDK_ERROR);
            else {
                core->phase = MK2_RELEASING;
                core->release_start = core->last_receive = now;
                core->release_stable_since = -1;
                result = 0;
            }
        }
    } else if (prefix == 7) {
        value = field(object, "Switch");
        if (json_object_object_length(object) != 1 || !value || json_object_get_type(value) != json_type_boolean ||
            !once(frame, length, "\"Switch\"")) result = reject(core, MK2_BAD_FRAME);
        else if (!json_object_get_boolean(value)) {
            /* Never power off; never stop a servo this plugin did not start. */
            if (core->owns_servo || core->phase == MK2_WAIT_FIRST) mk2_core_stop(core, MK2_NO_FAULT);
            result = 0;
        } else if (core->phase != MK2_READY || !mk2_core_commands_available(core, sample)) {
            /* A repeated enable is not a heartbeat or a fault-reset operation. */
            core->rejected_frames++;
        } else {
            /* Switch only arms an intent. Do NOT open SDK servo before a fresh,
             * bounded hold target has arrived and passed all first-frame checks. */
            core->phase = MK2_WAIT_FIRST; core->last_receive = now; result = 0;
        }
    } else if (core->phase != MK2_WAIT_FIRST && core->phase != MK2_ACTIVE) {
        result = reject(core, MK2_NOT_READY);
    } else {
        double data_time = NAN, target[MK2_AXES], dt = 0, velocity[MK2_AXES] = {0};
        value = field(object, "Robot0");
        bool valid = json_object_object_length(object) == 1 && value && json_object_get_type(value) == json_type_object &&
            json_object_object_length(value) == 2 && once(frame, length, "\"Robot0\"") &&
            once(frame, length, "\"time\"") && once(frame, length, "\"joint\"") &&
            number(field(value, "time"), &data_time) && array7(field(value, "joint"), target);
        enum mk2_fault fault = valid ? MK2_NO_FAULT : MK2_BAD_FRAME;
        if (valid && (data_time <= 0 || !powered_healthy(sample))) fault = MK2_STATE_ERROR;
        bool first = core->phase == MK2_WAIT_FIRST;
        if (!fault && first && data_time > 0.2) fault = MK2_BAD_TIME;
        if (!fault && !first) {
            dt = data_time - core->last_data_time;
            if (!isfinite(dt) || dt < core->policy.min_dt - 1e-9 || dt > core->policy.max_dt + 1e-9 ||
                fabs((data_time - core->first_data_time) - (now - core->first_receive)) > core->policy.clock_skew)
                fault = MK2_BAD_TIME;
        }
        for (int i = 0; !fault && i < MK2_AXES; i++) {
            if (target[i] < core->policy.lower[i] || target[i] > core->policy.upper[i]) fault = MK2_LIMIT;
            else if (fabs(target[i] - sample->joint[i]) > (first ? core->policy.first_hold_tolerance : core->policy.following_tolerance))
                fault = first ? MK2_FIRST_TARGET : MK2_FOLLOWING;
            else if (!first) {
                velocity[i] = (target[i] - core->last_target[i]) / dt;
                double acceleration = (velocity[i] - core->last_velocity[i]) / dt;
                if (!isfinite(velocity[i]) || fabs(velocity[i]) > core->policy.velocity[i] + 1e-9) fault = MK2_VELOCITY;
                else if (!isfinite(acceleration) || fabs(acceleration) > core->policy.acceleration[i] + 1e-9) fault = MK2_ACCELERATION;
            }
        }
        if (fault) {
            if (fault == MK2_STATE_ERROR)
                mk2_core_record_state_error(core, "joint_command", now, sample, true, data_time);
            result = reject(core, fault);
        }
        else {
            /* Copy BEFORE callbacks: even a failed SDK call may change state.
             * No extra SDK read, formatting or logging before the stop path. */
            const struct mk2_sample sdk_input = *sample;
            if (first) {
                core->servo_observation = (struct mk2_servo_observation){0};
                /* Gen-1-style activation recovery: clear a latched forced-stop
                 * (observed SDK move_state 7) once, BEFORE opening servo. This
                 * never powers off or clears driver/move faults; those were
                 * already rejected above. A failed recover means servo was not
                 * opened, so no stop/end cleanup is owed. */
                if (core->io.recover) {
                    core->sdk_error = core->io.recover(core->io.context);
                    observe_sdk_call(core, MK2_SDK_CALL_RECOVER, core->sdk_error, now, data_time, &sdk_input);
                }
                /* A failed SDK start can have partial side effects, so acquire
                 * cleanup responsibility immediately before making that call. */
                if (!core->sdk_error) {
                    core->owns_servo = true;
                    core->sdk_error = core->io.start(core->io.context, core->policy.servo_q, core->policy.servo_r);
                    observe_sdk_call(core, MK2_SDK_CALL_START, core->sdk_error, now, data_time, &sdk_input);
                }
            }
            if (!core->sdk_error) {
                core->sdk_error = core->io.joint(core->io.context, target, data_time);
                observe_sdk_call(core, MK2_SDK_CALL_JOINT, core->sdk_error, now, data_time, &sdk_input);
            }
            if (core->sdk_error) mk2_core_stop(core, MK2_SDK_ERROR);
            else {
                if (first) { core->first_receive = now; core->first_data_time = data_time; }
                core->last_receive = now; core->last_data_time = data_time;
                memcpy(core->last_target, target, sizeof(target));
                memcpy(core->last_velocity, velocity, sizeof(velocity));
                core->accepted_joints++; core->phase = MK2_ACTIVE; result = 0;
            }
        }
    }
    json_object_put(object);
    return result;
}
const char *mk2_phase_name(enum mk2_phase phase)
{
    static const char *names[] = {"locked", "ready", "waiting_first_joint", "active", "stopped", "fault", "releasing"};
    return phase >= MK2_LOCKED && phase <= MK2_RELEASING ? names[phase] : "invalid";
}
const char *mk2_fault_name(enum mk2_fault fault)
{
    static const char *names[] = {"none", "bad_frame", "not_ready", "bad_time", "position_limit",
        "first_target_not_hold", "following_error", "velocity_limit", "acceleration_limit", "watchdog",
        "sdk_error", "state_error", "clock_error", "plugin_unload"};
    return fault >= MK2_NO_FAULT && fault <= MK2_UNLOAD ? names[fault] : "invalid";
}
const char *mk2_sdk_call_name(enum mk2_sdk_call call)
{
    static const char *names[] = {"none", "RobotStopRecover", "ServoStart", "ServoJoint"};
    return call >= MK2_SDK_CALL_NONE && call <= MK2_SDK_CALL_JOINT ? names[call] : "invalid";
}