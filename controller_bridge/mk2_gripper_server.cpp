// MK2 standalone Piper gripper server (controller side).
//
// The MK2 arm plugin inside RobotMain only accepts Switch/Joint/Release frames,
// so gripper commands use a dedicated port: the PC binds a PUB on
// mk2_gripper_port (default 8002) and this process SUB-connects to it.
// It drives the Piper gripper over SocketCAN with piper_gripper::GripperClient.
//
// Wire format (same fields as the legacy JEServer EndEffector slot):
//   Gripper {"Robot0":{"mode":0,"position":0.5}}
//   Gripper {"Robot0":{"mode":2,"torque":0.5,"position":0.0,"angle_0p001mm":0,"effort_0p001Nm":500}}
//   Gripper {"Robot0":{"command":"open"}} / {"command":"close"} / {"init":true}
// position 0..1 -> angle 0..70000 (0.001 mm opening), torque N*m -> effort 0.001 N*m.
//
// Startup never moves the gripper: it clears errors while disabled, waits for
// feedback, then enables at the measured opening.
//
// The gripper firmware only accepts 0x159 with DLC 7 (byte 6: bit0 enable,
// bit1 clear error); piper_gripper sends DLC 8 and is ignored, so control
// frames go through our own raw socket and the library is used for 0x2A8 only.

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <thread>

#include <zmq.h>

#include "nlohmann/json.hpp"
#include "piper/gripper/gripper_client.hpp"

namespace {

namespace pg = piper::gripper;
using Clock = std::chrono::steady_clock;

constexpr int32_t kAngleMax = 70000;
constexpr int kEffortMax = 5000;

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

struct Options {
  std::string can = "can1";
  std::string endpoint = "tcp://192.168.0.100:8002";
  std::string feedback_endpoint = "tcp://*:8003";
  std::string robot = "Robot0";
  int default_effort = 1000;
  bool probe = false;
};

void usage() {
  std::fprintf(stderr,
    "usage: mk2_gripper_server [--can can1] [--endpoint tcp://192.168.0.100:8002]\n"
    "                          [--feedback-endpoint tcp://*:8003] [--robot Robot0]\n"
    "                          [--default-effort 1000] [--probe]\n"
    "  --probe  clear errors while DISABLED, print feedback, exit (no enable, no motion)\n");
}

bool parse_args(int argc, char ** argv, Options * o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](std::string * v) {
      if (i + 1 >= argc) return false;
      *v = argv[++i];
      return true;
    };
    std::string v;
    if (a == "--can" && next(&o->can)) continue;
    if (a == "--endpoint" && next(&o->endpoint)) continue;
    if (a == "--feedback-endpoint" && next(&o->feedback_endpoint)) continue;
    if (a == "--robot" && next(&o->robot)) continue;
    if (a == "--default-effort" && next(&v)) {
      try { o->default_effort = std::stoi(v); } catch (...) { return false; }
      if (o->default_effort < 0 || o->default_effort > kEffortMax) return false;
      continue;
    }
    if (a == "--probe") { o->probe = true; continue; }
    return false;
  }
  return o->can.rfind("can", 0) == 0 && o->endpoint.rfind("tcp://", 0) == 0 &&
         o->feedback_endpoint.rfind("tcp://", 0) == 0 && !o->robot.empty();
}

const char * status_name(pg::StatusCode s) {
  switch (s) {
    case pg::StatusCode::Disable: return "Disable";
    case pg::StatusCode::Enable: return "Enable";
    case pg::StatusCode::DisableAndClearError: return "DisableAndClearError";
    case pg::StatusCode::EnableAndClearError: return "EnableAndClearError";
  }
  return "?";
}

void print_state(const pg::GripperState & s, const char * tag) {
  std::printf("[gripper] %s opening=%.2fmm effort=%.3fNm status=0x%02x enabled=%d err=%d "
              "volt_low=%d motor_hot=%d overcurrent=%d drv_hot=%d hz=%.1f\n",
              tag, s.angle_0p001mm * 0.001, s.effort_0p001Nm * 0.001, s.status_code,
              s.foc.driver_enable_status, s.foc.driver_error_status, s.foc.voltage_too_low,
              s.foc.motor_overheating, s.foc.driver_overcurrent, s.foc.driver_overheating, s.hz);
  std::fflush(stdout);
}

std::optional<pg::GripperState> fresh_state(const pg::GripperClient & c, double max_age_s) {
  const auto r = c.GetArmGripperMsgs();
  if (r.code != pg::ErrorCode::Ok || r.value.timestamp_sec <= 0.0) return std::nullopt;
  const double now = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  if (now - r.value.timestamp_sec > max_age_s) return std::nullopt;
  return r.value;
}

// TX-only raw CAN socket (receives nothing).
int open_can_tx(const std::string & ifname) {
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) return -1;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
  ifreq ifr{};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0 ||
      (addr.can_ifindex = ifr.ifr_ifindex,
       ::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)) {
    ::close(fd);
    return -1;
  }
  return fd;
}

bool send(int can_fd, const pg::GripperCommand & cmd) {
  can_frame f{};
  f.can_id = 0x159;
  f.can_dlc = 7;
  const uint32_t a = static_cast<uint32_t>(cmd.angle_0p001mm);
  f.data[0] = static_cast<uint8_t>(a >> 24);
  f.data[1] = static_cast<uint8_t>(a >> 16);
  f.data[2] = static_cast<uint8_t>(a >> 8);
  f.data[3] = static_cast<uint8_t>(a);
  f.data[4] = static_cast<uint8_t>(cmd.effort_0p001Nm >> 8);
  f.data[5] = static_cast<uint8_t>(cmd.effort_0p001Nm);
  f.data[6] = static_cast<uint8_t>(cmd.status);
  if (::write(can_fd, &f, sizeof(f)) != static_cast<ssize_t>(sizeof(f))) {
    std::fprintf(stderr, "[gripper] CAN send failed: %s\n", std::strerror(errno));
    return false;
  }
  return true;
}

// Applies one JSON command onto `cmd` (legacy JEServer semantics). Returns error text or "".
std::string apply_command(const nlohmann::json & j, int default_effort, pg::GripperCommand * cmd) {
  if (!j.is_object()) return "command is not an object";
  pg::GripperCommand out = *cmd;
  out.effort_0p001Nm = static_cast<uint16_t>(default_effort);
  out.status = pg::StatusCode::Enable;
  out.set_zero = pg::SetZero::Invalid;
  bool has_field = false;

  auto number = [&](const char * key, double * v) -> std::string {
    if (!j.contains(key)) return "";
    if (!j[key].is_number() || !std::isfinite(j[key].get<double>())) return std::string("invalid ") + key;
    *v = j[key].get<double>();
    return "-";
  };
  double v = 0.0;
  std::string r;

  if (j.contains("mode")) {
    if (!j["mode"].is_number_integer()) return "invalid mode";
    const int mode = j["mode"].get<int>();
    if (mode == 1) return "preset mode is not supported by the Piper gripper";
    if (mode != 0 && mode != 2) return "unknown mode";
  }
  if (j.contains("set_zero")) return "set_zero is not allowed remotely";
  if ((r = number("position", &v)) == "-") {
    out.angle_0p001mm = static_cast<int32_t>(std::lround(std::clamp(v, 0.0, 1.0) * kAngleMax));
    has_field = true;
  } else if (!r.empty()) return r;
  if ((r = number("force", &v)) == "-") {
    out.effort_0p001Nm = static_cast<uint16_t>(std::lround(std::clamp(v, 0.0, 100.0) * 50.0));
    has_field = true;
  } else if (!r.empty()) return r;
  if ((r = number("torque", &v)) == "-") {
    out.effort_0p001Nm = static_cast<uint16_t>(std::lround(std::clamp(v, 0.0, kEffortMax * 0.001) * 1000.0));
    has_field = true;
  } else if (!r.empty()) return r;
  if ((r = number("angle_0p001mm", &v)) == "-") {
    out.angle_0p001mm = static_cast<int32_t>(std::clamp<long>(std::lround(v), 0, kAngleMax));
    has_field = true;
  } else if (!r.empty()) return r;
  if ((r = number("effort_0p001Nm", &v)) == "-") {
    out.effort_0p001Nm = static_cast<uint16_t>(std::clamp<long>(std::lround(v), 0, kEffortMax));
    has_field = true;
  } else if (!r.empty()) return r;

  const char * action_key = j.contains("command") ? "command" : j.contains("gripper") ? "gripper" :
                            j.contains("action") ? "action" : nullptr;
  if (action_key) {
    if (!j[action_key].is_string()) return "invalid open/close command";
    std::string a = j[action_key].get<std::string>();
    std::transform(a.begin(), a.end(), a.begin(), [](unsigned char ch) { return std::tolower(ch); });
    if (a == "open") out.angle_0p001mm = kAngleMax;
    else if (a == "close") out.angle_0p001mm = 0;
    else return "unsupported open/close command";
    has_field = true;
  }

  bool status_explicit = false;
  if (j.contains("status")) {
    if (!j["status"].is_number_integer()) return "invalid status";
    const int s = j["status"].get<int>();
    if (s < 0 || s > 3) return "unsupported status";
    out.status = static_cast<pg::StatusCode>(s);
    status_explicit = true;
    has_field = true;
  }
  if (j.contains("init") && j["init"].is_boolean() && j["init"].get<bool>() && !status_explicit) {
    out.status = pg::StatusCode::EnableAndClearError;
    has_field = true;
  }
  if (!has_field) return "missing/invalid fields";
  *cmd = out;
  return "";
}

}  // namespace

int main(int argc, char ** argv) {
  Options opt;
  if (!parse_args(argc, argv, &opt)) {
    usage();
    return 2;
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  setvbuf(stdout, nullptr, _IOLBF, 0);

  // One owner per CAN bus.
  const std::string lock_path = "/tmp/mk2_gripper_server." + opt.can + ".lock";
  const int lock_fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
  if (lock_fd < 0 || ::flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
    std::fprintf(stderr, "[gripper] another mk2_gripper_server owns %s\n", opt.can.c_str());
    return 3;
  }

  pg::Config cfg;
  cfg.can_ifname = opt.can;
  cfg.recv_timeout_ms = 100;
  pg::GripperClient client(cfg);
  if (const auto r = client.Start(); r.code != pg::ErrorCode::Ok) {
    std::fprintf(stderr, "[gripper] start on %s failed: %s\n", opt.can.c_str(), r.message.c_str());
    return 4;
  }
  const int can_fd = open_can_tx(opt.can);
  if (can_fd < 0) {
    std::fprintf(stderr, "[gripper] CAN TX socket on %s failed: %s\n", opt.can.c_str(), std::strerror(errno));
    return 4;
  }

  // Clear errors while disabled (no motion) until the gripper answers.
  pg::GripperCommand cmd;
  cmd.angle_0p001mm = 0;
  cmd.effort_0p001Nm = static_cast<uint16_t>(opt.default_effort);
  cmd.status = pg::StatusCode::DisableAndClearError;
  std::optional<pg::GripperState> st;
  const auto probe_deadline = Clock::now() + std::chrono::milliseconds(1500);
  while (!g_stop.load() && Clock::now() < probe_deadline && !(st = fresh_state(client, 0.5))) {
    send(can_fd, cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!st) {
    std::fprintf(stderr, "[gripper] no feedback (CAN id 0x2A8) on %s; check bus/power. Not enabling.\n",
                 opt.can.c_str());
    return 5;
  }
  print_state(*st, "after clear (disabled):");
  if (opt.probe) {
    std::printf("[gripper] probe done; gripper left DISABLED\n");
    return 0;
  }

  // Hold exactly where it is (feedback can be slightly below 0 when fully closed).
  cmd.angle_0p001mm = std::min(st->angle_0p001mm, kAngleMax);
  cmd.status = pg::StatusCode::Enable;
  if (!send(can_fd, cmd)) return 6;
  std::printf("[gripper] enabled at current opening %.2fmm effort=%.3fNm on %s\n",
              cmd.angle_0p001mm * 0.001, cmd.effort_0p001Nm * 0.001, opt.can.c_str());

  void * ctx = zmq_ctx_new();
  void * sub = zmq_socket(ctx, ZMQ_SUB);
  void * pub = zmq_socket(ctx, ZMQ_PUB);
  const int64_t max_msg = 4096;
  const int linger = 0;
  zmq_setsockopt(sub, ZMQ_MAXMSGSIZE, &max_msg, sizeof(max_msg));
  zmq_setsockopt(sub, ZMQ_LINGER, &linger, sizeof(linger));
  zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "Gripper ", 8);
  if (zmq_connect(sub, opt.endpoint.c_str()) != 0) {
    std::fprintf(stderr, "[gripper] zmq connect %s failed: %s\n", opt.endpoint.c_str(), zmq_strerror(zmq_errno()));
    return 7;
  }
  if (zmq_bind(pub, opt.feedback_endpoint.c_str()) != 0) {
    std::fprintf(stderr, "[gripper] zmq bind %s failed: %s\n", opt.feedback_endpoint.c_str(), zmq_strerror(zmq_errno()));
    return 7;
  }
  std::printf("[gripper] listening on %s for %s (Ctrl+C exits; gripper keeps holding)\n",
              opt.endpoint.c_str(), opt.robot.c_str());

  auto last_send = Clock::now();
  auto last_report = Clock::now();
  bool feedback_lost = false;
  uint8_t last_status = st->status_code;
  char buf[4097];

  while (!g_stop.load()) {
    zmq_pollitem_t item{sub, 0, ZMQ_POLLIN, 0};
    if (zmq_poll(&item, 1, 20) > 0 && (item.revents & ZMQ_POLLIN)) {
      const int n = zmq_recv(sub, buf, sizeof(buf) - 1, ZMQ_DONTWAIT);
      if (n > 8 && n < static_cast<int>(sizeof(buf))) {
        buf[n] = '\0';
        const auto j = nlohmann::json::parse(buf + 8, nullptr, false);
        if (j.is_discarded() || !j.is_object()) {
          std::fprintf(stderr, "[gripper] reject: bad JSON\n");
        } else if (!j.contains(opt.robot)) {
          // Not for this gripper.
        } else {
          const auto & robot = j[opt.robot];
          const auto & ee = (robot.is_object() && robot.contains("EndEffector")) ? robot["EndEffector"] : robot;
          pg::GripperCommand next = cmd;
          const std::string err = apply_command(ee, opt.default_effort, &next);
          if (!err.empty()) {
            std::fprintf(stderr, "[gripper] reject %s: %s\n", ee.dump().c_str(), err.c_str());
          } else if (send(can_fd, next)) {
            std::printf("[gripper] cmd %s -> opening=%.2fmm effort=%.3fNm %s\n", ee.dump().c_str(),
                        next.angle_0p001mm * 0.001, next.effort_0p001Nm * 0.001, status_name(next.status));
            // One-shot statuses are not repeated by the keepalive.
            if (next.status == pg::StatusCode::EnableAndClearError) next.status = pg::StatusCode::Enable;
            if (next.status == pg::StatusCode::DisableAndClearError) next.status = pg::StatusCode::Disable;
            cmd = next;
            last_send = Clock::now();
          }
        }
      }
    }

    // Re-send the same target at 10 Hz so feedback keeps flowing.
    if (Clock::now() - last_send >= std::chrono::milliseconds(100)) {
      send(can_fd, cmd);
      last_send = Clock::now();
    }

    const auto now_state = fresh_state(client, 1.0);
    if (!now_state && !feedback_lost) {
      std::fprintf(stderr, "[gripper] WARNING: no gripper feedback for >1 s\n");
      feedback_lost = true;
    } else if (now_state) {
      if (feedback_lost) std::printf("[gripper] feedback restored\n");
      feedback_lost = false;
      if (now_state->status_code != last_status || Clock::now() - last_report >= std::chrono::seconds(5)) {
        print_state(*now_state, now_state->status_code != last_status ? "status change:" : "state:");
        last_status = now_state->status_code;
        last_report = Clock::now();
      }
    }

    // Publish normalized actual position feedback (0.0 closed, 1.0 open).
    if (now_state) {
      const double position = std::clamp(static_cast<double>(now_state->angle_0p001mm) / kAngleMax, 0.0, 1.0);
      const nlohmann::json msg = {
        {opt.robot, {
          {"position", position},
          {"angle_0p001mm", now_state->angle_0p001mm},
          {"effort", now_state->effort_0p001Nm * 0.001},
          {"feedback_valid", true}
        }}
      };
      const std::string s = "GripperFeedback " + msg.dump();
      zmq_send(pub, s.c_str(), s.size(), ZMQ_DONTWAIT);
    }
  }

  std::printf("[gripper] exiting; gripper left %s at %.2fmm\n", status_name(cmd.status), cmd.angle_0p001mm * 0.001);
  zmq_close(sub);
  zmq_close(pub);
  zmq_ctx_term(ctx);
  client.Stop();
  ::close(can_fd);
  return 0;
}
