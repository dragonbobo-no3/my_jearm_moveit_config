// MK2 waist joint feedback server (controller side). READ-ONLY by default.
//
// The waist is a JE CAN joint sharing can1 with the left gripper. Once powered
// it streams 0x103/0x104/0x105 (~200 Hz) without any request. Without
// --enable-control this process only RECEIVES those frames and publishes
//   WaistFeedback {"Waist":{"position":<rad>,...,"feedback_valid":true}}
// on a ZMQ PUB bind (default tcp://*:8005). Protocol: Control_ByCAN.md (DBC
// "-m" = big-endian). GearPos_Fbk is 524288 bit/rev.
//
// Zero: the protocol has no set-zero command, so nothing is written to the
// joint. --calibrate-zero averages the stationary GearPos_Fbk and stores it in
// the zero file; the published waist angle = (GearPos_Fbk - zero_bit). Any
// future command path must add zero_bit back. Without a zero file the server
// refuses to start.
//
// --enable-control (opt-in) adds a jog path: SUB-connects to the PC
// (default tcp://192.168.0.100:8006) for
//   WaistCmd {"seq":N,"op":"enable|jog|run|hold|disable|set_zero","delta_deg":X,"dir":+-1}
// "run" (key held) moves continuously in segments while new-seq run messages
// keep arriving (stale > 0.5 s -> hold).
// "set_zero" (disabled, or active and settled; within the current limits)
// averages 1 s of stationary feedback, rewrites the zero file and applies it
// at once (limits move with the zero). Nothing is written to the joint.
// The PC repeats its latest message at ~10 Hz as heartbeat; an op is applied
// once per new seq. Nothing is sent on CAN until an "enable" arrives.
// Motion: the firmware restarts its S-curve (GearPos_Req) from the current
// position on EVERY 0x10 frame, so streaming targets makes a 50 Hz sawtooth.
// Each jog therefore sends ONE target (<= max-step away) and nothing more
// until the move has settled; jogs during a move are ignored. At rest the
// target is repeated at 10 Hz as keepalive (harmless when settled).
// Safety: target clamped to zero +-limit, Req-vs-feedback tracking, move
// envelope / timeout / mode / fault / stale-feedback checks -> disable
// (mode 0; vertical axis + brake).
// Enable targets the nearest byte-palindromic TargetValue (byte order cannot
// matter, <=1.41 deg away), then a 1-LSB step must be echoed big-endian in
// GearPos_Req before any jog is accepted. PC heartbeat loss -> hold, then
// release when stationary.
//
// Build (controller):
//   g++ -std=c++17 -O2 -Wall -Wextra -I/home/robot/mk2-gripper mk2_waist_server.cpp
//       -lzmq -pthread -o mk2_waist_server

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
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
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>
#include <thread>

#include <zmq.h>

#include "nlohmann/json.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kBitsPerRev = 524288.0;
constexpr double kGearRatio = 101.0;  // H20, motor rpm -> output
constexpr double kPi = 3.14159265358979323846;

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

struct Options {
  std::string can = "can1";
  std::string feedback_endpoint = "tcp://*:8005";
  std::string key = "Waist";
  std::string zero_file = "/home/robot/mk2-gripper/waist_zero.json";
  int rate_hz = 50;
  bool probe = false;
  bool calibrate = false;
  // Control (opt-in).
  bool enable_control = false;
  std::string cmd_endpoint = "tcp://192.168.0.100:8006";
  double limit_deg = 30.0;      // target clamp around zero
  double max_step_deg = 5.0;    // largest single jog
  double run_span_deg = 10.0;   // target distance per segment while a key is held
  double lag_deg = 1.0;         // |GearPos_Req - feedback| above this -> disable
  int fw_speed_raw = 8;         // 0x10 speedlimit x0.001 OUTPUT r/s (measured: 87 -> 31 deg/s) -> 2.9 deg/s
  int acc_raw = 1;              // 0x10 AccMax x0.05 OUTPUT r/s^2 (measured: 20 -> ~360 deg/s^2) -> 18 deg/s^2
  int iq_limit = 10;            // 0x10 Iqlimit, A
};

void usage() {
  std::fprintf(stderr,
    "usage: mk2_waist_server [--can can1] [--feedback-endpoint tcp://*:8005]\n"
    "                        [--key Waist] [--rate 50] [--zero-file PATH]\n"
    "                        [--probe | --calibrate-zero]\n"
    "                        [--enable-control [--cmd-endpoint tcp://192.168.0.100:8006]\n"
    "                         [--limit-deg 30] [--max-step-deg 5] [--lag-deg 1]\n"
    "                         [--fw-speed-raw 8] [--acc-raw 1] [--iq-limit 10]]\n"
    "  Without --enable-control: READ-ONLY, never transmits on CAN.\n"
    "  --probe           print one decoded feedback set and exit\n"
    "  --calibrate-zero  store the current (stationary) position as waist zero and exit\n");
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
    if (a == "--feedback-endpoint" && next(&o->feedback_endpoint)) continue;
    if (a == "--key" && next(&o->key)) continue;
    if (a == "--zero-file" && next(&o->zero_file)) continue;
    if (a == "--rate" && next(&v)) {
      try { o->rate_hz = std::stoi(v); } catch (...) { return false; }
      if (o->rate_hz < 1 || o->rate_hz > 200) return false;
      continue;
    }
    auto num = [&](double lo, double hi, double * out) {
      if (!next(&v)) return false;
      try { *out = std::stod(v); } catch (...) { return false; }
      return std::isfinite(*out) && *out >= lo && *out <= hi;
    };
    auto inum = [&](int lo, int hi, int * out) {
      double d = 0.0;
      if (!num(lo, hi, &d) || d != std::floor(d)) return false;
      *out = static_cast<int>(d);
      return true;
    };
    if (a == "--probe") { o->probe = true; continue; }
    if (a == "--calibrate-zero") { o->calibrate = true; continue; }
    if (a == "--enable-control") { o->enable_control = true; continue; }
    if (a == "--cmd-endpoint" && next(&o->cmd_endpoint)) continue;
    if (a == "--limit-deg") { if (num(0.5, 170.0, &o->limit_deg)) continue; return false; }
    if (a == "--max-step-deg") { if (num(0.1, 10.0, &o->max_step_deg)) continue; return false; }
    if (a == "--lag-deg") { if (num(0.2, 5.0, &o->lag_deg)) continue; return false; }
    if (a == "--fw-speed-raw") { if (inum(1, 28, &o->fw_speed_raw)) continue; return false; }  // <= 10 deg/s
    if (a == "--acc-raw") { if (inum(1, 4, &o->acc_raw)) continue; return false; }  // <= 72 deg/s^2
    if (a == "--iq-limit") { if (inum(1, 255, &o->iq_limit)) continue; return false; }
    return false;
  }
  return o->can.rfind("can", 0) == 0 && o->feedback_endpoint.rfind("tcp://", 0) == 0 && !o->key.empty() &&
         !o->zero_file.empty() && !(o->probe && o->calibrate) && o->cmd_endpoint.rfind("tcp://", 0) == 0;
}

// Raw CAN socket on ifname. RX socket: kernel filter passes 0x103/0x104/0x105.
// TX socket (rx=false): receives nothing.
int open_can(const std::string & ifname, bool rx) {
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) return -1;
  can_filter filters[3];
  const canid_t ids[3] = {0x103, 0x104, 0x105};
  for (int i = 0; i < 3; ++i) {
    filters[i].can_id = ids[i];
    filters[i].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  }
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, rx ? filters : nullptr, rx ? sizeof(filters) : 0);
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

uint32_t be(const uint8_t * p, int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; ++i) v = (v << 8) | p[i];
  return v;
}

struct Waist {
  // 0x104
  bool have104 = false;
  Clock::time_point t104{};
  int32_t req_bit = 0;
  int32_t fbk_bit = 0;
  double motor_rpm = 0.0;
  // 0x103
  bool have103 = false;
  Clock::time_point t103{};
  int mcu_dts_temp = 0;
  int mos_temp = 0;
  uint16_t fault = 0;
  int control_mode = 0;
  int motor_state = 0;
  double udc = 0.0;
  double iq = 0.0;
  // 0x105
  bool have105 = false;
  int motor_temp = 0;
  int force = 0;
  int32_t pos_err = 0;
  int mcu_ntc_temp = 0;
  uint64_t frames = 0;
};

void decode(const can_frame & f, Waist * w) {
  if (f.can_dlc < 8) return;
  const uint8_t * d = f.data;
  ++w->frames;
  switch (f.can_id & CAN_SFF_MASK) {
    case 0x103:
      w->mcu_dts_temp = d[0] - 60;
      w->mos_temp = d[1] - 60;
      w->fault = static_cast<uint16_t>(be(d + 2, 2));
      w->control_mode = d[4] >> 5;
      w->motor_state = (d[4] >> 2) & 0x7;
      w->udc = static_cast<double>(((d[4] & 0x3) << 8) | d[5]) * 0.1;
      w->iq = be(d + 6, 2) * 0.005 - 150.0;
      w->have103 = true;
      w->t103 = Clock::now();
      break;
    case 0x104:
      w->req_bit = static_cast<int32_t>(be(d, 3)) - 1000000;
      w->fbk_bit = static_cast<int32_t>(be(d + 3, 3)) - 1000000;
      w->motor_rpm = be(d + 6, 2) * 0.5 - 16384.0;
      w->have104 = true;
      w->t104 = Clock::now();
      break;
    case 0x105:
      w->motor_temp = d[0] - 50;
      w->force = static_cast<int>(be(d + 1, 2)) - 32768;
      w->pos_err = static_cast<int32_t>(be(d + 3, 3)) - 1000000;
      w->mcu_ntc_temp = d[6] - 60;
      w->have105 = true;
      break;
    default:
      break;
  }
}

double bit_to_rad(int32_t bit) { return bit / kBitsPerRev * 2.0 * kPi; }

constexpr double kBitsPerDeg = kBitsPerRev / 360.0;
constexpr double kStaleS = 0.3;  // 0x104 arrives in bursts (gaps up to ~95 ms measured)

// 0x10 TargetValue: 16 bit, bit = raw * 16 - 524288 (+-360 deg).
int bit_to_raw(double bit) {
  return std::clamp(static_cast<int>(std::lround((bit + 524288.0) / 16.0)), 0, 65535);
}
int32_t raw_to_bit(int raw) { return raw * 16 - 524288; }

can_frame make_cmd(int raw, int mode, const Options & o) {
  can_frame f{};
  f.can_id = 0x10;
  f.can_dlc = 8;
  f.data[0] = static_cast<uint8_t>(raw >> 8);  // big-endian (DBC "-m"); verified at enable
  f.data[1] = static_cast<uint8_t>(raw & 0xFF);
  const int brake = mode == 3 ? 1 : 0;           // 1 = release (doc: not yet implemented)
  f.data[2] = static_cast<uint8_t>(((mode & 0x7) << 5) | (brake << 4) | ((o.fw_speed_raw >> 8) & 0xF));
  f.data[3] = static_cast<uint8_t>(o.fw_speed_raw & 0xFF);
  f.data[4] = 128;                               // ReqIq = 0 A
  f.data[5] = static_cast<uint8_t>(o.iq_limit);
  f.data[6] = static_cast<uint8_t>((o.acc_raw >> 8) & 0x3);  // Profile 0, MotorID 0
  f.data[7] = static_cast<uint8_t>(o.acc_raw & 0xFF);
  return f;
}

enum class Ctl { Disabled, Enabling, Checking, Active, Stopping };
const char * ctl_name(Ctl s) {
  switch (s) {
    case Ctl::Disabled: return "disabled";
    case Ctl::Enabling: return "enabling";
    case Ctl::Checking: return "checking";
    case Ctl::Active: return "active";
    case Ctl::Stopping: return "stopping";
  }
  return "?";
}

struct Control {
  Ctl state = Ctl::Disabled;
  Clock::time_point t_state{};
  int sent_raw = 0;    // current target
  Clock::time_point t_tx{};
  bool moving = false;  // target sent, not yet settled
  bool need_tx = false;
  bool accepted = false;
  int tx_tries = 0;
  int32_t req_at_tx = 0;
  int run_dir = 0;      // held key: +-1
  bool running = false;
  bool aborting = false;  // soft stop in position mode before release
  Clock::time_point t_run{};
  bool hold_sent = false;
  int32_t move_from = 0;
  Clock::time_point t_move{};
  int enable_ticks = 0;
  int pal_raw = 0;
  bool req_ok = false;
  int req_bad = 0;
  int mode_bad = 0;
  int echo_ticks = 0;
  int disable_frames = 0;
  long long last_seq = -1;
  bool have_pc = false;
  Clock::time_point t_pc{};
  bool pc_lost_logged = false;
  // set_zero sampling
  bool zeroing = false;
  Clock::time_point t_zero{};
  Clock::time_point z_t104{};
  long long z_sum = 0;
  int z_n = 0;
  int32_t z_lo = 0, z_hi = 0;
  double z_max_rpm = 0.0;
  std::string error;
  std::string note;
};

constexpr int32_t kRawLimit = 1000000;  // 24-bit field with offset -1000000

bool load_zero(const std::string & path, int32_t * zero) {
  std::ifstream in(path);
  if (!in) return false;
  const auto j = nlohmann::json::parse(in, nullptr, false);
  if (j.is_discarded() || !j.is_object() || !j.contains("zero_bit") || !j["zero_bit"].is_number_integer())
    return false;
  const auto v = j["zero_bit"].get<long long>();
  if (v < -kRawLimit || v > kRawLimit) return false;
  *zero = static_cast<int32_t>(v);
  return true;
}

bool save_zero(const std::string & path, int32_t zero, const nlohmann::json & info) {
  nlohmann::json j = info;
  j["zero_bit"] = zero;
  j["zero_deg_protocol"] = zero / kBitsPerRev * 360.0;
  const std::time_t t = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S%z", std::localtime(&t));
  j["calibrated_at"] = stamp;
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) return false;
    out << j.dump(2) << "\n";
    if (!out.good()) return false;
  }
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

nlohmann::json to_json(const Waist & w, int32_t zero, double age_s) {
  const int32_t rel = w.fbk_bit - zero;
  return {
    {"position", bit_to_rad(rel)},
    {"position_deg", rel / kBitsPerRev * 360.0},
    {"position_bit", w.fbk_bit},
    {"zero_bit", zero},
    {"request_bit", w.req_bit},
    {"velocity", w.motor_rpm / kGearRatio * 2.0 * kPi / 60.0},
    {"motor_rpm", w.motor_rpm},
    {"position_err_bit", w.pos_err},
    {"fault", w.fault},
    {"control_mode", w.control_mode},
    {"motor_state", w.motor_state},
    {"udc", w.udc},
    {"iq", w.iq},
    {"temp_motor", w.motor_temp},
    {"temp_mos", w.mos_temp},
    {"temp_mcu", w.mcu_dts_temp},
    {"age", age_s},
    {"feedback_valid", true},
  };
}

void print_state(const Waist & w, int32_t zero, const char * tag) {
  std::printf("[waist] %s pos=%.3fdeg (raw %d bit, zero %d) req=%d bit err=%d bit motor=%.1frpm "
              "fault=0x%04x mode=%d state=%d udc=%.1fV iq=%.3fA temp m/mos/mcu=%d/%d/%d frames=%llu\n",
              tag, (w.fbk_bit - zero) / kBitsPerRev * 360.0, w.fbk_bit, zero, w.req_bit, w.pos_err, w.motor_rpm,
              w.fault, w.control_mode, w.motor_state, w.udc, w.iq, w.motor_temp, w.mos_temp,
              w.mcu_dts_temp, static_cast<unsigned long long>(w.frames));
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

  const std::string lock_path = "/tmp/mk2_waist_server." + opt.can + ".lock";
  const int lock_fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
  if (lock_fd < 0 || ::flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
    std::fprintf(stderr, "[waist] another mk2_waist_server owns %s\n", opt.can.c_str());
    return 3;
  }
  const int can_fd = open_can(opt.can, true);
  if (can_fd < 0) {
    std::fprintf(stderr, "[waist] CAN RX socket on %s failed: %s\n", opt.can.c_str(), std::strerror(errno));
    return 4;
  }

  Waist w;
  auto read_frames = [&](int timeout_ms) {
    pollfd p{can_fd, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return;
    can_frame f{};
    while (::recv(can_fd, &f, sizeof(f), MSG_DONTWAIT) == static_cast<ssize_t>(sizeof(f))) decode(f, &w);
  };

  const auto deadline = Clock::now() + std::chrono::milliseconds(1500);
  while (!g_stop.load() && Clock::now() < deadline && !(w.have103 && w.have104 && w.have105)) read_frames(50);
  if (!(w.have103 && w.have104)) {
    std::fprintf(stderr, "[waist] no feedback (0x103/0x104) on %s; check waist power (wait 10 s) and wiring.\n",
                 opt.can.c_str());
    return 5;
  }
  print_state(w, 0, "first feedback (raw, zero not applied):");
  if (opt.probe) return 0;

  if (opt.calibrate) {
    // Average 1 s of feedback; the waist must be still and fault-free.
    long long sum = 0;
    int n = 0;
    int32_t lo = w.fbk_bit, hi = w.fbk_bit;
    double max_rpm = 0.0;
    uint64_t seen = w.frames;
    const auto end = Clock::now() + std::chrono::seconds(1);
    while (!g_stop.load() && Clock::now() < end) {
      read_frames(5);
      if (w.frames == seen) continue;
      seen = w.frames;
      sum += w.fbk_bit;
      ++n;
      lo = std::min(lo, w.fbk_bit);
      hi = std::max(hi, w.fbk_bit);
      max_rpm = std::max(max_rpm, std::fabs(w.motor_rpm));
    }
    if (n < 50 || hi - lo > 50 || max_rpm > 5.0 || w.fault != 0) {
      std::fprintf(stderr, "[waist] calibration refused: samples=%d spread=%d bit max|motor|=%.1frpm fault=0x%04x "
                   "(need >=50 samples, spread<=50 bit, |motor|<=5rpm, fault=0)\n", n, hi - lo, max_rpm, w.fault);
      return 8;
    }
    const int32_t zero = static_cast<int32_t>(std::llround(static_cast<double>(sum) / n));
    int32_t old = 0;
    const bool had_old = load_zero(opt.zero_file, &old);
    const nlohmann::json info = {{"can", opt.can}, {"samples", n}, {"spread_bit", hi - lo},
                                 {"bits_per_rev", kBitsPerRev},
                                 {"previous_zero_bit", had_old ? nlohmann::json(old) : nlohmann::json()}};
    if (!save_zero(opt.zero_file, zero, info)) {
      std::fprintf(stderr, "[waist] cannot write %s: %s\n", opt.zero_file.c_str(), std::strerror(errno));
      return 9;
    }
    std::printf("[waist] zero calibrated: zero_bit=%d (protocol %.3fdeg) samples=%d spread=%d bit -> %s\n",
                zero, zero / kBitsPerRev * 360.0, n, hi - lo, opt.zero_file.c_str());
    if (had_old) std::printf("[waist] previous zero_bit=%d (delta %.3fdeg)\n", old, (zero - old) / kBitsPerRev * 360.0);
    std::printf("[waist] restart mk2_waist_server to apply\n");
    return 0;
  }

  int32_t zero = 0;
  if (!load_zero(opt.zero_file, &zero)) {
    std::fprintf(stderr, "[waist] no valid zero file %s; run --calibrate-zero with the waist at its zero pose\n",
                 opt.zero_file.c_str());
    return 10;
  }
  print_state(w, zero, "zero applied:");

  void * ctx = zmq_ctx_new();
  void * pub = zmq_socket(ctx, ZMQ_PUB);
  const int linger = 0;
  zmq_setsockopt(pub, ZMQ_LINGER, &linger, sizeof(linger));
  if (zmq_bind(pub, opt.feedback_endpoint.c_str()) != 0) {
    std::fprintf(stderr, "[waist] zmq bind %s failed: %s\n", opt.feedback_endpoint.c_str(), zmq_strerror(zmq_errno()));
    return 7;
  }
  std::printf("[waist] %s on %s, publishing WaistFeedback on %s at %d Hz\n",
              opt.enable_control ? "CONTROL AVAILABLE (passive until PC enables)" : "READ-ONLY",
              opt.can.c_str(), opt.feedback_endpoint.c_str(), opt.rate_hz);

  // ---- Control (opt-in) ----
  int tx_fd = -1;
  void * sub = nullptr;
  if (opt.enable_control) {
    tx_fd = open_can(opt.can, false);
    if (tx_fd < 0) {
      std::fprintf(stderr, "[waist] CAN TX socket on %s failed: %s\n", opt.can.c_str(), std::strerror(errno));
      return 11;
    }
    sub = zmq_socket(ctx, ZMQ_SUB);
    zmq_setsockopt(sub, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "WaistCmd ", 9);
    if (zmq_connect(sub, opt.cmd_endpoint.c_str()) != 0) {
      std::fprintf(stderr, "[waist] zmq connect %s failed: %s\n", opt.cmd_endpoint.c_str(), zmq_strerror(zmq_errno()));
      return 12;
    }
    std::printf("[waist] commands from %s: limit +-%.1fdeg, max step %.1fdeg, lag %.1fdeg, "
                "fw speed raw %d, acc raw %d, iq limit %d\n", opt.cmd_endpoint.c_str(), opt.limit_deg,
                opt.max_step_deg, opt.lag_deg, opt.fw_speed_raw, opt.acc_raw, opt.iq_limit);
  }
  double lim_lo = zero - opt.limit_deg * kBitsPerDeg;
  double lim_hi = zero + opt.limit_deg * kBitsPerDeg;
  auto secs = [](Clock::duration d) { return std::chrono::duration<double>(d).count(); };
  Control c;
  unsigned long long tx_frames = 0, tx_errors = 0;
  auto send = [&](int raw, int mode) {
    const can_frame f = make_cmd(raw, mode, opt);
    if (::write(tx_fd, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f))) ++tx_frames; else ++tx_errors;
  };
  auto enter = [&](Ctl s, Clock::time_point now) {
    if (s != c.state) std::printf("[waist] control: %s -> %s\n", ctl_name(c.state), ctl_name(s));
    c.state = s;
    c.t_state = now;
  };
  auto fail = [&](Clock::time_point now, const std::string & why) {
    c.error = why;
    std::fprintf(stderr, "[waist] CONTROL DISABLED: %s\n", why.c_str());
    print_state(w, zero, "at disable:");
    enter(Ctl::Disabled, now);
    c.disable_frames = 10;
  };
  // One target per move; the firmware S-curve does the rest. Sent from the
  // control tick (>=40 ms after the previous frame) and re-sent if Req does
  // not react (frames get lost on the slcan adapter / ignored by the drive).
  auto start_move = [&](int raw, Clock::time_point now) {
    c.sent_raw = raw;
    c.move_from = w.fbk_bit;
    c.t_move = now;
    c.moving = true;
    c.need_tx = true;
    c.accepted = false;
    c.tx_tries = 0;
  };
  // Stop a running move once: new target = where the planner is now.
  auto hold = [&](Clock::time_point now) {
    c.run_dir = 0;
    c.running = false;
    if (!c.moving || c.hold_sent) return;
    c.hold_sent = true;
    start_move(bit_to_raw(w.req_bit), now);
    c.note = "hold";
  };
  // Motion check failed while the drive is healthy: stop at the current
  // position in position mode (mode 0 would let the joint coast), then release.
  auto abort = [&](Clock::time_point now, const std::string & why) {
    if (c.aborting) return fail(now, why);
    c.error = why;
    std::fprintf(stderr, "[waist] CONTROL STOPPING: %s\n", why.c_str());
    print_state(w, zero, "at stop:");
    c.run_dir = 0;
    c.running = false;
    c.hold_sent = true;
    c.aborting = true;
    start_move(bit_to_raw(w.fbk_bit), now);
    enter(Ctl::Stopping, now);
  };

  auto handle_cmd = [&](const std::string & body, Clock::time_point now) {
    const auto j = nlohmann::json::parse(body, nullptr, false);
    if (j.is_discarded() || !j.is_object() || !j.contains("seq") || !j["seq"].is_number_integer() ||
        !j.contains("op") || !j["op"].is_string())
      return;
    c.have_pc = true;
    c.t_pc = now;
    const long long seq = j["seq"].get<long long>();
    if (seq == c.last_seq) return;  // heartbeat repeat
    c.last_seq = seq;
    const std::string op = j["op"].get<std::string>();
    if (c.zeroing && op != "hold" && op != "disable") {
      c.note = "busy: setting zero";
      return;
    }
    if (op == "set_zero") {
      const bool settled = c.state == Ctl::Disabled ||
                           (c.state == Ctl::Active && !c.moving && !c.need_tx && c.run_dir == 0);
      const bool fresh = secs(now - w.t104) < kStaleS;
      if (!settled || !fresh || w.fault != 0 || std::fabs(w.motor_rpm) > 2.0) {
        c.error = "set zero refused: need disabled or settled control, fresh feedback, fault 0, stationary joint";
        std::fprintf(stderr, "[waist] %s\n", c.error.c_str());
        return;
      }
      if (std::abs(w.fbk_bit - zero) > opt.limit_deg * kBitsPerDeg) {
        c.error = "set zero refused: joint is outside the current limits";
        std::fprintf(stderr, "[waist] %s\n", c.error.c_str());
        return;
      }
      c.zeroing = true;
      c.t_zero = now;
      c.z_t104 = w.t104;
      c.z_sum = 0;
      c.z_n = 0;
      c.z_lo = c.z_hi = w.fbk_bit;
      c.z_max_rpm = 0.0;
      c.error.clear();
      c.note = "setting zero: keep the waist still (1 s)";
      std::printf("[waist] set zero requested at %.3fdeg\n", (w.fbk_bit - zero) / kBitsPerDeg);
      return;
    }
    if (op == "enable") {
      if (c.state != Ctl::Disabled) return;
      const bool fresh = secs(now - w.t104) < kStaleS;
      if (!fresh || w.fault != 0 || std::fabs(w.motor_rpm) > 5.0) {
        c.error = "enable refused: need fresh feedback, fault 0 and a stationary joint";
        std::fprintf(stderr, "[waist] %s\n", c.error.c_str());
        return;
      }
      // Nearest byte-palindromic TargetValue: identical in either byte order.
      int best = -1;
      double best_d = 1e18;
      for (int b = 0; b < 256; ++b) {
        const int r = b * 257;
        const double bit = raw_to_bit(r);
        if (bit < lim_lo || bit > lim_hi) continue;
        const double d = std::fabs(bit - w.fbk_bit);
        if (d < best_d) { best_d = d; best = r; }
      }
      if (best < 0 || best_d > 1.5 * kBitsPerDeg) {
        c.error = "enable refused: no byte-symmetric target within 1.5deg inside the limits";
        std::fprintf(stderr, "[waist] %s\n", c.error.c_str());
        return;
      }
      c.pal_raw = c.sent_raw = best;
      c.moving = c.hold_sent = c.req_ok = c.need_tx = c.accepted = c.running = c.aborting = false;
      c.run_dir = 0;
      c.req_bad = c.mode_bad = c.echo_ticks = c.enable_ticks = 0;
      c.pc_lost_logged = false;
      c.error.clear();
      c.note = "enabling";
      std::printf("[waist] enable requested: hold target raw 0x%04X = %.3fdeg (feedback %.3fdeg)\n", best,
                  (raw_to_bit(best) - zero) / kBitsPerDeg, (w.fbk_bit - zero) / kBitsPerDeg);
      enter(Ctl::Enabling, now);
    } else if (op == "jog") {
      if (c.state != Ctl::Active || !j.contains("delta_deg") || !j["delta_deg"].is_number()) return;
      const double d = j["delta_deg"].get<double>();
      if (!std::isfinite(d) || std::fabs(d) > opt.max_step_deg + 1e-9) {
        c.note = "jog refused: step larger than max step";
        return;
      }
      if (c.moving) {
        c.note = "busy: jog ignored until the current step has settled";
        return;
      }
      const double tgt = std::clamp(raw_to_bit(c.sent_raw) + d * kBitsPerDeg, lim_lo, lim_hi);
      const int raw = bit_to_raw(tgt);
      if (raw == c.sent_raw) {
        c.note = "at limit";
        return;
      }
      c.hold_sent = false;
      start_move(raw, now);
      c.note = "moving";
    } else if (op == "run") {
      // Key held: move continuously while new-seq "run" messages keep coming.
      if (c.state != Ctl::Active || !j.contains("dir") || !j["dir"].is_number_integer()) return;
      const int dir = j["dir"].get<int>();
      if (dir != 1 && dir != -1) return;
      if (c.running && c.run_dir != dir) {
        hold(now);
        return;
      }
      c.run_dir = dir;
      c.t_run = now;
    } else if (op == "hold") {
      if (c.state == Ctl::Active) hold(now);
    } else if (op == "disable") {
      if (c.state == Ctl::Active) {
        hold(now);
        enter(Ctl::Stopping, now);
      } else if (c.state == Ctl::Enabling || c.state == Ctl::Checking) {
        fail(now, "disabled by PC during enable");
      }
    }
  };

  auto control_tick = [&](Clock::time_point now) {
    if (c.state == Ctl::Disabled) {
      if (c.disable_frames > 0) {
        send(bit_to_raw(w.fbk_bit), 0);
        --c.disable_frames;
      }
      return;
    }
    if (secs(now - w.t104) > kStaleS) {
      char b[64];
      std::snprintf(b, sizeof(b), "feedback stale (%.0f ms)", secs(now - w.t104) * 1000.0);
      return fail(now, b);
    }
    if (w.fault != 0) {
      char b[64];
      std::snprintf(b, sizeof(b), "joint fault 0x%04x", w.fault);
      return fail(now, b);
    }
    const double t = secs(now - c.t_state);
    const double pc_age = c.have_pc ? secs(now - c.t_pc) : 1e9;
    const double rpm = std::fabs(w.motor_rpm);

    if (c.state == Ctl::Enabling) {
      if (pc_age > 0.5) return fail(now, "PC heartbeat lost during enable");
      const int32_t pal_bit = raw_to_bit(c.pal_raw);
      // GearPos_Req is the firmware S-curve setpoint: it walks to the target.
      // Every frame restarts it, so after 3 frames resend only when the mode
      // has not switched yet or Req has arrived (keepalive, 10 Hz).
      if (std::abs(w.req_bit - pal_bit) <= 16) c.req_ok = true;
      if (c.enable_ticks < 3 || w.control_mode != 3 ||
          (std::abs(w.req_bit - pal_bit) <= 16 && secs(now - c.t_tx) >= 0.1)) {
        send(c.pal_raw, 3);
        c.t_tx = now;
      }
      ++c.enable_ticks;
      if (std::abs(w.fbk_bit - pal_bit) > 2.0 * kBitsPerDeg) return fail(now, "unexpected motion during enable");
      if (c.req_ok && w.control_mode == 3 && std::abs(w.fbk_bit - pal_bit) < 0.2 * kBitsPerDeg && rpm < 5.0) {
        // 1-LSB step towards zero: big-endian = +-16 bit, little-endian would be +-4096.
        c.sent_raw = c.pal_raw + (pal_bit < zero ? 1 : -1);
        c.echo_ticks = 0;
        enter(Ctl::Checking, now);
      } else if (t > 10.0) {
        return fail(now, c.req_ok ? "did not settle on the enable target"
                                  : "GearPos_Req did not reach the enable target in 10 s");
      }
      return;
    }

    if (c.state == Ctl::Checking) {
      if (pc_age > 0.5) return fail(now, "PC heartbeat lost during byte-order check");
      send(c.sent_raw, 3);
      const int32_t be_bit = raw_to_bit(c.sent_raw);
      const int32_t pal_bit = raw_to_bit(c.pal_raw);
      if (std::abs(w.fbk_bit - be_bit) > 0.5 * kBitsPerDeg) return fail(now, "unexpected motion during byte-order check");
      // Req must walk pal -> pal+-16 and stay there; a little-endian target
      // (+-4096) would carry it past.
      if (std::abs(w.req_bit - be_bit) > 32 && std::abs(w.req_bit - pal_bit) > 32)
        return fail(now, "byte-order check: GearPos_Req left the 1-LSB step (little-endian?); aborting");
      if (std::abs(w.req_bit - be_bit) <= 8) {
        if (++c.echo_ticks >= 15) {
          c.moving = false;
          c.note = "byte order verified (big-endian)";
          std::printf("[waist] %s; jog accepted\n", c.note.c_str());
          enter(Ctl::Active, now);
        }
      } else {
        c.echo_ticks = 0;
      }
      if (c.state == Ctl::Checking && t > 3.0) return fail(now, "byte-order check: GearPos_Req did not echo");
      return;
    }

    // Active / Stopping: one frame per move (each frame restarts the firmware
    // S-curve); keepalive every 0.2 s only when fully settled.
    if (pc_age > 0.5) {
      hold(now);
      if (!c.pc_lost_logged) std::fprintf(stderr, "[waist] PC heartbeat lost: holding\n");
      c.pc_lost_logged = true;
      if (c.state == Ctl::Active && pc_age > 1.0) enter(Ctl::Stopping, now);
    } else {
      c.pc_lost_logged = false;
    }
    if (c.run_dir != 0 && secs(now - c.t_run) > 0.5) {
      std::fprintf(stderr, "[waist] run not refreshed: holding\n");
      hold(now);
    }
    if (c.state == Ctl::Active && c.run_dir != 0 && !c.moving) {
      const double end = c.run_dir > 0 ? lim_hi : lim_lo;
      const double span = opt.run_span_deg * kBitsPerDeg;
      const int raw = bit_to_raw(std::clamp(static_cast<double>(w.fbk_bit) + c.run_dir * span,
                                            std::min(end, static_cast<double>(w.fbk_bit)),
                                            std::max(end, static_cast<double>(w.fbk_bit))));
      if (raw != c.sent_raw && std::abs(raw_to_bit(raw) - w.fbk_bit) > 16) {
        c.hold_sent = false;
        start_move(raw, now);
        c.running = true;
        c.note = c.run_dir > 0 ? "running +" : "running -";
      } else {
        c.run_dir = 0;
        c.note = "at limit";
      }
    }
    const int32_t tgt = raw_to_bit(c.sent_raw);
    if (c.need_tx && secs(now - c.t_tx) >= 0.04) {
      send(c.sent_raw, 3);
      c.t_tx = now;
      c.req_at_tx = w.req_bit;
      c.need_tx = false;
      ++c.tx_tries;
    }
    if (c.moving && !c.need_tx && !c.accepted && secs(w.t104 - c.t_tx) > 0.02 &&
        (std::abs(w.req_bit - tgt) <= 16 || std::abs(w.req_bit - tgt) < std::abs(c.req_at_tx - tgt) - 16))
      c.accepted = true;
    if (c.moving && !c.need_tx && !c.accepted && secs(now - c.t_tx) > 0.12 && secs(w.t104 - c.t_tx) > 0.05) {
      if (c.tx_tries >= 5) {
        if (!c.aborting) return abort(now, "drive ignored the target 5 times");
      } else {
        std::fprintf(stderr, "[waist] target not taken (Req %.3fdeg), resending\n", (w.req_bit - zero) / kBitsPerDeg);
        c.need_tx = true;
      }
    }
    if (c.moving && c.accepted && std::abs(w.req_bit - tgt) <= 16 && std::abs(w.fbk_bit - tgt) < 0.1 * kBitsPerDeg) {
      c.moving = false;
      c.running = false;
      if (c.run_dir == 0) c.note = "at target";
    }
    if (!c.moving && !c.need_tx && secs(now - c.t_tx) >= 0.2 && std::abs(w.fbk_bit - tgt) < 0.02 * kBitsPerDeg &&
        rpm < 2.0) {
      send(c.sent_raw, 3);
      c.t_tx = now;
    }
    c.mode_bad = (w.control_mode != 3 || w.motor_state != 4) ? c.mode_bad + 1 : 0;
    if (c.mode_bad >= 10) return fail(now, "joint left position mode / running state");
    if (c.aborting) {
      if ((rpm < 5.0 && t > 0.2) || t > 2.0) {
        std::printf("[waist] stopped at %.3fdeg, releasing\n", (w.fbk_bit - zero) / kBitsPerDeg);
        fail(now, c.error);
      }
      return;
    }
    const double lag = opt.lag_deg * kBitsPerDeg;
    c.req_bad = std::fabs(static_cast<double>(w.req_bit) - w.fbk_bit) > lag ? c.req_bad + 1 : 0;
    if (c.req_bad >= 3) {
      char b[96];
      std::snprintf(b, sizeof(b), "tracking error %.2fdeg > %.1fdeg (GearPos_Req vs feedback)",
                    (w.req_bit - w.fbk_bit) / kBitsPerDeg, opt.lag_deg);
      return abort(now, b);
    }
    const double margin = 0.5 * kBitsPerDeg;
    const double env_lo = (c.moving ? std::min(c.move_from, tgt) : tgt) - margin;
    const double env_hi = (c.moving ? std::max(c.move_from, tgt) : tgt) + margin;
    if (w.fbk_bit < env_lo || w.fbk_bit > env_hi) {
      char b[96];
      std::snprintf(b, sizeof(b), "position %.2fdeg left the move envelope (target %.2fdeg)",
                    (w.fbk_bit - zero) / kBitsPerDeg, (tgt - zero) / kBitsPerDeg);
      return abort(now, b);
    }
    if (c.moving && secs(now - c.t_move) > 10.0) return abort(now, "move did not settle within 10 s");
    if (c.state == Ctl::Stopping && (!c.moving || t > 5.0)) {
      c.note = "released";
      std::printf("[waist] released at %.3fdeg\n", (w.fbk_bit - zero) / kBitsPerDeg);
      enter(Ctl::Disabled, now);
      c.disable_frames = 10;
    }
  };

  const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
  auto next_pub = Clock::now();
  const auto ctl_period = std::chrono::milliseconds(20);
  auto next_ctl = Clock::now();
  auto last_report = Clock::now();
  bool lost = false;
  uint16_t last_fault = w.fault;
  int last_mode = w.control_mode;

  // set_zero: sample 1 s of 0x104 feedback, then store and apply the new zero.
  auto zero_tick = [&](Clock::time_point now) {
    if (!c.zeroing) return;
    if (w.t104 != c.z_t104) {
      c.z_t104 = w.t104;
      c.z_sum += w.fbk_bit;
      ++c.z_n;
      c.z_lo = std::min(c.z_lo, w.fbk_bit);
      c.z_hi = std::max(c.z_hi, w.fbk_bit);
      c.z_max_rpm = std::max(c.z_max_rpm, std::fabs(w.motor_rpm));
    }
    const bool settled = c.state == Ctl::Disabled ||
                         (c.state == Ctl::Active && !c.moving && !c.need_tx && c.run_dir == 0);
    if (settled && secs(now - c.t_zero) < 1.0) return;
    c.zeroing = false;
    if (!settled || c.z_n < 30 || c.z_hi - c.z_lo > 50 || c.z_max_rpm > 5.0 || w.fault != 0) {
      char b[160];
      std::snprintf(b, sizeof(b), "set zero refused: %s samples=%d spread=%d bit max|motor|=%.1frpm fault=0x%04x",
                    settled ? "not still:" : "control changed:", c.z_n, c.z_hi - c.z_lo, c.z_max_rpm, w.fault);
      c.error = b;
      std::fprintf(stderr, "[waist] %s\n", b);
      return;
    }
    const int32_t nz = static_cast<int32_t>(std::llround(static_cast<double>(c.z_sum) / c.z_n));
    const nlohmann::json info = {{"can", opt.can}, {"samples", c.z_n}, {"spread_bit", c.z_hi - c.z_lo},
                                 {"bits_per_rev", kBitsPerRev}, {"previous_zero_bit", zero},
                                 {"source", "set_zero command"}};
    if (!save_zero(opt.zero_file, nz, info)) {
      c.error = "set zero failed: cannot write " + opt.zero_file;
      std::fprintf(stderr, "[waist] %s: %s\n", c.error.c_str(), std::strerror(errno));
      return;
    }
    std::printf("[waist] zero set: zero_bit %d -> %d (moved %.3fdeg), saved to %s\n", zero, nz,
                (nz - zero) / kBitsPerDeg, opt.zero_file.c_str());
    char b[96];
    std::snprintf(b, sizeof(b), "zero set here (moved %+.3fdeg)", (nz - zero) / kBitsPerDeg);
    c.note = b;
    zero = nz;
    lim_lo = zero - opt.limit_deg * kBitsPerDeg;
    lim_hi = zero + opt.limit_deg * kBitsPerDeg;
  };

  while (!g_stop.load()) {
    read_frames(2);
    const auto now = Clock::now();
    if (sub) {
      char buf[1024];
      for (;;) {
        const int n = zmq_recv(sub, buf, sizeof(buf), ZMQ_DONTWAIT);
        if (n < 0) break;
        if (n < 9 || n > static_cast<int>(sizeof(buf))) continue;  // truncated / malformed
        handle_cmd(std::string(buf + 9, buf + n), now);
      }
    }
    if (opt.enable_control && now >= next_ctl) {
      next_ctl += ctl_period;
      if (next_ctl < now) next_ctl = now + ctl_period;
      control_tick(now);
    }
    zero_tick(now);
    if (now < next_pub) continue;
    next_pub = now + period;

    const double age = std::chrono::duration<double>(now - w.t104).count();
    if (age > 0.2) {
      if (!lost) std::fprintf(stderr, "[waist] WARNING: no 0x104 feedback for %.2f s\n", age);
      lost = true;
      continue;  // Publish nothing: the PC treats silence as stale.
    }
    if (lost) std::printf("[waist] feedback restored\n");
    lost = false;

    if (w.fault != last_fault || w.control_mode != last_mode) {
      print_state(w, zero, "change:");
      last_fault = w.fault;
      last_mode = w.control_mode;
      last_report = now;
    } else if (now - last_report >= std::chrono::seconds(10)) {
      print_state(w, zero, "state:");
      last_report = now;
    }
    nlohmann::json fb = to_json(w, zero, age);
    if (opt.enable_control) {
      fb["control"] = {
        {"available", true},
        {"state", ctl_name(c.state)},
        {"goal_deg", (raw_to_bit(c.sent_raw) - zero) / kBitsPerDeg},
        {"command_deg", (w.req_bit - zero) / kBitsPerDeg},
        {"moving", c.moving},
        {"running", c.run_dir},
        {"zeroing", c.zeroing},
        {"limit_deg", opt.limit_deg},
        {"max_step_deg", opt.max_step_deg},
        {"seq", c.last_seq},
        {"pc_age", c.have_pc ? secs(now - c.t_pc) : -1.0},
        {"error", c.error},
        {"note", c.note},
        {"tx_frames", tx_frames},
        {"tx_errors", tx_errors},
      };
    } else {
      fb["control"] = {{"available", false}};
    }
    const nlohmann::json msg = {{opt.key, fb}};
    const std::string s = "WaistFeedback " + msg.dump();
    zmq_send(pub, s.c_str(), s.size(), ZMQ_DONTWAIT);
  }

  if (opt.enable_control) {
    if (c.state != Ctl::Disabled || c.disable_frames > 0) {
      for (int i = 0; i < 5; ++i) {
        send(bit_to_raw(w.fbk_bit), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      std::printf("[waist] sent disable (mode 0) on exit\n");
    }
    zmq_close(sub);
    ::close(tx_fd);
  }
  std::printf("[waist] exiting (CAN frames sent: %llu, errors %llu)\n", tx_frames, tx_errors);
  zmq_close(pub);
  zmq_ctx_term(ctx);
  ::close(can_fd);
  return 0;
}
