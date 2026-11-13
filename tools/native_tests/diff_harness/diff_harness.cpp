// 차분 하네스: 같은 시나리오를 두 스케치(기준·후보)에 돌려 기체 밖으로 나가는 것을
// 빠짐없이 기록한다. tools/diff_firmware_behaviour.py 가 기준·후보로 각각 빌드하고
// 기록을 비교한다. 계획: docs/superpowers/plans/2026-11-13-zetin-fc-modules.md B 단계.
//
// - lockstep: 가상 시계 하나. setup() 을 부르고, 그 안에서 만들어진 pid_task·udp_task 와
//   loop() 를 각자 스레드에서 한 번만 시작해 시나리오 끝까지 이어서 돌린다. 차례는
//   delay·vTaskDelay·vTaskDelayUntil 안에서 기다렸다가 돌아오는 방식으로만 넘긴다
//   (shim 의 lockstep 훅). 같은 시각이면 PID → UDP → LOOP 순서. 코드 실행에는 시간이
//   걸리지 않는다. 예외(TaskDelayExit)는 시나리오가 끝날 때만, 각 스레드 안에서.
// - 기록(모두 일어난 순서대로): 모든 ledcWrite, 모든 UDP 송신(목적지 포함), 시리얼 출력(쓴
//   순간), NVS 쓰기, PID tick 마다 상태 두 줄(T: 출력·단계, X: 추정기·유지·착지 내부), 50 tick
//   마다 보정 누산기(Y), 끝 상태(raw 링의 남은 샘플 바이트, NVS 전체). float 은 비트(16진)로.
// - 플랜트는 PID 실행 횟수가 아니라 가상 경과 시간으로 1 ms 씩 진행한다. 시나리오가 PID 를
//   늦게 깨워(지연 주입) 재동기화·짧은 catch-up tick 경로를 탄다.
// - 도달 단정: 시나리오가 의도한 분기에 실제로 갔는지. 못 갔으면 종료 코드 3.
//
// 사용: diff_harness <scenario> <log>    (diff_harness --list 로 시나리오 목록)

#include <array>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "zetin_fc.ino"

namespace {
#include "sil_flow_plant.h"
}  // namespace

namespace {

// ---------------- 기록 ----------------
FILE *g_log = nullptr;
uint64_t g_now_us = 0;
std::string g_udp_text;      // 도달 단정용: 보낸 UDP 전부
int g_nvs_puts = 0;

uint32_t bits(float v) {
  uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

std::string escape(const std::string &s) {
  std::string out;
  char buf[8];
  for (unsigned char c : s) {
    if (c >= 0x20 && c < 0x7f && c != '\\') {
      out.push_back((char)c);
    } else {
      std::snprintf(buf, sizeof buf, "\\x%02x", c);
      out += buf;
    }
  }
  return out;
}

void logLine(const char *kind, const std::string &text) {
  std::fprintf(g_log, "%s %" PRIu64 " %s\n", kind, g_now_us, text.c_str());
}

uint64_t g_plant_us = 0;          // 플랜트가 진행된 가상 시각
uint64_t g_pid_stall_us = 0;      // 다음 PID 깨움을 이만큼 늦춘다(한 번)
uint64_t g_last_pid_us = 0;

// ---------------- lockstep 스케줄러 ----------------
enum Who { CTRL = 0, PID = 1, UDP = 2, LOOP = 3, NWHO = 4 };
const char *kWhoName[NWHO] = {"CTRL", "PID", "UDP", "LOOP"};

std::mutex g_m;
std::condition_variable g_cv;
int g_turn = CTRL;
bool g_stopping = false;
thread_local int t_self = CTRL;

struct Task {
  TaskFunction_t fn = nullptr;
  bool exists = false, done = false;
  uint64_t wake_us = 0;
  std::thread th;
};
Task g_task[NWHO];

void setClock(uint64_t t) {
  g_now_us = t;
  arduino_fake::micros_value = (uint32_t)t;
  arduino_fake::millis_value = (uint32_t)(t / 1000U);
  arduino_fake::tick_count = (TickType_t)(t / 1000U);
}

// 지금 스레드의 차례를 controller 에 넘기고, 다시 차례가 올 때까지 기다린다.
void yieldToController() {
  std::unique_lock<std::mutex> lk(g_m);
  g_turn = CTRL;
  g_cv.notify_all();
  g_cv.wait(lk, [] { return g_turn == t_self; });
  if (g_stopping) throw arduino_fake::TaskDelayExit{};
}

void lsSleepUs(uint64_t us) {
  if (t_self == CTRL) {           // setup() 안의 delay: 아직 다른 태스크가 없다
    setClock(g_now_us + us);
    return;
  }
  g_task[t_self].wake_us = g_now_us + us;
  yieldToController();
}

void lsSleepUntilTick(TickType_t tick) {
  // 32비트 tick 을 지금 근처의 64비트 ms 로 펼친다.
  const uint64_t nowMs = g_now_us / 1000U;
  const int32_t delta = (int32_t)(tick - (TickType_t)nowMs);
  uint64_t targetUs = (uint64_t)((int64_t)nowMs + delta) * 1000U;
  if (t_self == CTRL) {
    if (targetUs > g_now_us) setClock(targetUs);
    return;
  }
  if (t_self == PID && g_pid_stall_us) {   // 지연 주입: 이번 깨움을 늦춘다
    targetUs += g_pid_stall_us;
    g_pid_stall_us = 0;
  }
  // 이미 지난 시각이면 기다리지 않지만 차례는 넘긴다(FreeRTOS 도 여기서 양보한다).
  g_task[t_self].wake_us = targetUs > g_now_us ? targetUs : g_now_us;
  yieldToController();
}

void lsTaskCreated(TaskFunction_t fn, const char *name) {
  const std::string n = name;
  const int who = n == "PID" ? PID : (n == "UDP" ? UDP : -1);
  if (who < 0) {
    std::fprintf(stderr, "unknown task %s\n", name);
    std::exit(2);
  }
  g_task[who].fn = fn;
  g_task[who].exists = true;
}

void workerMain(int who) {
  t_self = who;
  {
    std::unique_lock<std::mutex> lk(g_m);
    g_cv.wait(lk, [who] { return g_turn == who; });
  }
  try {
    if (!g_stopping) {
      if (who == LOOP) {
        for (;;) loop();
      } else {
        g_task[who].fn(nullptr);
      }
    }
  } catch (const arduino_fake::TaskDelayExit &) {
  }
  std::unique_lock<std::mutex> lk(g_m);
  g_task[who].done = true;
  g_turn = CTRL;
  g_cv.notify_all();
}

void grant(int who) {
  std::unique_lock<std::mutex> lk(g_m);
  g_turn = who;
  g_cv.notify_all();
  g_cv.wait(lk, [] { return g_turn == CTRL; });
}

// ---------------- 시나리오 ----------------
struct Scenario;
using Event = std::function<void(Scenario &)>;

struct Scenario {
  std::string name;
  State s;
  Params P;
  SensorModel sensor;
  bool held = true;                 // 시동 전에는 손에 든 것처럼 플랜트를 고정
  bool rc_on = false;
  uint32_t rc_seq = 0;
  float rc_roll = 0, rc_pitch = 0, rc_yaw_rate = 0;
  uint64_t start_us = 0;
  uint32_t duration_ms = 10000;
  std::function<Disturbance(uint32_t)> disturbance;
  std::function<void(uint32_t, State &)> kinematic;   // 자세 강제(보정 시나리오)
  std::multimap<uint32_t, Event> events;              // 시작 후 ms → 사건
  std::deque<std::string> inbox;                      // udp_task 가 한 반복에 하나씩
  std::function<void(Scenario &)> per_tick;           // PID tick 뒤 관찰
  std::function<std::string(Scenario &)> reach;       // 빈 문자열이면 도달
  // IMU 고장 주입
  bool imu1_freeze = false, imu2_freeze = false;
  inv_imu_sensor_event_t imu1_frozen = {}, imu2_frozen = {};
  int imu2_gyro_offset = 0;
  bool imu1_read_error = false;
  bool imu_offsets = false;         // IMU 마다 다른 gyro bias·accel scale(원값)
  uint32_t rc_off_at_tick = 0;      // 이 플랜트 tick 에 RC 를 끊는다(0 = 없음)
  uint32_t resume_at_tick = 0;      // 이 플랜트 tick 에 resume 을 보낸다(0 = 없음)
  bool mag_inject = false;
  // 관찰
  uint32_t plant_tick = 0;
  std::map<std::string, int> seen;
  void at(uint32_t ms, Event e) { events.emplace(ms, std::move(e)); }
  void send(uint32_t ms, const std::string &cmd) {
    at(ms, [cmd](Scenario &sc) { sc.inbox.push_back(cmd); });
  }
  void mark(const std::string &k) { seen[k]++; }
  bool saw(const std::string &k) const { return seen.count(k) && seen.at(k) > 0; }
  bool serialHas(const std::string &needle) const {
    return arduino_fake::serial_output.find(needle) != std::string::npos;
  }
  bool udpHas(const std::string &needle) const {
    return g_udp_text.find(needle) != std::string::npos;
  }
};

void setTrueFlowCalibration() {
  const auto Mt = trueCalibration();
  float M[2][2] = {{(float)Mt[0][0], (float)Mt[0][1]}, {(float)Mt[1][0], (float)Mt[1][1]}};
  flowOdomSetCalibration(flowOdomCfg, M, 0);
}

void recordTick(Scenario &sc) {
  const FlowHoldTelemetry t = readFlowHoldTelemetry();
  char line[512];
  std::snprintf(line, sizeof line,
                "m=%d,%d,%d,%d thr=%d,%d,%d fs=%d lock=%d probe=%d hov=%08x/%d rl=%d/%d/%d "
                "hold=%d/%d app=%08x,%08x,%08x ang=%08x,%08x,%08x i=%08x,%08x,%08x yaw=%d imus=%d "
                "f=%d%d%d%d%d",
                motorOut[0], motorOut[1], motorOut[2], motorOut[3], (int)base_throttle,
                (int)min_throttle, (int)max_throttle, (int)fs_phase, (int)safety_lock,
                (int)fs_probe_state, bits(hover_est), (int)hover_valid, (int)rl_mode,
                (int)rl_stage, (int)rl_cut_reason, (int)t.poshold_state, (int)t.althold_state,
                bits(t.applied_roll_deg), bits(t.applied_pitch_deg), bits(t.applied_alt_us),
                bits(angleX), bits(angleY), bits(angleZ), bits(iTermRoll), bits(iTermPitch),
                bits(iTermYaw), (int)yaw_authority_state, (int)active_imus, (int)fault_rc,
                (int)fault_imu1, (int)fault_imu2, (int)fault_disagree, (int)fault_attitude);
  logLine("T", line);
  std::string x;
  char b[96];
  auto f = [&](const char *k, float v) { std::snprintf(b, sizeof b, " %s=%08x", k, bits(v)); x += b; };
  auto i = [&](const char *k, long v) { std::snprintf(b, sizeof b, " %s=%ld", k, v); x += b; };
  i("fz", (int)imu1_frozen_now * 4 + (int)imu2_frozen_now * 2 + (int)imu_disagree_now);
  f("tax", targetAngleX); f("tay", targetAngleY); f("taz", targetAngleZ); f("tyr", targetYawRate);
  f("tr0", tgtRate[0]); f("tr1", tgtRate[1]); f("tr2", tgtRate[2]);
  f("gx", gyroX); f("gy", gyroY); f("gz", gyroZ); f("ax", accX); f("ay", accY); f("az", accZ);
  i("msc", mixer_scaled); f("mys", mixer_yaw_scale); f("prc", pilotRollCmd); f("ppc", pilotPitchCmd);
  i("rc", (long)lastRcMs); i("rcs", (long)lastRcSeq); i("rct", (long)rcTotalPkts); i("rcd", (long)rcDroppedPkts);
  i("rlq", rl_requested_collective_us); f("rld", rl_delivered_collective_us); i("rlb", rl_base_ceiling);
  i("rlc", rl_collective_cap); i("rlh", rl_resume_handover); f("rhb", rlHandoverBase); i("rhs", rlHandoverSet);
  f("rlr", rl_h_ref); i("rlen", rangeland_enabled); i("phq", poshold_requested); i("ahq", althold_requested);
  i("fse", (long)fs_first_enter_ms); i("fsv", fs_first_enter_valid); i("fcs", flowcal_state);
  f("mh", magHeading); f("myo", magYawReferenceOffset); i("me", mag_enabled);
  f("ht", hoverTracker.estimate_us); i("hti", hoverTracker.initialized * 2 + hoverTracker.timing_initialized);
  i("htu", (long)hoverTracker.last_update_ms); i("hte", (long)hoverTracker.eligible_ms);
  i("hv", heightEst.valid); f("hd", heightEst.d_m); f("hh", heightEst.h_m); f("hvz", heightEst.vz_ms);
  i("ht_us", (long)heightEst.last_t_us);
  i("ps", posHold.state * 4 + posHold.started * 2 + posHold.prev_enabled);
  f("pp0", posHold.p[0]); f("pp1", posHold.p[1]); f("pr0", posHold.p_ref[0]); f("pr1", posHold.p_ref[1]);
  f("pi0", posHold.i[0]); f("pi1", posHold.i[1]); f("po0", posHold.out[0]); f("po1", posHold.out[1]);
  f("pyr", posHold.yaw_ref_deg); f("psm", posHold.since_meas_s);
  i("as", altHold.state); i("art", altHold.ref_throttle); f("ahr", altHold.h_ref); f("ai", altHold.i);
  f("ao", altHold.out); i("ras", rlAltHold.state); f("rai", rlAltHold.i); f("rao", rlAltHold.out);
  i("rls", rangeLand.stage); f("rlhr", rangeLand.h_ref); i("rlrm", rangeLand.ref_mm);
  i("rlan", rangeLand.anchor_mm); i("rlrj", rangeLand.rejecting); i("rlrc", rangeLand.reject_count);
  f("rldu", rangeLand.drop_us); f("rlhs", rangeLand.hold_scale);
  i("tk", rlPreTrack.have); i("tkr", rlPreTrack.ref_mm); i("tka", rlPreTrack.anchor_mm);
  i("tkj", rlPreTrack.rejecting); i("tks", (long)rlPreTrack.stable_since_ms);
  i("fa", flowAccel.valid); f("fax", flowAccel.ax_ms2); f("fay", flowAccel.ay_ms2); i("fau", flowAccelUsable);
  i("fo", flowOdomState.have_prev); i("fseq", (long)flowSeqSeen); i("rseq", (long)rangeSeqSeen);
  logLine("X", x.substr(1));
  if (sc.plant_tick % 50 == 0) {
    std::string y;
    auto d = [&](double v) { uint64_t u; std::memcpy(&u, &v, 8); std::snprintf(b, sizeof b, " %016llx", (unsigned long long)u); y += b; };
    std::snprintf(b, sizeof b, "ic=%d,%u,%u", (int)imuCalAcc.active, (unsigned)imuCalAcc.n, (unsigned)imuCalAcc.ticks); y += b;
    for (int k = 0; k < 3; k++) { d(imuCalAcc.sum1[k]); d(imuCalAcc.sq1[k]); d(imuCalAcc.sum2[k]); d(imuCalAcc.sq2[k]); }
    d(imuCalAcc.amag1); d(imuCalAcc.amag2);
    for (int c = 0; c < FLOW_CAL_DELAY_CANDIDATES; c++) {
      const FlowCalAccum &a = flowCalibrator.acc[c];
      std::snprintf(b, sizeof b, " fc%d=%u", c, (unsigned)a.n); y += b;
      d(a.sgg_xx); d(a.sgg_xy); d(a.sgg_yy); d(a.sfg[0][0]); d(a.sfg[0][1]); d(a.sfg[1][0]);
      d(a.sfg[1][1]); d(a.sff[0]); d(a.sff[1]);
    }
    x.clear();   // f() 는 x 에 덧붙인다. 여기부터는 Y 줄용.
    for (int k = 0; k < 3; k++) { f("gb1", gyro_bias1[k]); f("gb2", gyro_bias2[k]); }
    f("as1", accel_scale1); f("as2", accel_scale2);
    logLine("Y", y + x);
  }
}

// 플랜트 1 ms 한 걸음: 적분, 센서 모델, IMU 입력과 고장 주입.
void plantStepOnce(Scenario &sc) {
  const uint32_t tick = sc.plant_tick++;
  std::array<double, 3> acc = {0, 0, 0};   // 정지(손에 든 상태): 1 g 로 읽힌다
  if (sc.kinematic) {
    sc.kinematic(tick, sc.s);
    acc = {0, 0, 0};
  } else if (!sc.held) {
    acc = integrate(sc.s, sc.P, sc.disturbance ? sc.disturbance(tick) : Disturbance{}, tick);
  }
  sc.sensor.tick(tick, sc.s);
  injectImu(sc.s, acc, tick);
  if (sc.imu1_freeze) IMU1.next_event = sc.imu1_frozen;
  else sc.imu1_frozen = IMU1.next_event;
  if (sc.imu2_freeze) IMU2.next_event = sc.imu2_frozen;
  else sc.imu2_frozen = IMU2.next_event;
  IMU2.next_event.gyro[0] = (int16_t)(IMU2.next_event.gyro[0] + sc.imu2_gyro_offset);
  if (sc.imu_offsets) {   // IMU 마다 다른 원값 bias·scale(보정이 센서별로 적용되는지 본다)
    const int16_t o1[3] = {30, -20, 12}, o2[3] = {-16, 26, -6};
    for (int k = 0; k < 3; k++) {
      IMU1.next_event.gyro[k] = (int16_t)(IMU1.next_event.gyro[k] + o1[k]);
      IMU2.next_event.gyro[k] = (int16_t)(IMU2.next_event.gyro[k] + o2[k]);
      IMU1.next_event.accel[k] = (int16_t)lround(IMU1.next_event.accel[k] * 1.03);
      IMU2.next_event.accel[k] = (int16_t)lround(IMU2.next_event.accel[k] * 0.96);
    }
  }
  IMU1.read_status = sc.imu1_read_error ? -1 : 0;
  IMU1.keep_event_on_error = true;
  if (sc.mag_inject) {
    // 기체 yaw 에 따라 도는 수평 자기장(보정 전 원값 = 경철 + 기체 좌표 자기장).
    const double fieldN = 25.0, fieldD = 40.0;
    const double c = std::cos(sc.s.psi), sn = std::sin(sc.s.psi);
    bmm.next_data.float_x = (float)(MAG_HARD_IRON[0] + fieldN * c);
    bmm.next_data.float_y = (float)(MAG_HARD_IRON[1] - fieldN * sn);
    bmm.next_data.float_z = (float)(MAG_HARD_IRON[2] + fieldD);
  }
}

// PID 가 돌기 직전: 플랜트를 지금 가상 시각까지 1 ms 씩 진행한다.
void plantAdvance(Scenario &sc) {
  while (g_plant_us + 1000U <= g_now_us) {
    plantStepOnce(sc);
    g_plant_us += 1000U;
  }
}

int run(Scenario &sc, const char *logPath) {
  g_log = std::fopen(logPath, "w");
  if (!g_log) return 2;
  arduino_fake::lockstep_sleep_us = lsSleepUs;
  arduino_fake::lockstep_sleep_until_tick = lsSleepUntilTick;
  arduino_fake::lockstep_task_created = lsTaskCreated;
  arduino_fake::lockstep_ledc_write = [](int pin, uint32_t duty) {
    char b[48];
    std::snprintf(b, sizeof b, "%d %u", pin, (unsigned)duty);
    logLine("L", b);
  };
  arduino_fake::on_serial = [](const std::string &t) { logLine("S", escape(t)); };
  wifi_udp_fake::on_send = [](const std::string &p) {
    std::ostringstream dest;
    dest << wifi_udp_fake::last_dest_ip << ':' << wifi_udp_fake::last_dest_port << ' ';
    logLine("U", dest.str() + escape(p));
    g_udp_text += p;
    g_udp_text += '\n';
  };
  preferences_fake::on_put = [](const std::string &k, const std::vector<uint8_t> &v) {
    std::string hex;
    char b[4];
    for (uint8_t x : v) { std::snprintf(b, sizeof b, "%02x", x); hex += b; }
    logLine("N", k + " " + hex);
    g_nvs_puts++;
  };
  wifi_udp_fake::remote_ip = IPAddress(192, 168, 4, 2);
  wifi_udp_fake::remote_port = 4210;

  setClock(sc.start_us);
  setup();
  // 하네스 기체는 이상적인 IMU 와 트림 없는 기체다(SIL 과 같다). 같은 조정을 양쪽에 한다.
  for (int k = 0; k < 3; k++) { gyro_bias1[k] = gyro_bias2[k] = 0.0f; }
  accel_scale1 = accel_scale2 = 1.0f;
  trim_roll = trim_pitch = 0.0f;
  setTrueFlowCalibration();

  const uint64_t t0 = g_now_us;
  g_plant_us = t0;
  g_last_pid_us = t0;
  const uint64_t end = t0 + 1000ULL * sc.duration_ms;
  g_task[LOOP].exists = true;
  for (int w = PID; w < NWHO; w++) {
    if (!g_task[w].exists) {
      std::fprintf(stderr, "task %s was not created\n", kWhoName[w]);
      return 2;
    }
    g_task[w].wake_us = t0;
    g_task[w].th = std::thread(workerMain, w);
  }
  auto nextEvent = sc.events.begin();
  uint64_t nextRcUs = t0;
  for (;;) {
    int who = -1;
    for (int w = PID; w < NWHO; w++) {
      if (g_task[w].done) continue;
      if (who < 0 || g_task[w].wake_us < g_task[who].wake_us) who = w;
    }
    if (who < 0 || g_task[who].wake_us > end) break;
    setClock(g_task[who].wake_us);
    const uint32_t elapsedMs = (uint32_t)((g_now_us - t0) / 1000U);
    while (nextEvent != sc.events.end() && nextEvent->first <= elapsedMs) {
      nextEvent->second(sc);
      ++nextEvent;
    }
    if (sc.rc_on && g_now_us >= nextRcUs) {
      char b[96];
      std::snprintf(b, sizeof b, "rcr %u %.2f %.2f %.2f", (unsigned)++sc.rc_seq,
                    (double)sc.rc_roll, (double)sc.rc_pitch, (double)sc.rc_yaw_rate);
      sc.inbox.push_back(b);
      nextRcUs = g_now_us + 20000U;
    } else if (!sc.rc_on) {
      nextRcUs = g_now_us;
    }
    if (who == PID) {
      if (sc.held && !safety_lock) sc.held = false;   // 시동이 걸리면 손을 놓는다
      plantAdvance(sc);
      const uint64_t gap = g_now_us - g_last_pid_us;
      if (gap > 10000U) sc.mark("pid_gap_long");
      if (gap > 0 && gap < 200U) sc.mark("pid_gap_short");
      g_last_pid_us = g_now_us;
    }
    if (who == UDP && wifi_udp_fake::incoming_packet.empty() && !sc.inbox.empty()) {
      wifi_udp_fake::incoming_packet = sc.inbox.front();
      logLine("I", escape(sc.inbox.front()));
      sc.inbox.pop_front();
    }
    grant(who);
    if (who == PID) {
      recordTick(sc);
      if (sc.per_tick) sc.per_tick(sc);
    }
  }
  // 끝: 각 스레드를 깨워 그 안에서 예외로 빠지게 한다.
  {
    std::unique_lock<std::mutex> lk(g_m);
    g_stopping = true;
  }
  for (int w = PID; w < NWHO; w++) {
    if (!g_task[w].done) grant(w);
    g_task[w].th.join();
  }
  char b[160];
  std::snprintf(b, sizeof b, "raw head=%u tail=%u dropped=%u seq=%u first=%u",
                (unsigned)imuRawRing.head, (unsigned)imuRawRing.tail,
                (unsigned)imuRawRing.dropped, (unsigned)rawBatchSeq,
                (unsigned)imuRawRing.first_t_us);
  logLine("E", b);
  for (uint32_t k = imuRawRing.tail; k != imuRawRing.head; k++) {   // 보내지 못한 샘플 내용
    const ImuRawSample &smp = imuRawRing.samples[k & IMU_RAW_RING_MASK];
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&smp);
    std::string hex;
    char h[4];
    for (size_t j = 0; j < sizeof smp; j++) { std::snprintf(h, sizeof h, "%02x", raw[j]); hex += h; }
    logLine("E", "rawpending " + hex);
  }
  for (const auto &kv : preferences_fake::store) {
    std::string hex;
    char h[4];
    for (uint8_t x : kv.second) { std::snprintf(h, sizeof h, "%02x", x); hex += h; }
    logLine("E", "nvs " + kv.first + " " + hex);
  }
  std::fclose(g_log);
  const std::string missing = sc.reach ? sc.reach(sc) : std::string();
  if (!missing.empty()) {
    std::fprintf(stderr, "[%s] did not reach: %s\n", sc.name.c_str(), missing.c_str());
    return 3;
  }
  std::fprintf(stderr, "[%s] ok\n", sc.name.c_str());
  return 0;
}

// ---------------- 시나리오 정의 ----------------
// 공통: 시작 0.2 s 뒤 RC 스트림, 0.3 s 시동, 0.4 s 호버 스로틀. hold=true 면 고도·위치 유지.
void flyUp(Scenario &sc, bool holds, uint32_t at_ms = 200) {
  sc.at(at_ms, [](Scenario &s) { s.rc_on = true; });
  sc.send(at_ms + 100, "start");
  sc.send(at_ms + 200, "th 1340");
  if (holds) {
    sc.send(at_ms + 300, "althold 1");
    sc.send(at_ms + 320, "poshold 1");
  }
}

// PID 의 다음 깨움을 늦춘다: 15 ms(재동기화·rawDt > 10 ms), 1.95 ms(곧바로 다음 tick →
// realDt < 0.2 ms 치환), 1.6 ms(짧은 tick).
void stalls(Scenario &sc, uint32_t at_ms) {
  sc.at(at_ms, [](Scenario &) { g_pid_stall_us = 15000; });
  sc.at(at_ms + 300, [](Scenario &) { g_pid_stall_us = 1950; });
  sc.at(at_ms + 600, [](Scenario &) { g_pid_stall_us = 1600; });
}

void observeCommon(Scenario &sc) {
  if (fs_phase == FS_DESCENDING) sc.mark("descending");
  if (fs_phase == FS_CUT_TIMEOUT) sc.mark("cut_timeout");
  if (fs_phase == FS_CUT_LANDED) sc.mark("cut_landed");
  if (rl_mode == RL_MODE_RANGE && fs_phase == FS_DESCENDING) sc.mark("range_descent");
  if (rl_mode == RL_MODE_TIMED && fs_phase == FS_DESCENDING) sc.mark("timed_descent");
  if (rl_mode == RL_MODE_TIMED_FALLBACK) sc.mark("fallback");
  if (!safety_lock) sc.mark("armed");
  if (hover_valid) sc.mark("hover_valid");
  if (yaw_authority_state != YAW_AUTH_NORMAL) sc.mark("yaw_authority_changed");
  if (active_imus == 1) sc.mark("one_imu");
  if (active_imus == 0) sc.mark("no_imu");
  if (fault_disagree) sc.mark("disagree");
  if (fault_attitude) sc.mark("overtilt");
  if (rl_stage >= RL_STAGE_NEAR_GROUND) sc.mark("near_ground");
  if (mag_enabled && !safety_lock) sc.mark("mag_fusing_armed");
  if (rl_resume_handover) sc.mark("handover");
  if (sc.saw("handover") && fs_phase == FS_DESCENDING && !rl_resume_handover) sc.mark("after_handover_descending");
  if (sc.saw("near_ground") && fs_phase == FS_DESCENDING && rl_mode == RL_MODE_RANGE &&
      rl_stage < RL_STAGE_NEAR_GROUND) sc.mark("revoked");
  if (sc.saw("no_imu") && !safety_lock) sc.mark("rearmed_after_imu_loss");
}

std::string need(const Scenario &sc, std::initializer_list<const char *> keys) {
  std::string missing;
  for (const char *k : keys) {
    if (!sc.saw(k)) missing += std::string(missing.empty() ? "" : ", ") + k;
  }
  return missing;
}

std::string needSerial(const Scenario &sc, std::initializer_list<const char *> texts) {
  std::string missing;
  for (const char *t : texts) {
    if (!sc.serialHas(t)) missing += std::string(missing.empty() ? "" : ", ") + "serial:" + t;
  }
  return missing;
}

std::string needUdp(const Scenario &sc, std::initializer_list<const char *> texts) {
  std::string missing;
  for (const char *t : texts) {
    if (!sc.udpHas(t)) missing += std::string(missing.empty() ? "" : ", ") + "udp:" + t;
  }
  return missing;
}

std::string join(std::string a, const std::string &b) {
  if (a.empty()) return b;
  if (b.empty()) return a;
  return a + ", " + b;
}

void landSensor(Scenario &sc) {
  sc.sensor.ground_offset_mm = 30;
  sc.sensor.report_short = true;
  sc.sensor.noise_mm = 8;
}

std::map<std::string, std::function<void(Scenario &)>> scenarios() {
  std::map<std::string, std::function<void(Scenario &)>> m;

  m["hold_rcloss_timed"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    flyUp(sc, true);
    stalls(sc, 3000);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    stalls(sc, 7200);
    sc.duration_ms = 11000;
    sc.reach = [](Scenario &s) {
      return need(s, {"armed", "hover_valid", "descending", "timed_descent", "cut_timeout",
                      "pid_gap_long", "pid_gap_short"});
    };
  };

  m["no_hover_cut"] = [](Scenario &sc) {
    sc.s.h = 0.5;
    flyUp(sc, false);
    sc.at(1200, [](Scenario &s) { s.rc_on = false; });   // 호버 추정이 유효해지기 전
    sc.duration_ms = 3000;
    sc.reach = [](Scenario &s) { return needSerial(s, {"CUT (NO HOVER EST)"}); };
  };

  m["ground_cut"] = [](Scenario &sc) {
    sc.s.h = 0.5;
    flyUp(sc, false);
    sc.send(4000, "th 1120");                            // 호버 유효 뒤 지상 스로틀
    sc.at(5500, [](Scenario &s) { s.rc_on = false; });
    sc.duration_ms = 7000;
    sc.reach = [](Scenario &s) {
      return join(need(s, {"hover_valid"}), needSerial(s, {"GROUND CUT"}));
    };
  };

  m["range_resume_handover"] = [](Scenario &sc) {
    sc.s.h = 1.5;
    landSensor(sc);
    sc.send(150, "rangeland 1");
    flyUp(sc, true);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    // 바닥 근접(기준 스로틀이 호버보다 내려간 뒤)에서 링크를 살리고 resume 한다. 인계는
    // 그 collective 에서 호버까지 천천히 오르며, 그 도중에 다시 끊긴다.
    sc.per_tick = [](Scenario &s) {
      observeCommon(s);
      if (s.saw("near_ground") && !s.saw("rc_back")) {
        s.mark("rc_back");
        s.rc_on = true;
        s.resume_at_tick = s.plant_tick + 100;   // RC 패킷이 몇 개 들어온 뒤
      }
      if (s.resume_at_tick && s.plant_tick >= s.resume_at_tick) {
        s.resume_at_tick = 0;
        s.inbox.push_back("resume");
      }
      if (s.saw("handover") && !s.saw("reloss_sent")) {
        s.mark("reloss_sent");
        s.rc_off_at_tick = s.plant_tick + 150;
      }
      if (s.rc_off_at_tick && s.plant_tick >= s.rc_off_at_tick) {
        s.rc_on = false;
        s.rc_off_at_tick = 0;
      }
    };
    sc.duration_ms = 24000;
    sc.reach = [](Scenario &s) {
      return join(need(s, {"range_descent", "handover", "after_handover_descending"}),
                  needSerial(s, {">>> RESUME"}));
    };
  };

  m["rangeland_near_ground_revoke"] = [](Scenario &sc) {
    sc.s.h = 0.8;
    landSensor(sc);
    sc.send(150, "rangeland 1");
    flyUp(sc, true);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    // 바닥 근접에 들어가면 0.8 s 동안 위로 밀어 올린다 → 단계 철회.
    sc.per_tick = [](Scenario &s) {
      observeCommon(s);
      if (s.saw("near_ground") && !s.disturbance) {
        const uint32_t from = s.plant_tick;
        const double up = s.P.mass * kG * 0.6;
        s.disturbance = [from, up](uint32_t tick) {
          Disturbance d;
          if (tick >= from && tick < from + 800) d.force_up = up;
          return d;
        };
      }
    };
    sc.duration_ms = 22000;
    sc.reach = [](Scenario &s) { return need(s, {"range_descent", "near_ground", "revoked"}); };
  };

  m["manual_rcloss"] = [](Scenario &sc) {
    sc.s.h = 0.8;
    flyUp(sc, false);
    sc.at(5000, [](Scenario &s) { s.rc_on = false; });
    sc.duration_ms = 9500;
    sc.reach = [](Scenario &s) { return need(s, {"armed", "descending", "cut_timeout"}); };
  };

  m["resume_reloss_stop_rearm"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    flyUp(sc, true);
    sc.at(5000, [](Scenario &s) { s.rc_on = false; });
    sc.at(5800, [](Scenario &s) { s.rc_on = true; });
    sc.send(5900, "resume");
    sc.at(7500, [](Scenario &s) { s.rc_on = false; });
    sc.at(8300, [](Scenario &s) { s.rc_on = true; });
    sc.send(8400, "stop");
    sc.send(9000, "start");
    sc.send(9100, "th 1340");
    sc.duration_ms = 11000;
    sc.reach = [](Scenario &s) {
      return join(need(s, {"armed", "descending"}),
                  needSerial(s, {">>> RESUME", ">>> STOP"}));
    };
  };

  m["resume_cumulative_refused"] = [](Scenario &sc) {
    sc.s.h = 1.5;
    flyUp(sc, true);
    for (uint32_t k = 0; k < 6; k++) {
      const uint32_t t = 4500 + k * 2200;
      sc.at(t, [](Scenario &s) { s.rc_on = false; });
      sc.at(t + 700, [](Scenario &s) { s.rc_on = true; });
      sc.send(t + 800, "resume");
    }
    sc.duration_ms = 19000;
    sc.reach = [](Scenario &s) { return needSerial(s, {"RESUME REFUSED cumulative"}); };
  };

  m["rangeland_contact"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    landSensor(sc);
    sc.send(150, "rangeland 1");
    flyUp(sc, true);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    stalls(sc, 7500);
    sc.duration_ms = 20000;
    sc.reach = [](Scenario &s) { return need(s, {"range_descent", "near_ground", "cut_landed"}); };
  };

  m["rangeland_fallback"] = [](Scenario &sc) {
    sc.s.h = 1.5;
    landSensor(sc);
    sc.send(150, "rangeland 1");
    flyUp(sc, true);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    sc.at(7300, [](Scenario &s) {
      s.sensor.range_off_from = s.plant_tick;
      s.sensor.range_off_to = std::numeric_limits<uint32_t>::max();
    });
    sc.duration_ms = 14000;
    sc.reach = [](Scenario &s) { return need(s, {"range_descent", "fallback"}); };
  };

  m["rangeland_near_ground_loss"] = [](Scenario &sc) {
    sc.s.h = 0.6;
    landSensor(sc);
    sc.send(150, "rangeland 1");
    flyUp(sc, true);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    sc.per_tick = [](Scenario &s) {
      observeCommon(s);
      if (s.saw("near_ground") && s.sensor.range_off_to == 0) {
        s.sensor.range_off_from = s.plant_tick;
        s.sensor.range_off_to = std::numeric_limits<uint32_t>::max();
      }
    };
    sc.duration_ms = 16000;
    sc.reach = [](Scenario &s) { return need(s, {"range_descent", "near_ground", "cut_timeout"}); };
  };

  m["range_lost_at_entry"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    landSensor(sc);
    sc.send(150, "rangeland 1");
    flyUp(sc, true);
    sc.at(6450, [](Scenario &s) {
      s.sensor.range_off_from = s.plant_tick;
      s.sensor.range_off_to = std::numeric_limits<uint32_t>::max();
    });
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    sc.duration_ms = 11000;
    sc.reach = [](Scenario &s) { return need(s, {"timed_descent", "cut_timeout"}); };
  };

  m["overtilt"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    flyUp(sc, false);
    sc.disturbance = [](uint32_t tick) {
      Disturbance d;
      if (tick > 4000 && tick < 4250) d.torque_roll = 0.5;
      return d;
    };
    sc.duration_ms = 6000;
    sc.reach = [](Scenario &s) { return need(s, {"armed", "overtilt"}); };
  };

  m["imu_freeze_one_then_both"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    flyUp(sc, true);
    sc.at(4000, [](Scenario &s) { s.imu1_freeze = true; });
    sc.at(6000, [](Scenario &s) { s.imu2_freeze = true; });
    sc.at(7000, [](Scenario &s) { s.imu1_freeze = false; s.imu2_freeze = false; });
    sc.send(7200, "stop");
    sc.send(7500, "start");
    sc.send(7600, "th 1340");
    sc.duration_ms = 9500;
    sc.reach = [](Scenario &s) { return need(s, {"one_imu", "no_imu", "rearmed_after_imu_loss"}); };
  };

  m["imu_disagree"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    flyUp(sc, true);
    sc.at(4000, [](Scenario &s) { s.imu2_gyro_offset = 400; });   // 약 24 dps
    sc.duration_ms = 6000;
    sc.reach = [](Scenario &s) { return need(s, {"disagree"}); };
  };

  m["imu_read_error"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    flyUp(sc, true);
    sc.at(4000, [](Scenario &s) { s.imu1_read_error = true; });
    sc.at(4030, [](Scenario &s) { s.imu1_read_error = false; });
    sc.at(5000, [](Scenario &s) { s.imu1_read_error = true; });
    sc.at(5600, [](Scenario &s) { s.imu1_read_error = false; });
    sc.duration_ms = 7000;
    sc.reach = [](Scenario &s) { return need(s, {"one_imu"}); };
  };

  m["low_throttle_yaw"] = [](Scenario &sc) {
    sc.s.h = 0.0;
    sc.at(200, [](Scenario &s) { s.rc_on = true; });
    sc.send(300, "start");
    sc.send(400, "th 1150");
    // 스틱은 중립인데 기체가 빠르게 돌면 저스로틀에서 yaw 배분이 깎인다 → LIMITED.
    sc.at(1500, [](Scenario &s) { s.s.r = 6.0; });
    sc.at(1700, [](Scenario &s) { s.s.r = 6.0; });
    sc.at(2500, [](Scenario &s) { s.rc_yaw_rate = 90.0f; });   // 스틱을 움직이면 해제
    sc.at(2800, [](Scenario &s) { s.rc_yaw_rate = 0.0f; });
    sc.send(4000, "th 1340");
    sc.duration_ms = 6500;
    sc.reach = [](Scenario &s) { return need(s, {"armed", "yaw_authority_changed"}); };
  };

  m["commands"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    sc.send(100, "gains");
    sc.send(120, "yaw 1");
    sc.send(140, "trim 1.5 -2.25");
    sc.send(160, "magc 0.001 0.002 0.003");
    sc.send(180, "raw 0");
    sc.send(190, "hg kv 6");
    sc.send(200, "hg kpos 0.9");
    sc.send(210, "hg khi 30");
    sc.send(220, "hg vmax 0.4");
    flyUp(sc, true, 250);
    for (const char *c : {"pa 0.55", "ia 0.21", "da 0.016", "pr 0.5", "ir 0.2", "dr 0.015",
                          "pp 0.5", "ip 0.2", "dp 0.015", "py 1.6", "iy 0.06", "dy 0.001",
                          "rp 0.5", "ri 0.2", "rd 0.015", "yp 1.5", "yi 0.05", "yd 0",
                          "ap 6", "ar 6.1", "at 5.9", "ay 3", "gains", "hg kv 6",
                          "hg kp 0.9", "hg bogus 1", "raw 1", "yaw 0", "yaw 1",
                          "trim 20 -20", "th 1360", "th abc", "xyz", "pa", "trim 1",
                          "rangeland 1", "flowcal 1", "imucal", "magcal 1", "mag 2",
                          "start", "rc 3 1.0 -1.0 2.0", "rc 1.0 2.0", "rcr bad",
                          "  gains  ", "poshold 0", "althold 0", "poshold 1"}) {
      static uint32_t t = 1500;
      sc.send(t, c);
      t += 60;
    }
    sc.at(5500, [](Scenario &s) { s.rc_on = false; });
    sc.send(6200, "gains");
    sc.send(6250, "pa 0.7");
    sc.send(6300, "trim 0 0");
    sc.duration_ms = 9000;
    sc.reach = [](Scenario &s) {
      return join(need(s, {"armed", "descending"}),
                  needSerial(s, {"Yaw hold ON", "refused", ">>> hold gain kv = 6.000",
                                 ">>> hold gain khi = 30.000"}));
    };
  };

  m["flowcal_ok_and_failed"] = [](Scenario &sc) {
    sc.s.h = 0.5;
    sc.sensor.delay_s = 0.020;
    sc.kinematic = [](uint32_t tick, State &s) {
      const double t = tick * kDt;
      const double w1 = 2 * kPi * 0.8, w2 = 2 * kPi * 0.55;
      double roll = 0, pitch = 0, p = 0, q = 0;
      if (t < 12) { roll = 0.25 * std::sin(w1 * t); p = 0.25 * w1 * std::cos(w1 * t); }
      else if (t < 24) { pitch = 0.25 * std::sin(w1 * t); q = 0.25 * w1 * std::cos(w1 * t); }
      else if (t < 30) {
        roll = 0.2 * std::sin(w1 * t); p = 0.2 * w1 * std::cos(w1 * t);
        pitch = 0.2 * std::sin(w2 * t); q = 0.2 * w2 * std::cos(w2 * t);
      }
      s.phi = roll; s.theta = pitch; s.p = p; s.q = q; s.r = 0;
      s.n = s.e = 0; s.h = 0.5; s.vn = s.ve = s.vh = 0;
    };
    sc.send(50, "flowcal 1");
    sc.send(29900, "flowcal 0");
    sc.send(30500, "flowcal 1");
    sc.send(30700, "flowcal 0");
    sc.duration_ms = 31500;
    sc.reach = [](Scenario &s) { return needUdp(s, {"FLOWCAL,1,", "FLOWCAL,0,"}); };
  };

  m["imucal_ok_and_moved"] = [](Scenario &sc) {
    sc.s.h = 0.0;
    sc.imu_offsets = true;
    sc.send(100, "imucal");
    sc.at(3000, [](Scenario &s) {
      s.kinematic = [](uint32_t tick, State &st) {
        st.p = 0.3 * std::sin(2.0 * kPi * tick / 400.0);
      };
    });
    sc.send(3100, "imucal");
    sc.at(5400, [](Scenario &s) { s.kinematic = nullptr; s.s = State(); s.s.h = 0.0; });
    flyUp(sc, false, 5500);                              // 보정값으로 이륙
    sc.duration_ms = 9000;
    sc.reach = [](Scenario &s) {
      std::string m = join(join(needUdp(s, {"IMUCAL,1,", "IMUCAL,0,MOVED"}),
                                needSerial(s, {"saved=1"})), need(s, {"armed"}));
      return g_nvs_puts > 0 ? m : join(m, "nvs write");
    };
  };

  m["mag_fusion_and_magcal"] = [](Scenario &sc) {
    sc.s.h = 1.0;
    sc.mag_inject = true;
    sc.send(100, "magcal 1");
    sc.at(150, [](Scenario &s) {
      s.kinematic = [](uint32_t tick, State &st) { st.psi = 0.002 * tick; st.r = 2.0; };
    });
    sc.at(3300, [](Scenario &s) { s.kinematic = nullptr; s.s.psi = 0; s.s.r = 0; });
    sc.send(3400, "magcal 0");
    sc.send(3500, "mag 1");
    flyUp(sc, true, 3600);
    sc.at(5000, [](Scenario &s) { s.rc_yaw_rate = 60.0f; });
    sc.at(7000, [](Scenario &s) { s.rc_yaw_rate = 0.0f; });
    sc.send(9000, "mag 0");
    sc.duration_ms = 10000;
    sc.reach = [](Scenario &s) {
      return join(need(s, {"armed", "mag_fusing_armed"}),
                  needSerial(s, {"Mag ON", "Mag OFF", "[MAGCAL] samples="}));
    };
  };

  m["refused_arm_paths"] = [](Scenario &sc) {
    sc.s.h = 0.0;
    sc.at(100, [](Scenario &s) { s.rc_on = true; });
    sc.send(200, "magcal 1");
    sc.send(300, "start");                                   // magcal 중 거부
    sc.send(400, "magcal 0");
    sc.at(500, [](Scenario &s) { s.imu2_gyro_offset = 400; });
    sc.send(900, "start");                                   // 불일치 중 거부
    sc.at(1000, [](Scenario &s) { s.imu2_gyro_offset = 0; });
    sc.send(1100, "flowcal 1");
    sc.send(1200, "start");                                  // flowcal 중 거부
    sc.send(1300, "flowcal 0");
    sc.send(1500, "start");
    sc.send(1600, "start");                                  // 이미 무장
    sc.send(1700, "imucal");                                 // 무장 중 거부
    sc.send(1800, "stop");
    sc.duration_ms = 2500;
    sc.reach = [](Scenario &s) {
      return needSerial(s, {"START REFUSED", "already armed", "IMUCAL refused"});
    };
  };

  m["micros_wrap"] = [](Scenario &sc) {
    sc.start_us = (1ULL << 32) - 4000000ULL;   // 비행 중 micros 가 감긴다
    sc.s.h = 1.0;
    landSensor(sc);
    sc.send(150, "rangeland 1");
    flyUp(sc, true);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    sc.duration_ms = 16000;
    sc.reach = [](Scenario &s) { return need(s, {"armed", "descending"}); };
  };

  m["millis_wrap"] = [](Scenario &sc) {
    sc.start_us = ((1ULL << 32) - 4000ULL) * 1000ULL;   // 비행 중 millis 가 감긴다
    sc.s.h = 1.0;
    flyUp(sc, true);
    sc.at(6500, [](Scenario &s) { s.rc_on = false; });
    sc.duration_ms = 11000;
    sc.reach = [](Scenario &s) { return need(s, {"armed", "descending", "cut_timeout"}); };
  };

#if WIFI_LATENCY_DEBUG
  // 자기계 첫 초기화의 delay(100) 이 udp_task 를 100 ms 멈춰 지연 이벤트를 만든다.
  m["latency_debug"] = [](Scenario &sc) {
    sc.s.h = 0.0;
    sc.mag_inject = true;
    sc.send(100, "mag 1");
    sc.duration_ms = 4500;
    sc.reach = [](Scenario &s) { return needSerial(s, {"[LATSUM]", "UDP_LOOP"}); };
  };
#endif
  return m;
}

}  // namespace

int main(int argc, char **argv) {
  auto all = scenarios();
  if (argc == 2 && std::string(argv[1]) == "--list") {
    for (const auto &kv : all) std::cout << kv.first << '\n';
    return 0;
  }
  if (argc != 3 || !all.count(argv[1])) {
    std::fprintf(stderr, "usage: %s <scenario> <log> | --list\n", argv[0]);
    return 2;
  }
  Scenario sc;
  sc.name = argv[1];
  sc.per_tick = observeCommon;
  all[argv[1]](sc);
  return run(sc, argv[2]);
}
