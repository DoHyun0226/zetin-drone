#pragma once

#include <Arduino.h>

// [페일세이프 자동착륙 헬퍼 정리]
//  RC(조종 명령)가 RC_TIMEOUT_MS 동안 끊기면 pid_task가 아래 함수들로 자동착륙을 진행한다
//   1) 호버 추정(HoverThrottleEstimator): 평소 비행 중 "떠 있는 데 필요한 스로틀"을 학습해 둔다
//   2) 하강: 호버 스로틀보다 FS_DESCENT_DELTA_US 낮춰 천천히 내려간다 (failsafeDescentThrottle)
//   3) 종료: 현재는 시간 기반. FS_MAX_MS가 지나면 모터 정지 (failsafeStep → FS_CUT_TIMEOUT)
//  착지 감지기(LandDetector, 능동 프로브)는 계속 돌지만 판정에는 쓰지 않고 텔레메트리 기록용이다

// 자동착륙 상태. 텔레메트리 Failsafe_Phase로 그대로 나간다.
enum FailsafePhase : uint8_t {
  FS_NONE        = 0,   // 자동착륙 아님 (정상 비행 또는 대기)
  FS_DESCENDING  = 1,   // 하강 중
  FS_CUT_LANDED  = 2,   // 착지 판정으로 모터 정지 (현재 코드에서는 선택되지 않음, landed=false)
  FS_CUT_TIMEOUT = 3,   // FS_MAX_MS 시간 초과로 모터 정지 (현재 실제 종료 경로)
  FS_CUT_ABORT   = 4,   // 하강 중 다른 이유(stop, IMU 고장 등)로 강제 중단
};

// 자동착륙 능동 프로브 상태. 그대로 텔레메트리에 내보내므로 값은 append-only
// wire contract다.
// (append-only: 새 값은 끝에만 추가하고 기존 숫자는 바꾸지 않는다 → 예전 로그/지상국 해석이 깨지지 않게)
enum FailsafeProbeState : uint8_t {
  FS_PROBE_WAIT        = 0,   // 다음 프로브까지 대기
  FS_PROBE_DIP         = 1,   // 스로틀을 잠깐 낮추는 중 (딥 진행)
  FS_PROBE_EVALUATE    = 2,   // 딥이 끝나 반응을 평가한 직후 (한 tick)
  FS_PROBE_UNAVAILABLE = 3,   // 딥을 빼면 1000µs 아래라 프로브 불가 (이후 계속 이 상태)
  FS_PROBE_BLOCKED     = 4,   // 요청한 딥이 실제 모터에 충분히 전달되지 않아 결과 무효
};

// 호버 후보인 시간만 누적한다. 부적합 구간은 누적과 LPF를 멈추지만 이미
// 모은 시간/추정치는 보존한다. 일단 유효해진 추정치도 비행 중 후보 조건이
// 잠시 깨져도 보존하고, 유효한 샘플에서만 느린 LPF로 갱신한다.
struct HoverThrottleEstimator {
  float estimate_us = 0.0f;          // 추정 호버 스로틀 (µs) → hover_est
  bool initialized = false;          // 첫 유효 샘플로 estimate_us를 채웠는가
  bool timing_initialized = false;   // last_update_ms에 기준 시각이 들어갔는가
  bool valid = false;                // 누적 eligible_ms ≥ valid_ms → 믿을 만한 추정치 (hover_valid). 한 번 true면 유지
  uint32_t last_update_ms = 0;       // 직전 호출 시각 (경과 시간 계산용)
  uint32_t eligible_ms = 0;          // 호버 조건을 만족한 시간의 누적 합 (ms)
};

// 매 tick 호출. 호버 조건(sample_eligible)일 때만 스로틀을 저역통과 평균해 추정치를 갱신한다
//  sample_eligible : 수평(±10°) + |accel|≈1g + 스로틀 > 1150µs + 시동 중 + 자동착륙 아님
//  throttle_us     : 현재 base_throttle
//  dt_s, tau_s     : 루프 주기와 필터 시정수(HOVER_LPF_TAU_S = 3초)
//  valid_ms        : 이만큼 누적되면 valid = true (HOVER_VALID_MS = 1.5초)
static inline void updateHoverThrottleEstimator(
    HoverThrottleEstimator &estimator, bool sample_eligible,
    int throttle_us, uint32_t now_ms, float dt_s, float tau_s,
    uint32_t valid_ms) {
  // 경과 시간은 조건과 상관없이 매번 갱신한다 → 부적합 구간 시간이 다음 유효 샘플에 몰려 더해지지 않게
  uint32_t elapsed_ms = 0;
  if (estimator.timing_initialized) {
    elapsed_ms = now_ms - estimator.last_update_ms;
  } else {
    estimator.timing_initialized = true;
  }
  estimator.last_update_ms = now_ms;

  if (!sample_eligible) return;   // 조건 밖이면 추정치와 누적 시간 모두 그대로 둔다

  if (!estimator.initialized) {
    // 첫 샘플은 그대로 시작값으로 (0에서부터 천천히 올라오면 시간이 오래 걸리므로)
    estimator.estimate_us = (float)throttle_us;
    estimator.initialized = true;
  } else {
    // 1차 저역통과: alpha = dt/(tau+dt). 3초에 걸쳐 천천히 따라가 순간 스로틀 변화는 무시
    const float alpha = dt_s / (tau_s + dt_s);
    estimator.estimate_us +=
        alpha * ((float)throttle_us - estimator.estimate_us);
  }

  // uint32 오버플로 방지: 남은 여유(room)보다 크게 더하지 않는다
  const uint32_t room = UINT32_MAX - estimator.eligible_ms;
  estimator.eligible_ms += elapsed_ms < room ? elapsed_ms : room;
  if (estimator.eligible_ms >= valid_ms) {
    estimator.valid = true;
  }
}

// 착지 감지기 설정값 묶음 (.ino의 FS_* 상수에서 FS_LAND_PROBE_CONFIG로 채운다)
struct LandProbeConfig {
  float lpf_alpha;            // |accel| 저역통과 계수 (FS_LAND_LPF_ALPHA)
  uint32_t min_descend_ms;    // 하강 시작 후 이 시간이 지나야 첫 프로브
  uint32_t period_ms;         // 프로브 반복 주기
  uint32_t dip_ms;            // 딥 유지 시간
  uint32_t sample_delay_ms;   // 딥 시작 후 이 시간이 지나야 반응 측정 시작 (모터 반응 지연 대기)
  float response_g;           // 이보다 작은 가속도 변화면 "반응 없음"
  float ground_accel_tol_g;   // 착지 후보 조건: |accel|이 1g ± 이 값 안
  uint8_t confirm_n;          // "반응 없음" 연속 횟수 기준
  int dip_us;                 // 실제 적용할 딥 크기 (µs). failsafeProbeDipUs()로 계산해 넣는다
};

// hover_est에서 1000us를 뺀 유효 collective에 비례해 딥을 만든다. 측정
// 가능한 하한과 제어 authority 상한은 런타임 값이므로 적용값과 clamp 여부를
// 함께 반환한다. upper_exclusive_us는 반드시 min_us보다 커야 한다.
//  예) hover_est 1340µs, dip_frac 0.118 → (1340-1000)×0.118 ≈ 40µs 딥
//  clamped(참조 출력): 하한/상한에 걸려 요청값과 달라졌으면 true → 호출 쪽이 경고 로그를 찍는다
static inline int failsafeProbeDipUs(
    float hover_estimate_us, float dip_frac, int min_us,
    int upper_exclusive_us, bool &clamped) {
  // NaN/무한대나 잘못된 설정이면 최소 딥으로 대체
  if (!isfinite(hover_estimate_us) || !isfinite(dip_frac) ||
      dip_frac <= 0.0f || upper_exclusive_us <= min_us) {
    clamped = true;
    return min_us;
  }

  const int requested_us = (int)lroundf(
      (hover_estimate_us - 1000.0f) * dip_frac);
  // [min_us, upper_exclusive_us) 범위로 자른다. 상한은 "미만"이라 upper - 1이 최대값
  const int applied_us =
      requested_us < min_us
          ? min_us
          : (requested_us >= upper_exclusive_us
                 ? upper_exclusive_us - 1
                 : requested_us);
  clamped = applied_us != requested_us;
  return applied_us;
}

// resume 거부 사유. .ino 안에 두면 arduino-cli가 자동 생성한 함수
// 프로토타입이 enum 선언보다 앞에 삽입돼 컴파일이 깨진다(헤더는 그 전처리
// 대상이 아니다). 네이티브 테스트는 .ino를 g++로 직접 컴파일해 이 단계가
// 없으므로 이 결함을 잡지 못한다 — 아두이노 컴파일이 유일한 관문이다.
// ("resume" 명령: 자동착륙 하강 중 링크가 돌아왔을 때 조종을 다시 넘겨받는 명령)
enum ResumeRefusalReason : uint8_t {
  RESUME_ALLOWED = 0,          // 허용
  RESUME_REFUSED_PHASE,        // 하강 중이 아님 (되돌릴 자동착륙이 없음)
  RESUME_REFUSED_RC,           // RC가 아직 끊긴 상태
  RESUME_REFUSED_TILT,         // 기울기가 SAFETY_ANGLE 초과
  RESUME_REFUSED_IMU,          // 쓸 수 있는 IMU가 없거나 두 IMU 불일치
  RESUME_REFUSED_HOVER,        // 호버 추정치가 유효하지 않음 (복귀할 스로틀을 모름)
  RESUME_REFUSED_CUMULATIVE,   // 첫 자동착륙 진입 후 FS_RESUME_MAX_MS가 지남 (반복 resume로 오래 버티는 것 방지)
};

// 시리얼 로그용 사유 문자열
static inline const char *resumeRefusalName(ResumeRefusalReason reason) {
  switch (reason) {
    case RESUME_REFUSED_PHASE: return "phase";
    case RESUME_REFUSED_RC: return "rc";
    case RESUME_REFUSED_TILT: return "tilt";
    case RESUME_REFUSED_IMU: return "imu";
    case RESUME_REFUSED_HOVER: return "hover";
    case RESUME_REFUSED_CUMULATIVE: return "cumulative";
    default: return "unknown";
  }
}

// 착지 감지기의 누적 상태. pid_task가 소유하고 진입 시 {}로 초기화한다.
// [능동 프로브 원리] 스로틀을 잠깐 낮췄을 때(딥) 공중이면 기체가 살짝 떨어지며 |accel|이 줄고,
//  땅에 닿아 있으면 지면이 받쳐서 거의 변화가 없다 → "반응 없음"이 연속되면 착지 후보
struct LandDetector {
  bool     filt_init = false;       // filt에 첫 값이 들어갔는가
  float    filt = 1.0f;             // LPF된 |accel| (g)
  FailsafeProbeState probe_state = FS_PROBE_WAIT;   // 현재 프로브 단계
  uint32_t probe_start_ms = 0;      // 마지막 딥 시작 시각 (하강 시작 기준 경과 ms)
  float    pre_dip_g = 1.0f;        // 딥 직전 filt 값
  float    dip_min_g = 1.0f;        // 딥 중 filt 최솟값
  float    last_response_g = 0.0f;  // pre_dip_g - dip_min_g
  uint8_t  no_response_count = 0;   // "반응 없음" 연속 횟수
  bool     probe_delivery_valid = true;   // 이번 딥이 실제 모터에 충분히 전달됐는가
};

// 지금 딥 중인가 → true면 pid_task가 실제 스로틀에서 dip_us만큼 뺀다
static inline bool landDetectorProbeActive(const LandDetector &det) {
  return det.probe_state == FS_PROBE_DIP;
}

// mixAndDesaturate 뒤에 호출해 실제 collective 전달 여부를 검출기에 돌려준다.
// 표본 채집 구간의 한 tick이라도 미달이면 부분 딥의 최솟값은 신뢰하지 않는다.
// (allocator가 자세 유지를 위해 collective를 밀어 올리면 요청한 딥이 실제로는 덜 적용될 수 있다)
static inline void recordLandDetectorProbeDelivery(
    LandDetector &det, uint32_t elapsed_ms, bool delivery_valid,
    const LandProbeConfig &config) {
  if (det.probe_state != FS_PROBE_DIP) return;
  const uint32_t dip_elapsed_ms = elapsed_ms - det.probe_start_ms;
  if (dip_elapsed_ms >= config.sample_delay_ms &&
      dip_elapsed_ms < config.dip_ms && !delivery_valid) {
    det.probe_delivery_valid = false;
  }
}

// 매 tick 호출. 검출기가 대기→딥→후보 판정을 소유하고 후보 조건 충족 시 true다.
// 현재 pid_task는 반환값을 착지 컷에 쓰지 않는다(landed=false). 상태/응답은
// 기록하고 landDetectorProbeActive()로 실제 스로틀 딥을 계속 구동한다.
//
// LPF는 min_descend 게이트보다 먼저 항상 갱신한다. 딥 직전값과 딥 중 최솟값의
// 차분을 쓴다. 딥 전후 일정한 성분은 줄지만 저주파 운동도 120ms 동안 변하므로
// 완전 상쇄를 보장하지 않는다. response_g와 confirm_n은 후보 판정 기준이며,
// 실제 지면/공중 분리 성능은 검증되지 않았다. docs/failsafe_land_research.md.
//  accel_magnitude_g : 이번 tick |accel| (g)
//  elapsed_ms        : 하강 시작 후 경과 시간
//  base_throttle_us  : 딥을 빼기 전 하강 스로틀 (딥 후 1000µs 아래인지 확인용)
static inline bool updateLandDetector(
    LandDetector &det, float accel_magnitude_g, uint32_t elapsed_ms,
    int base_throttle_us, const LandProbeConfig &config) {
  // 1) |accel| 저역통과 (진동 노이즈 제거). 첫 값은 그대로 시작
  if (!det.filt_init) {
    det.filt = accel_magnitude_g;
    det.filt_init = true;
  } else {
    det.filt += config.lpf_alpha * (accel_magnitude_g - det.filt);
  }

  if (det.probe_state == FS_PROBE_UNAVAILABLE) {
    return false;   // 한 번 불가 판정이 나면 이번 하강 동안 다시 시도하지 않는다
  }

  if (elapsed_ms < config.min_descend_ms) {
    return false;   // 하강 직후 과도 구간에는 프로브하지 않는다
  }

  // 판정과 다음 대기 전환을 별도 tick으로 유지한다. 20Hz 텔레메트리는
  // 지속되는 DIP/UNAVAILABLE 상태와 누적 카운터/마지막 응답을 주 진단으로 쓴다.
  if (det.probe_state == FS_PROBE_EVALUATE ||
      det.probe_state == FS_PROBE_BLOCKED) {
    det.probe_state = FS_PROBE_WAIT;
    return false;
  }

  // 2) 대기: 프로브 시각이 되면 딥 시작
  if (det.probe_state == FS_PROBE_WAIT) {
    if (base_throttle_us - config.dip_us < 1000) {
      det.probe_state = FS_PROBE_UNAVAILABLE;   // 딥을 빼면 ESC 정지 신호 아래 → 프로브 불가
      return false;
    }
    // 첫 프로브는 min_descend_ms에, 이후는 직전 프로브 시작 + period_ms마다
    const uint32_t next_probe_ms =
        det.probe_start_ms == 0
            ? config.min_descend_ms
            : det.probe_start_ms + config.period_ms;
    if (elapsed_ms < next_probe_ms) return false;

    det.probe_state = FS_PROBE_DIP;
    det.probe_start_ms = elapsed_ms;
    det.pre_dip_g = det.filt;      // 딥 직전 기준값 저장
    det.dip_min_g = det.filt;
    det.probe_delivery_valid = true;
    return false;
  }

  // 3) 딥 중: sample_delay 이후의 최솟값을 기록
  const uint32_t dip_elapsed_ms = elapsed_ms - det.probe_start_ms;
  if (dip_elapsed_ms >= config.sample_delay_ms &&
      det.filt < det.dip_min_g) {
    det.dip_min_g = det.filt;
  }
  if (dip_elapsed_ms < config.dip_ms) {
    return false;
  }

  // 4) 딥 종료: 반응 크기 = 직전값 - 최솟값 (공중이면 떨어지면서 |accel|이 줄어 반응이 크다)
  det.last_response_g = det.pre_dip_g - det.dip_min_g;
  if (!det.probe_delivery_valid) {
    det.probe_state = FS_PROBE_BLOCKED;
    // 전달이 부족한 프로브는 카운트를 올리거나 지우지 않는다. 현재 pid_task는
    // 프로브 결과와 무관하게 FS_MAX_MS 백스톱으로 컷한다.
    return false;
  }
  det.probe_state = FS_PROBE_EVALUATE;
  if (det.last_response_g > config.response_g) {
    det.no_response_count = 0;   // 반응 있음 = 공중 → 연속 카운트 리셋
  } else if (det.no_response_count < UINT8_MAX) {
    det.no_response_count++;     // 반응 없음 = 지면 의심 → 연속 카운트 증가 (255에서 멈춤)
  }
  // 과거의 폐기된 방식은 1g 복귀를 착지의 충분조건으로 썼다. 여기서는 능동
  // 프로브 연속 무반응이 여전히 주 판별이며, 지면에 정지하면 반드시 1g라는
  // 사실만 이용해 1g 근처를 후보 판정의 필요조건으로 추가한다. 1g만으로는
  // true를 반환하지 않으며, true도 현재 pid_task에서는 착지 확정이 아니다.
  const bool near_stationary_1g =
      fabsf(det.filt - 1.0f) <= config.ground_accel_tol_g;
  return det.no_response_count >= config.confirm_n && near_stationary_1g;
}

// 하강 스로틀 = 진입 시 호버 스로틀 - descent_delta_us. 1000µs(정지) 아래로는 내려가지 않는다
static inline int failsafeDescentThrottle(int entry_throttle,
                                          int descent_delta_us) {
  const int throttle = entry_throttle - descent_delta_us;
  return throttle < 1000 ? 1000 : throttle;
}

// 이번 tick의 다음 단계: 착지 판정이면 LANDED, 시간 초과면 TIMEOUT, 아니면 계속 하강
static inline FailsafePhase failsafeStep(bool landed, uint32_t elapsed_ms,
                                         uint32_t max_ms) {
  if (landed) return FS_CUT_LANDED;
  if (elapsed_ms >= max_ms) return FS_CUT_TIMEOUT;
  return FS_DESCENDING;
}
