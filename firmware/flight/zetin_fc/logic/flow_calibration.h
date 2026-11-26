#pragma once

#include <math.h>
#include <stdint.h>

// 3901-L0X 광류 보정. 손으로 들고 제자리에서 roll·pitch 를 흔드는 동안
// 광류 한 구간의 카운트 f 와 같은 구간의 자이로 적분 g [rad] 를 모아
// f ≈ M·g 인 2×2 행렬 M [counts/rad] 를 최소제곱으로 구한다. M 에 센서축↔기체축
// 대응·부호·스케일이 모두 들어 있으므로 부호를 손으로 정하지 않는다.
//
// 광류와 자이로 사이 지연은 모른다(Matek MCU 가 측정 구간 시각을 주지 않는다).
// 그래서 지연 후보 FLOW_CAL_DELAY_CANDIDATES 개를 동시에 누적하고 잔차가 가장
// 작은 후보를 고른다. 지연을 무시하면 정상 보정이 거부되거나 스케일이 10% 넘게
// 작아질 수 있다(계획 검토 ②-6).
//
// 판정이 하나라도 실패하면 결과를 쓰지 않는다. 특히 두 축이 거의 같이 움직인
// 자극은 M 의 모양 검사를 통과하면서 축을 뒤바꿀 수 있어(검토 ②-3 반례)
// 입력 공분산의 최소 고유값·조건수·상관으로 따로 거른다.
// 설계: docs/superpowers/plans/2026-10-29-flow-position-hold.md §4.2

static const int FLOW_CAL_DELAY_CANDIDATES = 11;
static const uint32_t FLOW_CAL_DELAY_STEP_US = 10000;   // 0, 10, …, 100 ms

static const uint32_t FLOW_CAL_MIN_SAMPLES = 150;         // 10 Hz 로 약 15 s
static const double FLOW_CAL_MIN_EIG_RAD2 = 0.02 * 0.02;  // 구간당 0.02 rad
static const double FLOW_CAL_MAX_COND = 8.0;
static const double FLOW_CAL_MAX_ABS_CORR = 0.6;
static const double FLOW_CAL_MAX_RESIDUAL = 0.25;          // 1 − R²
static const double FLOW_CAL_MAX_OFFAXIS_RATIO = 0.25;
static const double FLOW_CAL_MIN_COUNTS_PER_RAD = 3.0 * 180.0 / M_PI;
static const double FLOW_CAL_MAX_COUNTS_PER_RAD = 30.0 * 180.0 / M_PI;

enum FlowCalReason : uint8_t {
  FLOW_CAL_OK = 0,
  FLOW_CAL_TOO_FEW,
  FLOW_CAL_LOW_EXCITATION,
  FLOW_CAL_COLLINEAR,
  FLOW_CAL_POOR_FIT,
  FLOW_CAL_NOT_PERMUTATION,
  FLOW_CAL_SCALE_RANGE,
};

// 한 지연 후보의 정규방정식 누적. double 은 ESP32 에서 소프트웨어 연산이지만
// 보정은 10 Hz × 11 후보라 부담이 없고, 제곱합의 정밀도가 필요하다.
struct FlowCalAccum {
  double sgg_xx = 0.0, sgg_xy = 0.0, sgg_yy = 0.0;   // Σ g gᵀ
  double sfg[2][2] = {{0.0, 0.0}, {0.0, 0.0}};       // Σ f_i g_j
  double sff[2] = {0.0, 0.0};                        // Σ f_i²
  uint32_t n = 0;
};

struct FlowCalibrator {
  FlowCalAccum acc[FLOW_CAL_DELAY_CANDIDATES];
};

struct FlowCalResult {
  bool ok = false;
  uint8_t reason = FLOW_CAL_TOO_FEW;
  float m[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};      // counts/rad
  uint32_t delay_us = 0;
  float residual = 1.0f;
  float min_eig_rad2 = 0.0f;
  float cond = 0.0f;
  float corr = 0.0f;
  uint32_t samples = 0;
};

static inline void flowCalReset(FlowCalibrator &cal) {
  for (int k = 0; k < FLOW_CAL_DELAY_CANDIDATES; k++) cal.acc[k] = FlowCalAccum();
}

static inline void flowCalAddSample(FlowCalibrator &cal, int k,
                                    int32_t fx, int32_t fy,
                                    float gx_rad, float gy_rad) {
  if (k < 0 || k >= FLOW_CAL_DELAY_CANDIDATES) return;
  FlowCalAccum &a = cal.acc[k];
  const double f[2] = {(double)fx, (double)fy};
  const double g[2] = {(double)gx_rad, (double)gy_rad};
  a.sgg_xx += g[0] * g[0];
  a.sgg_xy += g[0] * g[1];
  a.sgg_yy += g[1] * g[1];
  for (int i = 0; i < 2; i++) {
    a.sfg[i][0] += f[i] * g[0];
    a.sfg[i][1] += f[i] * g[1];
    a.sff[i] += f[i] * f[i];
  }
  a.n++;
}

static inline bool flowCalInvert(const float m[2][2], float out[2][2]) {
  const float det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  const float scale = fabsf(m[0][0]) + fabsf(m[0][1]) +
                      fabsf(m[1][0]) + fabsf(m[1][1]);
  if (!(scale > 0.0f) || fabsf(det) <= 1e-6f * scale * scale) return false;
  out[0][0] = m[1][1] / det;
  out[0][1] = -m[0][1] / det;
  out[1][0] = -m[1][0] / det;
  out[1][1] = m[0][0] / det;
  return true;
}

// 한 후보의 M 과 잔차비. 정규행렬이 특이하면 false.
static inline bool flowCalFitCandidate(const FlowCalAccum &a, double m[2][2],
                                       double &residual) {
  const double det = a.sgg_xx * a.sgg_yy - a.sgg_xy * a.sgg_xy;
  if (!(det > 0.0)) return false;
  // M = (Σ f gᵀ)(Σ g gᵀ)⁻¹
  const double ixx = a.sgg_yy / det, ixy = -a.sgg_xy / det, iyy = a.sgg_xx / det;
  double sse = 0.0;
  for (int i = 0; i < 2; i++) {
    m[i][0] = a.sfg[i][0] * ixx + a.sfg[i][1] * ixy;
    m[i][1] = a.sfg[i][0] * ixy + a.sfg[i][1] * iyy;
    // 최소제곱 해에서 행별 SSE = Σf² − mᵀ(Σ g f)
    sse += a.sff[i] - (m[i][0] * a.sfg[i][0] + m[i][1] * a.sfg[i][1]);
  }
  const double total = a.sff[0] + a.sff[1];
  if (!(total > 0.0)) return false;
  residual = sse > 0.0 ? sse / total : 0.0;
  return true;
}

static inline FlowCalResult flowCalSolve(const FlowCalibrator &cal) {
  FlowCalResult r;
  // 모든 후보는 같은 광류 구간을 보므로 샘플 수·자극은 후보 0 으로 판정한다.
  const FlowCalAccum &a0 = cal.acc[0];
  r.samples = a0.n;
  if (a0.n < FLOW_CAL_MIN_SAMPLES) {
    r.reason = FLOW_CAL_TOO_FEW;
    return r;
  }

  const double n = (double)a0.n;
  const double cxx = a0.sgg_xx / n, cxy = a0.sgg_xy / n, cyy = a0.sgg_yy / n;
  const double mean = 0.5 * (cxx + cyy);
  const double half = sqrt(0.25 * (cxx - cyy) * (cxx - cyy) + cxy * cxy);
  const double eigMin = mean - half, eigMax = mean + half;
  r.min_eig_rad2 = (float)eigMin;
  r.cond = eigMin > 0.0 ? (float)(eigMax / eigMin) : INFINITY;
  r.corr = (cxx > 0.0 && cyy > 0.0) ? (float)(cxy / sqrt(cxx * cyy)) : 0.0f;
  // 한 축만 흔들었으면 그 축의 분산부터 작다. 두 축 모두 흔들었는데 같이
  // 움직였으면 축별 분산은 충분해도 최소 고유값·조건수·상관에 걸린다.
  if (cxx < FLOW_CAL_MIN_EIG_RAD2 || cyy < FLOW_CAL_MIN_EIG_RAD2) {
    r.reason = FLOW_CAL_LOW_EXCITATION;
    return r;
  }
  if (eigMin < FLOW_CAL_MIN_EIG_RAD2 || r.cond > FLOW_CAL_MAX_COND ||
      fabs(r.corr) > FLOW_CAL_MAX_ABS_CORR) {
    r.reason = FLOW_CAL_COLLINEAR;
    return r;
  }

  int best = -1;
  double bestResidual = 0.0;
  double bestM[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
  for (int k = 0; k < FLOW_CAL_DELAY_CANDIDATES; k++) {
    double m[2][2];
    double residual;
    if (cal.acc[k].n != a0.n || !flowCalFitCandidate(cal.acc[k], m, residual)) {
      continue;
    }
    if (best < 0 || residual < bestResidual) {
      best = k;
      bestResidual = residual;
      for (int i = 0; i < 2; i++) {
        bestM[i][0] = m[i][0];
        bestM[i][1] = m[i][1];
      }
    }
  }
  if (best < 0) {
    r.reason = FLOW_CAL_COLLINEAR;
    return r;
  }
  r.delay_us = (uint32_t)best * FLOW_CAL_DELAY_STEP_US;
  r.residual = (float)bestResidual;
  for (int i = 0; i < 2; i++) {
    r.m[i][0] = (float)bestM[i][0];
    r.m[i][1] = (float)bestM[i][1];
  }
  if (bestResidual > FLOW_CAL_MAX_RESIDUAL) {
    r.reason = FLOW_CAL_POOR_FIT;
    return r;
  }

  // 각 행은 한 기체축에 거의 혼자 대응해야 하고, 두 행은 서로 다른 축이어야 한다.
  int bigCol[2];
  for (int i = 0; i < 2; i++) {
    const double a = fabs(bestM[i][0]), b = fabs(bestM[i][1]);
    const double big = a > b ? a : b, small = a > b ? b : a;
    bigCol[i] = a > b ? 0 : 1;
    if (!(big > 0.0) || small / big > FLOW_CAL_MAX_OFFAXIS_RATIO) {
      r.reason = FLOW_CAL_NOT_PERMUTATION;
      return r;
    }
  }
  if (bigCol[0] == bigCol[1]) {
    r.reason = FLOW_CAL_NOT_PERMUTATION;
    return r;
  }
  for (int i = 0; i < 2; i++) {
    const double big = fabs(bestM[i][bigCol[i]]);
    if (big < FLOW_CAL_MIN_COUNTS_PER_RAD || big > FLOW_CAL_MAX_COUNTS_PER_RAD) {
      r.reason = FLOW_CAL_SCALE_RANGE;
      return r;
    }
  }
  r.ok = true;
  r.reason = FLOW_CAL_OK;
  return r;
}
