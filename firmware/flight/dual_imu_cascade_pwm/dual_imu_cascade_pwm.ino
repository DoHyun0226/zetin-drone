#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM42670P.h>
#include <DFRobot_BMM350.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <errno.h>
#include <float.h>

// 프로젝트 헤더: 하드웨어와 무관한 계산 로직만 담아 PC 네이티브 테스트에서도 그대로 쓴다
#include "mag_yaw_fusion.h"     // 자기계 heading 계산 + yaw 상보필터
#include "yaw_command.h"        // yaw 바깥 루프: heading 잠금/스틱 각속도 모드
#include "failsafe_land.h"      // RC 끊김 시 자동착륙, 호버 스로틀 추정
#include "msp_sensor.h"         // 3901-L0X 광류/라이다 UART 파서
#include "control_allocator.h"  // PID 출력 → 모터 4개 분배 (포화 시 우선순위)
#include "yaw_authority.h"      // 저스로틀 yaw 권한 부족 감시, 조건부 적분

// [IMU 원시 데이터 스트림(ZIMU) 정리]
//  pid_task(1kHz)가 매 tick 두 IMU의 레지스터 원값을 링 버퍼에 넣고(생산자),
//  udp_task가 최대 50개씩 묶어 UDP 바이너리 패킷으로 PC에 보낸다(소비자)
//  20Hz CSV 텔레메트리로는 안 보이는 1kHz 진동/지연을 오프라인으로 분석하려는 용도
static const uint32_t IMU_RAW_RING_SIZE = 512;                    // 링 버퍼 칸 수 (2의 거듭제곱이어야 아래 MASK가 동작)
static const uint32_t IMU_RAW_RING_MASK = IMU_RAW_RING_SIZE - 1;  // index & MASK = index % 512 (나눗셈 없이 빠르게)
static const uint8_t IMU_RAW_BATCH_MAX = 50;                      // 한 패킷에 담는 최대 샘플 수 (50ms 분량)
// v2: ImuRawSample 에 failsafe_phase 추가. 디코더는 v1 도 계속 읽는다.
static const uint8_t IMU_RAW_VERSION = 2;


/*
GCC/Clang 컴파일러 확장 문법-->컴파일러에게 이 함수, 변수, 구조체는 특별하게 다뤄달라고 지시할 때 사용
기본 형태 
void foo(void) __attribute__((속성));
int x __attribute__((속성)) = 0;

 - 괄호 두 겹으로 쓰는 것이 규칙, 속성 여러 개는 쉼표로 이어 씀
 - packed: 구조체 멤버 사이 패딩을 없애는 용도-> 통신 패킷, 센서 레지스터 구조체 등에 사용
*/

struct __attribute__((packed)) ImuRawSample { 
  uint16_t dt_us;         // 2B  직전 샘플과의 시간 간격 (µs)
  int16_t imu1_gyro[3];   // 6B  IMU1 자이로 x,y,z (레지스터 원값)
  int16_t imu1_accel[3];  // 6B  IMU1 가속도
  int16_t imu2_gyro[3];   // 6B  IMU2 자이로
  int16_t imu2_accel[3];  // 6B  IMU2 가속도
  // v2. 1kHz 로그를 자동착륙 위상으로 자를 수 있어야 오프라인에서 착지 판별식
  // 후보를 평가할 수 있다. .bin 은 펌웨어 micros(), 20Hz CSV 는 PC 벽시계라
  // 공통 키가 없어서 2026-08-01 분석이 여기서 막혔다.
  uint8_t failsafe_phase; // 1B  그 순간의 자동착륙 단계
};                        // 합계 27B

struct __attribute__((packed)) ImuRawHeader { // 패킷 머리(20B)
  char magic[4];        // "ZIMU" - 받는 쪽이 어떤 패킷인지 식별
  uint8_t version;      // 2 - 형식이 바뀌어도 디코더가 구분할 수 있게
  uint8_t n_samples;    // 이 패킷에 든 샘플 수(1~50)
  uint16_t reserved;    // 예약 (0)
  uint32_t batch_seq;   // 패킷 일련번호 -> 번호가 비면 UDP 패킷 손실
  uint32_t base_t_us;   // 첫 샘플의 절대 시각
  uint32_t dropped;     // 링 버퍼가 넘쳐서 버린 샘플 누적 개수
};

struct __attribute__((packed)) ImuRawDatagram { // 실제 전송 단위
  ImuRawHeader header;                          // 20B
  ImuRawSample samples[IMU_RAW_BATCH_MAX];      // 27B * 50 = 1350B
};

struct __attribute__((packed)) ImuCalDatagram {
  // ZCAL: ZIMU의 원시값을 실제 물리 단위로 바꾸는 데 필요한 보정 상수들을 담은 패킷
  // ZIMU에 담긴 건 센서 레지스터 원값-> 이 숫자만으로 실제 회전 속도, 가속도 알 수 없음-> 펌웨어 안에서 '실제값 = (원값 * 스케일) - 바이어스'로 바꿔서 사용
  // raw 스트림이 켜져 있는 동안 1초에 1번 전송 (UDP 손실, PC 로그 중간 시작 대비)
  char magic[4];            // 4B  magic = "ZCAL": 보정값 패킷이라는 표시
  uint8_t version;          // 1B  형식 버전(현재 2)
  uint8_t reserved[3];      // 3B  예비 칸. 뒤의 float들을 4의 배수 위치(오프셋 8)부터 시작하게 맞춤
  float gyro_bias1[3];      // 12B IMU1 자이로 바이어스 x,y,z (dps). 부팅 시 정지 상태에서 측정한 영점 오차
  float gyro_bias2[3];      // 12B IMU2 자이로 바이어스 x,y,z (dps). IMU1 센서축 기준으로 표현
  float accel_scale1;       // 4B  IMU1 가속도 보정 배율. 부팅 시 정지 상태 |a| = 1g가 되도록 측정
  float accel_scale2;       // 4B  IMU2 가속도 보정 배율
  float gyro_scale;         // 4B  자이로 원값(LSB) -> dps 변환 상수 (GYRO_SCALE, FSR ±2000dps 기준)
  float accel_scale;        // 4B  가속도 원값(LSB) -> g 변환 상수 (ACCEL_SCALE, FSR ±16g 기준)
  float imu2_sign[3];       // 12B IMU2 축 부호 x,y,z (+1/-1). IMU2 장착 방향을 IMU1 축에 맞추는 값
};                          // 합계 60B

// Arduino's generated function prototypes are inserted before later global
// declarations, so snapshot return types must be declared in this top block.
// 모터 분배기(allocator) 결과를 pid_task → sendTelemetry로 넘기는 한 tick 스냅샷 (텔레메트리 필드 66~75)
struct AllocationTelemetrySample {
  bool scaled;               // 이번 tick 자세 명령이 축소됐는가 (rp_scale<1 또는 yaw_scale<1)
  float rp_scale;            // roll/pitch 배율 (1 = 그대로)
  float yaw_scale;           // yaw 배율 λ (1 = 그대로, 0 = yaw 포기)
  float collective_us;       // 실제 적용된 스로틀 (µs)
  float pid_roll_us;         // 축별 PID 전체 출력 (µs)
  float pid_pitch_us;
  float pid_yaw_us;
  float i_roll_us;           // 그중 I항 기여 (µs)
  float i_pitch_us;
  float i_yaw_us;
  int yaw_authority_state;   // YawAuthorityState (0 NORMAL, 1 LIMITED, 2 RECOVERING)
};

// static_assert(조건, "틀렸을 때 보여줄 메시지") --> 컴파일할 때 조건을 검사해서, 틀리면 컴파일 자체를 실패시키는 C++ 문법
static_assert(sizeof(ImuRawSample) == 27, "ZIMU v2 sample must be 27 bytes");
static_assert(sizeof(ImuRawHeader) == 20, "ZIMU header must be 20 bytes");
static_assert(sizeof(ImuRawDatagram) == 1370,
              "maximum ZIMU v2 datagram must be 1370 bytes");
static_assert(sizeof(ImuCalDatagram) == 60, "ZCAL datagram must be 60 bytes");

struct ImuRawRing {
  ImuRawSample samples[IMU_RAW_RING_SIZE];  // 원형으로 재사용하는 저장 공간 (512칸)
  volatile uint32_t head = 0;               // 쓰기 위치 (Core 1만 증가시킴)
  volatile uint32_t tail = 0;               // 읽기 위치 (Core 0만 증가시킴)
  volatile uint32_t dropped = 0;            // 가득 차서 버린 수
  volatile uint32_t first_t_us = 0;         // 버퍼가 비었을 때 들어온 첫 샘플 시각
};

static ImuRawRing imuRawRing;
static ImuRawSample imuRawBatch[IMU_RAW_BATCH_MAX]; // 링에서 꺼낸 샘플을 잠깐 모아 두는 작업대
// IMU_RAW_BATCH_MAX = 50 
static ImuRawDatagram imuRawDatagram;               // 헤더를 붙인 보낼 패킷--> udp.write()에 그대로 넘김
static ImuCalDatagram imuCalDatagram;               // 보정값 패킷(ZCAL)
volatile bool raw_stream_enabled = true;            // 기본 ON
// volatile: 컴파일러가 임의로 값을 레지스터에 캐싱, 코드를 최적화하지 않고, 매번 실제 메모리에 직접 접근하도록 지시하는 키워드
static bool rawProducerTimeValid = false;           // 생산자: 기준 시각이 있나?
static uint32_t rawProducerLastUs = 0;              // 생산자: 직전에 넣은 샘플의 시각
static bool rawConsumerTimeValid = false;           // 소비자: 기준 시각이 있나?
static uint32_t rawConsumerLastUs = 0;              // 소비자: 직전에 꺼낸 샘플의 시각
// 생산자 쪽(pid_task, 넣을 때), 소비자 쪽(udp_task, 꺼내서 보낼 때)-->패킷의 첫 샘플 시각이 헤더의 base_t_us가 됨
static uint32_t rawBatchSeq = 0;          // 역할: 패킷 일련번호. 보낼 때마다 +1, 사용처: PC가 번호가 빈 걸 보고 UDP 손실을 감지
static uint32_t rawLastSendMs = 0;        // 역할: 마지막 ZIMU 전송 시각, 사용처: 50개가 안 모여도 50ms가 지나면 보냄
static uint32_t rawLastCalMs = 0;         // 마지막 ZCAL 전송 시각: 1초마다 보정값 재전송

// [WiFi 지연 디버그] 빌드 옵션으로 WIFI_LATENCY_DEBUG=1을 주면 켜진다 (기본 0 = 아래 코드가 통째로 빠짐)
//  udp_task/pid_task가 기준(30ms / 5ms)보다 오래 걸린 순간을 링 버퍼에 기록하고 loop()가 시리얼로 출력한다
#ifndef WIFI_LATENCY_DEBUG
#define WIFI_LATENCY_DEBUG 0
#endif
#if WIFI_LATENCY_DEBUG
#ifndef WIFI_LATENCY_UDP_THRESHOLD_US
#define WIFI_LATENCY_UDP_THRESHOLD_US 30000U
#endif
#ifndef WIFI_LATENCY_PID_THRESHOLD_US
#define WIFI_LATENCY_PID_THRESHOLD_US 5000U
#endif

enum LatKind : uint8_t {
  LAT_UDP_LOOP,
  LAT_UDP_PARSE,
  LAT_UDP_SEND,
  LAT_PID
};

struct LatEvt {
  uint32_t t_ms;
  uint32_t dur_us;
  uint8_t kind;
};

static const uint32_t LAT_RING_SIZE = 128;
static const uint32_t LAT_RING_MASK = LAT_RING_SIZE - 1;

struct LatRing {
  LatEvt events[LAT_RING_SIZE];
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t dropped = 0;
};

static LatRing udpLatRing;
static LatRing pidLatRing;

static uint32_t latUdpMaxUs = 0;
static uint32_t latPidMaxUs = 0;
static uint32_t latUdpLoopOver30 = 0;
static uint32_t latUdpLoopOver100 = 0;
static uint32_t latUdpLoopOver300 = 0;
static uint32_t latPidOver5 = 0;
static uint32_t latLastSummaryMs = 0;
#endif
// ==========================================================
// 1. 튜닝 파라미터
// ==========================================================
// [Cascade 제어 정리] 두 겹으로 겹친 제어기
//  목표 각도 ─▶ [Outer: Kp_Angle_*] ─▶ 목표 각속도 ─▶ [Inner: K*_Rate_*] ─▶ 모터 출력 차이
//     (°)          250Hz                  (dps)            1kHz               (µs)
//  Kp_Angle_* : 각도 오차를 보고 "목표 각속도"를 정할 때 사용 (예: 5° 틀어짐 → 30dps로 돌아가라)
//  K*_Rate_*  : 목표 각속도와 실제 각속도의 차이를 보고, 그걸 따라가도록 "모터 출력(µs)"을 정할 때 사용
//               (예: 30dps로 돌아야 하는데 지금 10dps → 모터 출력 차이를 이만큼 더 벌려라)
//  바깥 루프가 목표 각속도를 만들고, 안쪽 루프가 그 각속도를 실제로 만들어 낸다

// [Outer] 각도 P: 각도 오차 -> 목표 각속도(dps)
// 비행 중 노트북에서 같은 UDP 명령으로 실시간으로 바꿀 수 있고, pid_task가 매 tick 읽기 때문에 volatile로 선언
volatile float Kp_Angle_Roll  = 6.0f; // 단위 dps/° : 각도 오차 1°당 목표 각속도 6dps
volatile float Kp_Angle_Pitch = 6.0f;
volatile float Kp_Angle_Yaw   = 3.0f; // heading은 천천히 맞춰도 되고, 급하게 돌리면 모터 여유를 많이 먹어 절반

// [Inner] 각속도 PID: 각속도 오차 -> 모터 파워
//  Kp_Rate (µs/dps)     : 지금 각속도 오차에 비례해 즉시 반응
//  Ki_Rate (µs/(dps·s)) : 오래 남는 오차를 쌓아서 제거 (CG 치우침, 모터 편차)
//  Kd_Rate (µs/(dps/s)) : 각속도가 급하게 변하면 제동 → 오버슈트 감소
//  yaw는 반토크라는 약한 힘으로 돌아서 Kp가 3배(1.5), 원래 둔하게 움직여 D항은 노이즈만 키우므로 0
// Ki_Rate_Roll/Pitch: 0.005 -> 0.05 (호스트 SIL 근거, commit 735f77d). 0.005는
// 일정 CG 치우침 외란에서 10초 후 ±50µs 클램프의 ~1.4%만 써 적분기가 사실상
// 비활성이었다(파라미터 무관하게 강건한 결론). 0.05는 적분기를 활성화하되
// 클램프의 <5µs만 쓰는 보수적 시작값이다. SIL의 완전트림 값(0.5)은 실기 지연·
// 노이즈에서 진동 위험이 있어 일부러 피했고, 최종값은 테더 벤치에서 상향 튜닝한다.
volatile float Kp_Rate_Roll  = 0.50f, Ki_Rate_Roll  = 0.05f, Kd_Rate_Roll  = 0.015f; 
volatile float Kp_Rate_Pitch = 0.50f, Ki_Rate_Pitch = 0.05f, Kd_Rate_Pitch = 0.015f;
volatile float Kp_Rate_Yaw   = 1.50f, Ki_Rate_Yaw   = 0.05f,  Kd_Rate_Yaw   = 0.0f;

volatile int  base_throttle = 1000; // 모터 4개의 기본 출력
volatile int  min_throttle  = 1050; // 각 모터 출력의 하한
volatile int  max_throttle  = 1300; // 각 모터 출력의 상한
volatile bool yaw_hold_override = false;   // "yaw 1": heading 고정 + 재슬레이빙 금지
// true면 updateYawOuter()의 hold가 항상 true:
//  - 요 스틱(rcr yaw_rate)은 무시되고, targetAngleZ를 현재 heading으로 따라가게(슬레이빙) 하지 않는다
//  - 요 목표 각속도 = wrap(targetAngleZ - angleZ) * Kp_Angle_Yaw (±MAX_TARGET_RATE_YAW)
//    → 외란으로 밀려도 원래 방위로 복원, rc 명령의 yaw 필드가 "절대 heading 지정"으로 동작
//  - rc로 따로 안 주면 시동 순간의 heading이 목표가 된다 (잠금 중엔 매 tick targetAngleZ = angleZ)
// 켜기: "yaw 1"은 시동 해제(safety_lock) 상태에서만 수락. 끄기: "yaw 0" 또는 잠금 진입 시 자동 해제
// 주의: 목표-현재 차이가 크면 최대 180dps로 급회전(슬램), angleZ 드리프트도 그대로 따라 돈다 → 벤치 시험용

volatile bool yaw_hold_now = false;   // 텔레메트리용: 현재 heading 잠금 중인가
volatile int yaw_authority_state = YAW_AUTH_NORMAL;   // yaw 권한 상태 (yaw_authority.h). pid_task가 쓰고 텔레메트리가 읽는다

const int MAG_THROTTLE_REF_US = 1000;   // 보정 기준(모터 off 근처), 
// 모터 전류 간섭 보정: body-frame ΔB(µT)는 base_throttle에 비례한다고 근사한다.
// hard/soft-iron 보정과 body 축 변환을 거친 mag에서, 틸트 보정 전에 뺀다.
// mag.x -= mag_comp_x*(base_throttle-REF) 등. µT/µs. 런타임 `magc x y z`. 0=off.
// 벤치 특성화 2026-07-27 (log 023805, 검증 024330): throttle 간섭 +3.64→+0.02°/100µs.
// 2026-08-04: soft-iron 행렬 도입으로 이 계수가 사는 좌표계가 바뀌었다. 간섭은
// raw 축의 물리적 ΔB이고 보정은 선형이므로 새 계수 = W @ 기존계수로 환산했다.
// 같은 날 log 032922의 벤치 4단계 재검증 통과 기록이 있다. 방위·배터리 조건을
// 넓힌 검증이나 현재 하드웨어의 재시험을 뜻하지 않는다. docs/bmm350_yaw_bench_test.md.
volatile float mag_comp_x =  0.007467272f;  // µT/µs // 자기 보정 계수
volatile float mag_comp_y = -0.001149868f;
volatile float mag_comp_z = -0.000526263f;
// [모터 전류 자기 간섭 보정 정리]
//  모터 전류가 만든 자기장이 섞이면 스로틀을 올릴수록 heading이 틀어진다 (보정 전 약 3.6°/100µs)
//  MAG_THROTTLE_REF_US(1000µs, ESC 정지 신호) = 전류 없음 → 간섭 0으로 보는 기준점
//  보정: magThrDelta = base_throttle - 1000, mag.축 -= mag_comp_축 × magThrDelta (축별 µT/µs 계수)
//  예) base_throttle 1300 → 300µs × 0.00747 ≈ x축 2.24µT 빼기 (지구 수평 자기장 수십 µT라 몇 도 차이)
//  한계: 개별 모터 출력이 아니라 공통 스로틀 기준이라, PID가 모터별로 다르게 줄 때의 간섭 차이는 못 잡는다

// ==========================================================
// 2. 시스템 상수
// ==========================================================
// 런타임에 바뀌지 않는 하드웨어/통신/안전 설정값. const라 비행 중 변경 불가(재빌드 필요)

// --- WiFi / UDP: 드론이 AP(공유기)가 되고 노트북이 접속한다 ---
const char* WIFI_SSID    = "Drone_Tuning";  // 드론이 만드는 WiFi 이름
const char* WIFI_PASS    = "12345678";      // WiFi 비밀번호 (8자 이상이어야 WPA2 AP가 열림)
const int   UDP_PORT     = 4210;            // 명령 수신/텔레메트리 송신 포트 (지상국과 같아야 함)
const int   WIFI_CHANNEL = 11;  // ch6 혼잡(인근 AP 다수) → 스캔상 가장 빈 채널로 이동

// --- 핀 배치 (ESP32-S3 GPIO 번호) ---
// 모터 PWM 출력 핀. F/R = 앞/뒤, L/R = 왼쪽/오른쪽. 믹서의 motor[0..3] 순서와 같다
const int pinM1   = 4;   // FL
const int pinM2   = 5;   // RR
const int pinM3   = 6;   // FR
const int pinM4   = 7;   // RL
// SPI 칩 선택(CS) 핀: SCK/MISO/MOSI는 두 IMU가 공유하고 CS로 누구와 통신할지 고른다
const int SPI_CS1 = 10;  // IMU1
const int SPI_CS2 = 9;   // IMU2
// Matek 3901-L0X (광류+라이다). 모듈 TX -> 이 핀, 3.3V 로직이라 직결한다.
// 모듈 RX 는 연결하지 않는다 — 우리는 보낼 것이 없고 수신 전용이다.
// 19/20 은 USB D-/D+ 라 피했고 스트래핑 핀도 피했다.
const int MSP_SENSOR_RX_PIN = 16;          // 3901-L0X 수신용 UART RX 핀 (Serial1)
const uint32_t MSP_SENSOR_BAUD = 115200;   // UART 통신 속도 (모듈 설정과 같아야 함)
// 이보다 오래 프레임이 없으면 quality 를 -1 로 보고한다. 얼어붙은 센서가
// 마지막 값으로 유효해 보이면 안 된다(MAG_STALE_MS 와 같은 이유).
const uint32_t MSP_SENSOR_STALE_MS = 500;
// udp_task 한 반복에서 처리할 바이트 상한. BMM350 의 블로킹 I2C 가 RC
// 수신큐를 넘치게 했던 전례가 있어 이 경로는 절대 블로킹하지 않는다.
// 실제 유량은 20Hz x ~15B = 300B/s 라 이 상한에 닿을 일이 없다.
const int MSP_SENSOR_MAX_BYTES_PER_TICK = 64;

// BMM350 자기계: I2C 핀과 센서 전원 스위치
const int BMM_SDA = 35;   // I2C 데이터
const int BMM_SCL = 36;   // I2C 클럭
const int LDO_SHD = 38;   // 자기계 전원 레귤레이터(LDO) 켜기 핀. initMagnetometer()에서 HIGH

// --- ESC PWM 신호 (LEDC 하드웨어 PWM) ---
// 400Hz = 2500µs 주기마다 1000~2000µs 펄스를 보낸다 (1000 정지, 2000 최대)
const int ESC_FREQ   = 400;                  // PWM 주파수 (Hz)
const int ESC_RES    = 14;                   // 듀티 분해능 14비트 = 0~16383 단계
const int ESC_PERIOD = 2500;                 // us = 1/400Hz. 400Hz PWM은 2500µs 주기
const int ESC_MAXDUTY = (1 << ESC_RES) - 1;  // magic number 제거
// writeMotor()에서 duty = µs × ESC_MAXDUTY / ESC_PERIOD 로 펄스 폭을 듀티 값으로 바꾼다

// --- 센서 단위 변환 (setup의 FSR 설정과 짝) ---
// 자이로 FSR ±2000dps → 16.4 LSB/(deg/s), 가속도 FSR ±16g → 2048 LSB/g
const float GYRO_SCALE   = 1.0f / 16.4f;     // raw -> deg/s
const float ACCEL_SCALE  = 1.0f / 2048.0f;   // raw -> g

// --- 자세/명령 제한 ---
const float SAFETY_ANGLE = 60.0f;            // 롤/피치가 이 각도(°)를 넘으면 OVER-TILT로 모터 정지
const float YAW_DEADZONE = 0.3f;             // |yaw 각속도| < 0.3dps면 angleZ 적분 안 함 (정지 중 드리프트 억제)
const float MAX_TARGET_ANGLE_RP = 30.0f;     // UDP 오입력에 대한 자세 명령 제한
const float TRIM_MAX_DEG = 10.0f;   // 트림 절대값 상한
const float MAX_TARGET_RATE_RP  = 300.0f;    // outer-loop 출력 제한 (deg/s)
const float MAX_TARGET_RATE_YAW = 180.0f;    // yaw 목표 각속도 상한 (deg/s)
// yaw 모드 판정 임계치. 벤치에서 조정한다(설계 문서 §1 참조).
const float YAW_RATE_DEADZONE   = 3.0f;    // 명령 각속도 절댓값이 이 값 미만이면 스틱 중립
const float YAW_HOLD_SETTLE_DPS = 10.0f;   // bodyGz 절댓값이 이 값 미만이면 새 잠금 진입 가능
// --- 자기계(나침반) 융합 ---
// 250Hz outer loop에서 K=0.001 -> tau ≈ 0.004/K = 4s.
const float K_MAG = 0.001f;                  // angleZ를 나침반 방향으로 끌어당기는 비율 (작을수록 천천히)
const uint32_t MAG_SAMPLE_PERIOD_MS = 20;    // BMM350 read = 50Hz, core 0 only
const uint32_t MAG_STALE_MS = 100;           // 이보다 오래된 자기계 값은 융합에 쓰지 않음(ms)

// scripts/magcal_fit.py 출력값을 아래 두 상수에 붙여 넣는다.
// 2026-08-04 캘리브 (log flight_log_2026-08-04_023840.csv, 1255샘플 중 inlier
// 1119, 커버리지비 2.08). 이전 min/max offset은 여기서 6.79µT 벗어나 있었고
// 그것은 수평성분 32.2µT 대비 최대 ±12.2° heading 오차였다.
// 교차검증: 무작위 절반 분할 8회의 중심 차이 중앙값 0.29µT (시간 분할은
// 후반 커버리지비가 39.6이라 b_z를 구속하지 못하므로 쓰면 안 된다).
// 물리 정합성: 피팅 반경 50.10µT vs 한국 총강도 실측 ~50.4µT (오차 0.6%).
// hard-iron: 기체에 붙은 고정 자석 성분 (µT). 측정값에서 뺀다
const float MAG_HARD_IRON[3] = {-3.457566904f, 4.794903225f, -17.307261041f};
// soft-iron: 금속 때문에 타원으로 찌그러진 측정을 원으로 펴는 3x3 행렬
const float MAG_SOFT_IRON[3][3] = {
  {0.998555071f, 0.005825468f, 0.018437852f},
  {0.005825468f, 0.976739944f, 0.006050833f},
  {0.018437852f, 0.006050833f, 1.026752877f},
}; 

// 여기까지 읽음 --> 다음에 어디까지 읽었는 지 질문할 때 해당 줄 번호 반환할 것

// BMM350 축이 기체 축과 다르면 벤치 sign test 뒤 여기만 바꾼다.
const float MAG_BODY_SIGN_X = 1.0f;
const float MAG_BODY_SIGN_Y = 1.0f;
const float MAG_BODY_SIGN_Z = 1.0f;

// --- IMU 장착 방향 부호 ---
// IMU2는 x,z축이 IMU1 대비 반전 (y는 동일). 첫 파일 기준 그대로.
// 원값에 곱해서 IMU2를 IMU1 축 기준으로 맞춘 뒤 두 값을 평균/비교한다
const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;
static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

// --- 안전/redundancy 임계값 (하드웨어 맞춰 튜닝 필요) ---
const uint32_t RC_TIMEOUT_MS      = 500;   // 조종 명령(rc/rcr)이 이 시간 동안 없으면 페일세이프 (20Hz면 10개 연속 손실)
// 아래 failsafe 상수는 전부 벤치 조정 대상이다.
// [페일세이프 자동착륙 요약] RC가 끊기면 hover_est - FS_DESCENT_DELTA_US 로 스로틀을
// 낮춰 천천히 내려가고, FS_MAX_MS가 지나면 모터를 끈다. 아래 FS_PROBE_* 는 착지 감지
// 실험용 값으로 현재 판정에는 쓰이지 않고 텔레메트리 기록에만 쓰인다.
constexpr int CTRL_MARGIN = 150;              // th 명령 시 min/max = base ∓ 150µs. PID가 쓸 수 있는 여유
const int      FS_DESCENT_DELTA_US = 60;      // 자동착륙: 호버 스로틀보다 이만큼 낮춰 하강
static_assert(FS_DESCENT_DELTA_US < CTRL_MARGIN,
              "FS_DESCENT_DELTA_US must be less than CTRL_MARGIN");
// 컴파일 타임 가드는 delta < CTRL_MARGIN만 검사한다. 실제 하한은
// delta < min(CTRL_MARGIN, entry_throttle - 1050)이며 entry_throttle은
// 런타임 값이라 static_assert로 잡을 수 없다. 예: entry=1150이면 delta=120도
// 이미 collective 하한에 걸려 실제 하강이 일어나지 않는다.
const uint32_t FS_MIN_DESCEND_MS   = 1000;    // 하강 시작 후 이 시간이 지나야 착지 프로브 시작
constexpr float FS_LAND_LPF_ALPHA = 0.03f;   // 1kHz에서 약 5Hz (착지 감지용 |accel| 필터)
// 능동 프로브 상수는 모두 Stage E 벤치 조정 대상이다. 호버 1340us는 아이들
// 1000us 대비 340us이고, 그 11.8%인 40us 딥은 명목 질량의 기존 동작을
// 보존한다. 질량이 달라져도 같은 추력 비율을 유지하도록 hover_est 기준으로
// 매 자동착륙 진입 때 계산한다. 런타임 결과는 측정 가능한 20us 이상,
// CTRL_MARGIN 미만으로 clamp하고 clamp 여부를 진입 로그로 공개한다.
// 2026-08-01: 딥을 2배로 올려 공중 응답만 키우려 했으나 **이 기체에서는
// 불가능**하다. 전달 가능한 딥의 상한은
//     (hover_est - FS_DESCENT_DELTA_US) - CTRL_MARGIN - 1050
// 이고 hover_est=1290 에서 그 값이 30us 인데 현재 딥이 이미 34us 다. 더 키우면
// 믹서가 collective 를 밀어올려 전달률이 FS_PROBE_MIN_DELIVERY_FRAC(80%) 아래로
// 떨어지고 BLOCKED 가 된다(2배면 44%). hover_est 1340 이상이면 여유가 생기지만
// 우리 호버는 1290 이다. R13 회귀 시험이 이 한계를 잡아냈다.
// 딥을 늘리려면 CTRL_MARGIN 이나 1050 하한을 먼저 손봐야 하며 둘 다 안전
// 관련이라 별도 검토가 필요하다 → docs/failsafe_land_research.md
// [프로브] 스로틀을 잠깐 낮춰(딥) 기체가 반응하면 공중, 반응 없으면 지면으로 본다
constexpr float FS_PROBE_DIP_FRAC      = 0.118f;   // 딥 크기 = (hover_est - 1000) × 11.8%
const int      FS_PROBE_DIP_MIN_US      = 20;      // 딥 최소 크기 (µs)
const uint32_t FS_PROBE_PERIOD_MS       = 400;     // 프로브 반복 주기
const uint32_t FS_PROBE_DIP_MS          = 120;     // 딥 유지 시간
const uint32_t FS_PROBE_SAMPLE_DELAY_MS = 30;      // 딥 시작 후 이만큼 기다렸다가 반응 측정 시작
// ⚠️ 이 값은 **판정에 쓰이지 않는다.** 프로브는 텔레메트리 41~43 기록 전용이고
// 착지 컷은 시간 기반 백스톱이 한다(아래 FS_MAX_MS, 그리고 하강 블록의
// `const bool landed = false`).
//
// 0.06 → 0.03 변경의 근거는 **철회됐다**(07231eb). 근거였던 "2026-08-01 공중
// 오판 9회 중 4회"는 20Hz 단일 샘플을 자유낙하로 오독한 것이고, 그 하강은
// 실제로는 지면에서 났다 — 즉 공중 오판은 일어나지 않았다. 오히려 0.03으로
// 내린 뒤 015448에서 지면 응답 10회가 전부 임계 위가 되어 확정이 불가능해졌다.
// 되돌릴 값이지만 프로브가 판정에서 빠진 지금은 무의미하므로 그대로 둔다.
// 전말은 docs/failsafe_land_research.md.
constexpr float FS_PROBE_RESPONSE_G    = 0.03f;    // 이보다 작은 가속도 변화(g)면 "반응 없음"
// 응답은 전달 딥에 비례한다. 기존 1.5배 명목 응답 보장에서 80%를 요구하면
// 임계 대비 1.2배가 남지만, 50% 전달은 임계 아래라 판정 근거가 사라진다.
constexpr float FS_PROBE_MIN_DELIVERY_FRAC = 0.80f; // 요청한 딥의 80% 이상이 실제 모터에 반영돼야 유효한 프로브
// 계단 응답은 response=FS_PROBE_DIP_FRAC*(1-exp(-FS_PROBE_DIP_MS*
// FS_LAND_LPF_ALPHA/1ms))로 LPF와 결합된다. V2 SIL 실측은 alpha=0.03에서
// 0.1146g, 0.01에서 0.0789g, 0.005에서 0.0464g였다. 따라서 alpha를 낮추면
// 노이즈뿐 아니라 프로브 감도도 같이 낮아진다. 딥은 최소 3τ(95% 응답) 동안
// 지속하고, 그때 관측 가능한 응답은 판정 임계의 1.5배보다 커야 한다.
static_assert(FS_PROBE_DIP_MS * FS_LAND_LPF_ALPHA >= 3.0f,
              "probe dip must span at least three LPF time constants");
static_assert(FS_PROBE_DIP_FRAC * 0.95f > 1.5f * FS_PROBE_RESPONSE_G,
              "observable probe response must exceed the decision margin");
static_assert(
    FS_PROBE_MIN_DELIVERY_FRAC * FS_PROBE_DIP_FRAC * 0.95f
        > 1.2f * FS_PROBE_RESPONSE_G,
    "minimum delivered probe must preserve the response margin");
const float    FS_LAND_ACCEL_TOL_G      = 0.10f;   // 착지 후보 조건: |accel|이 1g ± 0.1g 안 (정지 상태)
// ⚠️ 이 값도 **판정에 쓰이지 않는다** — 프로브 상태(텔레메트리 41~42)를 계산할
// 때만 들어간다.
//
// 2에서 4로 올린 근거였던 "2026-08-01 공중에서 CUT_LANDED가 났다"는 관측은
// **철회됐다**(07231eb). 그 컷은 지면에서 났고 정상 동작이었다. 다만 함께
// 측정한 사실 하나는 유효하다: 딥이 없어도 호버 중 120ms 창의 |accel| 낙폭이
// p95 0.0904g로 임계 0.06g보다 크다 — 교란만으로 프로브 신호 전체 크기가
// 나온다. 연속 판정 횟수를 늘려도 판별식이 갖지 못한 정보를 만들 수는 없고,
// 그래서 프로브 자체가 판정에서 빠졌다. 근본 대책은
// docs/failsafe_land_research.md (거리계로 착지를 측정하는 쪽).
const uint8_t  FS_PROBE_CONFIRM_N       = 4;       // "반응 없음"이 이 횟수 연속이면 착지 후보
// 프로브가 판정에서 빠진 뒤로 이 값은 **하강 시간 예산**이다. 고도나 접지를
// 확인하지 않으므로 예산이 부족하면 공중에서도 TIMEOUT으로 모터가 꺼진다.
//
// 2026-08-01 실측 (접지 충격 임펄스 적분으로 접지 속도를 재고 등가속 모델
// v=a·t, h=½a·t² 로 역산):
//   023021  t=0.80s v=0.76m/s → a=0.95   h=0.30m
//   023226  t=0.82s v=1.30m/s → a=1.59   h=0.53m
//   023904  t=1.76s v=2.11m/s → a=1.20   h=1.86m   ← 환경상 최고 고도
// 평균 a≈1.24 m/s² (추력 부족 60/290=21%=2.06 m/s² 에서 항력만큼 감소).
// 아래는 이 평균 가속도를 일정하다고 가정한 외삽이며 실측 고도 한계가 아니다:
//   2.0s→2.49m  2.5s→3.89m  3.0s→5.60m  3.5s→7.62m
// 3.0s에서 실제로 5.60m를 내려온다는 보장은 없다. 아래 static_assert는
// 프로브 시간 배치만 검사한다(2720ms 초과 필요). 2.5s는 컴파일되지 않는다.
const uint32_t FS_MAX_MS           = 3000;          // 자동착륙 최대 시간. 지나면 모터 정지 (현재 실제 종료 조건)
const uint32_t FS_RESUME_MAX_MS    = 3U * FS_MAX_MS; // 첫 자동착륙 진입 후 이 시간이 지나면 resume 거부
static_assert(FS_PROBE_DIP_MIN_US < CTRL_MARGIN,
              "FS_PROBE_DIP_MIN_US must be less than CTRL_MARGIN");
static_assert(FS_PROBE_SAMPLE_DELAY_MS < FS_PROBE_DIP_MS,
              "probe sample delay must be shorter than the dip");
static_assert(FS_PROBE_DIP_MS < FS_PROBE_PERIOD_MS,
              "probe dip must finish before the next probe");
// 검출기를 제어에 쓰던 당시의 시간 배치 가드다. 현재 반환값은 기록에만 쓴다.
// 당시 요구는 "접지 후 confirm_n회 연속 무반응이 백스톱 전에 끝날 것"인데
// 접지 시각은 런타임 값이라 컴파일 타임에 못 쓴다. 대신 필요조건만 건다:
// 접지 전에 버려지는 공중 프로브를 최소 1회 허용하고도 confirm_n회가 더
// 들어가야 한다 = 총 confirm_n+1회. 이건 필요조건일 뿐 충분조건이 아니다 —
// 접지가 늦으면 여전히 타임아웃이 날 수 있고, 그쪽은 SIL V1/V2/V4가 잡는다.
// (confirm_n-1로 쓰면 period=2000에서 3120<5000으로 통과하지만 SIL은 3건
//  타임아웃이었다. 그 오차를 변조 검사로 발견해 고친 식이다.)
static_assert(FS_MIN_DESCEND_MS
                  + FS_PROBE_PERIOD_MS * FS_PROBE_CONFIRM_N
                  + FS_PROBE_DIP_MS < FS_MAX_MS,
              "landing must be confirmable before the backstop cut");
// 위 프로브 상수들을 한 구조체로 묶어 failsafe_land.h의 updateLandDetector()에 넘긴다
const LandProbeConfig FS_LAND_PROBE_CONFIG = {
    FS_LAND_LPF_ALPHA,
    FS_MIN_DESCEND_MS,
    FS_PROBE_PERIOD_MS,
    FS_PROBE_DIP_MS,
    FS_PROBE_SAMPLE_DELAY_MS,
    FS_PROBE_RESPONSE_G,
    FS_LAND_ACCEL_TOL_G,
    FS_PROBE_CONFIRM_N,
    FS_PROBE_DIP_MIN_US,
};
// 아래 호버 추정 상수는 모두 Stage E-0 벤치 조정 대상이다. 10°/±0.05g는
// 자세·수직 정상상태만 받는 보수적 시작값이고, 3초 LPF는 요구 범위(2~5초)의
// 중앙값이다. 1.5초 누적은 짧은 스로틀 통과를 호버로 확정하지 않으면서
// 실측 상한 모델인 0.06g 저주파 진동에서도 3초 안에 유효해지는 시작값이다.
// [호버 추정] 수평·정지 상태의 스로틀을 천천히 평균내 "떠 있는 데 필요한 스로틀"(hover_est)을 학습
const float    HOVER_LEVEL_MAX_DEG    = 10.0f;   // 롤/피치가 이 안일 때만 호버 샘플로 인정
const float    HOVER_ACCEL_TOL_G      = 0.05f;   // |accel|이 1g ± 0.05g 안일 때만 (상승/하강 중 제외)
const int      HOVER_MIN_THROTTLE_US  = 1150;    // 이 스로틀 이하는 지상으로 보고 제외
// hover_est 하한이 이 값보다 낮으면 하강값에서 딥을 뺀 스로틀이 1000us
// 아래가 되어 프로브가 UNAVAILABLE에 고정된다. 현재 컷은 프로브 가용성과
// 무관하게 FS_MAX_MS 타임아웃을 사용하지만 진단용 딥의 가용성은 유지한다.
static_assert(
    HOVER_MIN_THROTTLE_US
        >= 1000 + FS_DESCENT_DELTA_US + FS_PROBE_DIP_MIN_US,
    "hover estimate floor must keep the probe dip above idle");
const float    HOVER_LPF_TAU_S        = 3.0f;    // 호버 평균 필터 시정수 (초)
const uint32_t HOVER_VALID_MS         = 1500;    // 조건을 만족한 시간이 누적 1.5초 넘으면 hover_valid = true
// 실측 호버는 약 1340us다. 무장 직후 spool-up 로그(041032)는 1100~1190us
// 구간이 전체 무장 시간의 9.4%였으므로, 지상 즉시컷은 보수적으로 1150us
// 이하에서만 허용한다. 이보다 높으면 hover_est가 오염됐더라도 자동착륙을
// 택한다(유효 호버 추정치가 있어야 한다). 지상에서도 FS_MAX_MS 동안 모터가
// 돌 수 있으며, 이 절대 하한은 고도나 실제 접지를 판별하는 센서가 아니다.
const int      FS_GROUND_CUT_MAX_US  = 1150;     // RC 끊김 시 스로틀이 이 이하면 지상으로 보고 즉시 모터 정지

// --- 센서 고장 감시 ---
const uint32_t PID_WDT_TIMEOUT_MS = 500;     // pid_task 정지 시 강제 재부팅 (1kHz 루프 대비 큰 여유)
const int32_t  FROZEN_DELTA_RAW   = 1;       // 6축 raw 변화량 합이 이 이하면 정지 의심 (LSB)
const uint32_t IMU_FROZEN_MS      = 300;     // 그 상태가 이만큼 지속되면 freeze 확정
const float    GYRO_DISAGREE_DPS  = 15.0f;   // 두 IMU 각속도 차이 임계 (deg/s)
const uint32_t IMU_DISAGREE_MS    = 150;     // 불일치 지속 시간

// --- 부팅 시 자이로 bias 캘리브레이션 (calibrate_bias) ---
const int   BIAS_CALIB_SAMPLES   = 2000;     // 1ms 간격 2000개 = 약 2초 동안 평균
const float BIAS_MOVEMENT_THRESH = 1.0f;     // 축별 stddev 상한 (deg/s)
const int   BIAS_CALIB_RETRIES   = 3;        // 움직임이 감지되면 최대 3번 재시도
const float ACCEL_CAL_MIN_G      = 0.8f;     // 이 범위 밖은 센서 이상: 보정 금지
const float ACCEL_CAL_MAX_G      = 1.2f;

// --- 루프 주기 ---
const int OUTER_DIV = 4;                     // outer loop = 1kHz / 4 = 250Hz

// --- 필터/적분 상한 ---
const float GYRO_SW_LPF_HZ = 80.0f;  // 하드웨어 LPF(121Hz)에 겹치는 소프트웨어 LPF
const float I_TERM_MAX_US  = 50.0f;  // 축별 적분 기여 상한 (모터 µs)

// ==========================================================
// 3. 유틸
// ==========================================================
// 1차 저역통과 필터(LPF): 빠르게 흔들리는 노이즈는 줄이고 느린 변화만 통과시킨다
//  RC 회로와 같은 식: rc = 1/(2π·fc), alpha = dt/(rc+dt). 매 호출마다 출력이 입력 쪽으로 alpha 비율만큼 다가간다
//  예) fc 80Hz, dt 1ms → alpha ≈ 0.33 / fc 40Hz → alpha ≈ 0.20 (차단 주파수가 낮을수록 더 부드럽고 더 늦다)
class LowPassFilter {
public:
  float alpha, last = 0.0f;   // alpha: 반영 비율, last: 직전 출력(필터 상태)
  LowPassFilter(float cutoff_hz, float dt) {
    float rc = 1.0f / (2.0f * PI * cutoff_hz);
    alpha = dt / (rc + dt);
  }
  float update(float in) { last += alpha * (in - last); return last; }   // 새 입력을 넣고 필터 출력을 돌려준다
  void reset(float value = 0.0f) { last = value; }                         // 상태 초기화 (잠금/재시동 시 이전 값이 튀지 않게)
};

// Arduino sketch preprocessor의 자동 함수 원형보다 먼저 보여야 하는 반환 타입.
using MotorMix = ControlAllocation;   // 예전 이름(MotorMix)을 새 allocator 결과 구조체의 별칭으로 유지

// 텔레메트리용 IMU별 값 (IMU1 축 기준 → body 축으로 변환된 값). gyro는 dps, accel은 g
struct ImuTelemetrySample {
  float imu1GyroX, imu1GyroY, imu1GyroZ;
  float imu1AccelX, imu1AccelY, imu1AccelZ;
  float imu2GyroX, imu2GyroY, imu2GyroZ;
  float imu2AccelX, imu2AccelY, imu2AccelZ;
};

// IMU별 freeze 감시 (raw 레지스터 값이 멈췄는지)
struct FreezeMon {
  int16_t  lastGyro[3] = {0, 0, 0};    // 직전 tick 자이로 레지스터 원값
  int16_t  lastAccel[3] = {0, 0, 0};   // 직전 tick 가속도 레지스터 원값
  uint32_t since = 0;                  // 값 변화가 없기 시작한 시각 (0 = 지금 변하는 중)
  bool     init  = false;              // 첫 값을 받았는가
};

// [링 버퍼 lock-free 동작] 생산자(Core 1)는 head만, 소비자(Core 0)는 tail만 증가시킨다
//  head, tail은 계속 커지는 카운터이고 실제 칸 위치는 & MASK로 구한다. (head - tail) = 들어 있는 개수
//  __atomic_*: 두 코어가 동시에 읽고 쓰는 변수를 안전하게 다루는 GCC 내장 함수
//   - RELEASE(쓰기): 이 저장 전에 한 메모리 쓰기(샘플 복사)가 다른 코어에 먼저 보이도록 보장
//   - ACQUIRE(읽기): 이 읽기 뒤의 메모리 읽기가 상대가 RELEASE 전에 쓴 내용을 보도록 보장
//   - RELAXED: 순서 보장 없이 값만 원자적으로 (자기 자신만 쓰는 변수나 통계 카운터)
//  이 짝 덕분에 mutex 없이도 "샘플을 다 쓴 뒤에 head가 올라간 것"을 소비자가 보게 된다

// 링 버퍼에 쌓여 있는 샘플 수
static inline uint32_t imuRawRingCount(const ImuRawRing &ring) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  return head - tail;
}

// [생산자, Core 1] 샘플 1개를 넣는다. 가득 차 있으면 버리고 dropped를 1 올린다 (pid_task를 기다리게 하지 않음)
static inline bool imuRawRingPush(
    ImuRawRing &ring, const ImuRawSample &sample, uint32_t sampleUs) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= IMU_RAW_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return false;
  }
  if (head == tail) {
    // 비어 있던 버퍼의 첫 샘플이면 절대 시각을 기록 (소비자가 dt_us를 누적해 시각을 복원하는 기준점)
    __atomic_store_n(&ring.first_t_us, sampleUs, __ATOMIC_RELAXED);
  }
  ring.samples[head & IMU_RAW_RING_MASK] = sample;           // 먼저 샘플을 복사하고
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);  // 그다음 head를 올려 공개한다
  return true;
}

// [소비자, Core 0] 샘플 1개를 꺼낸다. 비어 있으면 false
static inline bool imuRawRingPop(ImuRawRing &ring, ImuRawSample &sample) {
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  sample = ring.samples[tail & IMU_RAW_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

// 두 IMU의 레지스터 원값(보정 전)과 시간 간격, 자동착륙 단계를 27B 샘플 하나로 묶는다
static inline ImuRawSample makeImuRawSample(
    uint16_t dtUs, const inv_imu_sensor_event_t &e1,
    const inv_imu_sensor_event_t &e2, uint8_t failsafePhase) {
  ImuRawSample sample = {};
  sample.dt_us = dtUs;
  sample.failsafe_phase = failsafePhase;
  for (int axis = 0; axis < 3; axis++) {
    sample.imu1_gyro[axis] = e1.gyro[axis];
    sample.imu1_accel[axis] = e1.accel[axis];
    sample.imu2_gyro[axis] = e2.gyro[axis];
    sample.imu2_accel[axis] = e2.accel[axis];
  }
  return sample;
}

// 헤더(ZIMU)를 채우고 샘플 n개를 복사해 보낼 패킷을 만든다. 반환값 = 실제 보낼 바이트 수 (20 + 27×n)
static inline size_t buildImuRawDatagram(
    ImuRawDatagram &datagram, uint32_t batchSeq, uint32_t baseTUs,
    uint32_t dropped, const ImuRawSample *samples, uint8_t nSamples) {
  if (nSamples == 0 || nSamples > IMU_RAW_BATCH_MAX) return 0;
  memcpy(datagram.header.magic, "ZIMU", 4);
  datagram.header.version = IMU_RAW_VERSION;
  datagram.header.n_samples = nSamples;
  datagram.header.reserved = 0;
  datagram.header.batch_seq = batchSeq;
  datagram.header.base_t_us = baseTUs;
  datagram.header.dropped = dropped;
  memcpy(datagram.samples, samples, nSamples * sizeof(ImuRawSample));
  return sizeof(ImuRawHeader) + nSamples * sizeof(ImuRawSample);
}

// IMU1 센서 축 값을 body 축(텔레메트리 기준)으로 바꾼다. pid_task의 bodyGx/bodyGy/bodyGz 변환과 같은 부호
//  body x = 센서 y, body y = -센서 x, gyro z는 -센서 z (accel z는 +). accel은 여기서 ACCEL_SCALE을 곱해 g 단위로
static inline ImuTelemetrySample makeImuTelemetrySample(
    const float g1[3], const float a1[3],
    const float g2[3], const float a2[3]) {
  return {
    g1[1], -g1[0], -g1[2],
    a1[1] * ACCEL_SCALE, -a1[0] * ACCEL_SCALE, a1[2] * ACCEL_SCALE,
    g2[1], -g2[0], -g2[2],
    a2[1] * ACCEL_SCALE, -a2[0] * ACCEL_SCALE, a2[2] * ACCEL_SCALE
  };
}
#if WIFI_LATENCY_DEBUG
static inline void recordLatEvent(LatRing &ring, uint32_t tMs,
                                  uint32_t durUs, LatKind kind) {
  uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= LAT_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return;
  }
  ring.events[head & LAT_RING_MASK] = {tMs, durUs, (uint8_t)kind};
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);
}

static inline bool popLatEvent(LatRing &ring, LatEvt &event) {
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  event = ring.events[tail & LAT_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static const char *latKindName(uint8_t kind) {
  switch (kind) {
    case LAT_UDP_LOOP:  return "UDP_LOOP";
    case LAT_UDP_PARSE: return "UDP_PARSE";
    case LAT_UDP_SEND:  return "UDP_SEND";
    case LAT_PID:       return "PID";
    default:            return "UNKNOWN";
  }
}

static void drainLatRing(LatRing &ring) {
  LatEvt event;
  while (popLatEvent(ring, event)) {
    if (event.kind == LAT_PID) {
      if (event.dur_us > latPidMaxUs) latPidMaxUs = event.dur_us;
      if (event.dur_us > 5000U) latPidOver5++;
    } else {
      if (event.dur_us > latUdpMaxUs) latUdpMaxUs = event.dur_us;
      if (event.kind == LAT_UDP_LOOP) {
        if (event.dur_us > 30000U) latUdpLoopOver30++;
        if (event.dur_us > 100000U) latUdpLoopOver100++;
        if (event.dur_us > 300000U) latUdpLoopOver300++;
      }
    }
    Serial.printf("[LAT] t=%lu kind=%s dur_us=%lu\n",
                  (unsigned long)event.t_ms, latKindName(event.kind),
                  (unsigned long)event.dur_us);
  }
}
#endif
// [IMU freeze 감시] 센서 값이 IMU_FROZEN_MS(300ms) 동안 사실상 안 변하면 true
//  살아 있는 센서는 정지 상태에서도 노이즈로 값이 계속 조금씩 바뀐다 → 완전히 같은 값 반복 = SPI 통신 끊김/센서 정지
static bool checkFreeze(FreezeMon &m, const inv_imu_sensor_event_t &e, uint32_t nowMs) {
  if (!m.init) {
    for (int k = 0; k < 3; k++) {
      m.lastGyro[k] = e.gyro[k];
      m.lastAccel[k] = e.accel[k];
    }
    m.init = true;
    return false;
  }

  // abs(x)+abs(y)+abs(z)의 변화만 보면 축 변화가 서로 상쇄될 수 있다.
  // 각 레지스터의 변화량을 직접 더해 통신 정지/동일 프레임 반복을 감시한다.
  int32_t delta = 0;
  for (int k = 0; k < 3; k++) {
    delta += abs((int32_t)e.gyro[k] - m.lastGyro[k]);
    delta += abs((int32_t)e.accel[k] - m.lastAccel[k]);
  }

  bool frozen = false;
  if (delta <= FROZEN_DELTA_RAW) {
    if (m.since == 0) m.since = nowMs;
    else if (nowMs - m.since >= IMU_FROZEN_MS) frozen = true;
  } else {
    m.since = 0;
  }
  for (int k = 0; k < 3; k++) {
    m.lastGyro[k] = e.gyro[k];
    m.lastAccel[k] = e.accel[k];
  }
  return frozen;
}

// RC 워치독. nowMs는 루프 맨 위에서 뜬 값이라 udp_task(코어0)가 그 뒤에
// lastRcMs를 갱신하면 lastRcMs가 nowMs보다 미래일 수 있다. 부호 있는 차이로
// 비교해야 unsigned 언더플로로 즉시 타임아웃이 터지지 않는다. millis() wrap도
// 모듈러 뺄셈이라 그대로 안전하다.
static inline bool rcTimedOut(uint32_t nowMs, uint32_t lastMs) {
  return (int32_t)(nowMs - lastMs) > (int32_t)RC_TIMEOUT_MS;
}

// ==========================================================
// 4. 시스템 변수
// ==========================================================
// [코어 분담] Core 1 = pid_task(1kHz 제어), Core 0 = udp_task(명령 수신/텔레메트리/자기계/광류)
//  두 코어가 같이 쓰는 변수는 volatile 또는 portMUX(임계구역)로 보호한다
WiFiUDP   udp;                              // UDP 소켓 (udp_task만 사용)
char      packetBuffer[256];                // 받은 명령 문자열 버퍼
IPAddress laptopIP;                         // 마지막으로 명령을 보낸 노트북 주소 → 텔레메트리 목적지
int       laptopPort            = 0;
volatile bool connectionEstablished = false;   // 노트북에서 패킷을 한 번이라도 받았는가 (그 전엔 보낼 곳이 없음)

// startGyro/startAccel은 ODR·FSR만 설정하고 UI 필터 대역폭은 칩 전원
// 기본값(bypass) 그대로 둔다. 그러면 프롭 진동이 필터 없이 PID에 들어오므로
// protected icm_driver에 접근해 하드웨어 LPF를 켠다 (gyro 121Hz, accel 25Hz).
class ICM42670WithLPF : public ICM42670 {
public:
  ICM42670WithLPF(SPIClass &spi_bus, uint8_t cs) : ICM42670(spi_bus, cs) {}
  int setLowPassFilters() {
    int rc = 0;
    rc |= inv_imu_set_gyro_ln_bw(&icm_driver, GYRO_CONFIG1_GYRO_FILT_BW_121);
    rc |= inv_imu_set_accel_ln_bw(&icm_driver, ACCEL_CONFIG1_ACCEL_FILT_BW_25);
    return rc;
  }
};

ICM42670WithLPF IMU1(SPI, SPI_CS1);    // 같은 SPI 버스, CS 핀으로 구분
ICM42670WithLPF IMU2(SPI, SPI_CS2);
DFRobot_BMM350_I2C bmm(&Wire, 0x14);   // BMM350 자기계, I2C 주소 0x14

volatile bool  safety_lock  = true;   // true = 시동 꺼짐(모터 정지). 부팅 시 잠금 상태로 시작
// [safety 요청 구조] udp_task(Core 0)는 "start/stop 해 달라"는 요청 플래그만 세우고,
//  실제 safety_lock 변경은 pid_task(Core 1)가 매 tick applyPendingSafetyRequest()에서 한다
//  → 모터를 쓰는 쪽과 잠금을 바꾸는 쪽이 같은 코어라 한 tick 안에서 상태가 엇갈리지 않는다
// 런타임 safety_lock 전이는 Core 1(pid_task)만 수행한다. Core 0은 요청만
// 게시하며, 두 요청이 함께 대기하면 disarm이 arm보다 우선한다.
volatile bool  safety_disarm_requested = false;   // "stop" 요청
volatile bool  safety_arm_requested = false;      // "start" 요청
volatile bool  failsafe_resume_requested = false; // "resume" 요청 (자동착륙 중 조종 복귀)
volatile bool  hover_reset_requested = false;     // start 시 호버 추정치 초기화 요청
portMUX_TYPE safetyRequestMux = portMUX_INITIALIZER_UNLOCKED;   // 위 요청 플래그들을 보호하는 스핀락
// 런타임 쓰기는 pid_task만 담당한다. Core 0은 텔레메트리/명령 처리에서 읽기만 한다.
volatile uint8_t fs_phase = FS_NONE;   // 텔레메트리 Failsafe_Phase
volatile uint8_t fs_probe_state = FS_PROBE_WAIT;   // 착지 프로브 상태 (텔레메트리 41)
volatile uint8_t fs_probe_no_response = 0;         // "반응 없음" 연속 횟수 (텔레메트리 42)
volatile float fs_probe_response_g = 0.0f;         // 마지막 프로브 반응 크기 g (텔레메트리 43)
// 한 비행에서 최초 자동착륙에 들어간 시각. resume은 지우지 않으며 새 start가
// 실제로 Core 1에서 arm 전이될 때만 초기화한다.
volatile uint32_t fs_first_enter_ms = 0;
volatile bool fs_first_enter_valid = false;
// 3901-L0X 상태. udp_task 만 쓰고 sendTelemetry 가 읽는다(같은 코어).
MspSensorParser mspSensor;                 // UART 바이트 파서 상태
volatile int32_t msp_range_mm = 0;         // 라이다 거리 (mm)
volatile int     msp_range_quality = -1;   // -1 = 신선한 프레임 없음
volatile int32_t msp_flow_x = 0, msp_flow_y = 0;   // 광류 이동량
volatile int     msp_flow_quality = -1;
uint32_t msp_last_range_ms = 0;            // 마지막 라이다 프레임 수신 시각 (신선도 판단)
uint32_t msp_last_flow_ms = 0;             // 마지막 광류 프레임 수신 시각
uint16_t msp_reported_unknown = 0;         // 같은 ID 를 반복해 찍지 않기 위해

volatile float hover_est = 0.0f;       // 추정 호버 collective (us)
// hover_valid는 만료시키지 않는다. false는 RC 끊김 시 즉시 컷을 뜻하므로
// 공중에서 freshness 만료로 false가 되면 추락한다. 낡은 추정치가 덜 위험하다.
volatile bool hover_valid = false;
// hoverTracker와 그 공개 mirror는 pid_task만 쓴다. Core 0의 start는
// safetyRequestMux를 통해 reset을 요청하고 직접 구조체를 초기화하지 않는다.
HoverThrottleEstimator hoverTracker = {};
// 목표 자세 (°). X = roll, Y = pitch, Z = yaw(heading). rc/rcr 명령이 쓰고 pid_task 바깥 루프가 읽는다
volatile float targetAngleX = 0.0f, targetAngleY = 0.0f, targetAngleZ = 0.0f;
// 기체 트림(도). 추정기 0°와 진짜 수평의 차이를 보정한다. 비행별 상태가 아니라
// 기체 속성이므로 start/stop이 지우지 않는다.
volatile float trim_roll = 0.0f;
volatile float trim_pitch = 0.0f;
volatile float targetYawRate = 0.0f;   // rcr이 준 yaw 각속도 명령 (dps)

volatile float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f; // 추정 각도
volatile float gyroX  = 0.0f, gyroY  = 0.0f, gyroZ  = 0.0f; // 융합 각속도 (body frame)
volatile float accX   = 0.0f, accY   = 0.0f, accZ   = 0.0f; // 융합 가속도 (g)
ImuTelemetrySample imuSampleSnapshot = {};                 // IMU별 텔레메트리 값 (pid_task가 매 tick 갱신)
portMUX_TYPE imuSampleMux = portMUX_INITIALIZER_UNLOCKED;   // 12개 float를 한 번에 복사하도록 보호 (중간에 섞인 값 방지)
volatile int   motorOut[4] = {1000,1000,1000,1000};   // 마지막으로 내보낸 모터 출력 (텔레메트리용)
volatile float tgtRate[3]  = {0,0,0}; // roll,pitch,yaw dps
volatile int   pidLoopHz   = 0;       // 실측 pid_task 주기 (1초마다 계산, 정상이면 약 1000)

// Core 0 writer -> Core 1 reader. 짝수 seq만 완성된 벡터다.
volatile bool  mag_enabled = false;       // "mag 1": 자기계를 yaw 융합에 사용
volatile bool  mag_ready = false;         // BMM350 초기화 완료
volatile bool  mag_calibrating = false;   // "magcal 1": 캘리브레이션용 원시값 수집 중
volatile float magFieldX = 0.0f, magFieldY = 0.0f, magFieldZ = 0.0f;   // 보정된 최신 자기장 (Core 0 → Core 1)
volatile float magHeading = 0.0f;         // 마지막 기울기 보정 heading (텔레메트리)
volatile float magTelemX = 0.0f, magTelemY = 0.0f, magTelemZ = 0.0f;   // 텔레메트리로 보낼 자기장
volatile bool magTelemCalActive = false;  // 위 값이 캘리브레이션용 원시값인가
volatile uint32_t magSampleMs = 0;        // 최신 자기장 측정 시각
volatile bool magSampleValid = false;     // 한 번이라도 측정값이 들어왔는가
volatile bool mag_reference_pending = true;   // 다음 융합 때 기준 오프셋을 다시 잡아라

portMUX_TYPE magSnapshotMux = portMUX_INITIALIZER_UNLOCKED;   // 자기계 관련 공유 변수 보호
// 나침반 heading(자북 기준)을 angleZ 기준으로 옮기는 오프셋 = angleZ - heading (기준을 잡는 순간 값)
//  mag을 켜거나 start로 angleZ를 0으로 만들 때 다시 잡는다 → 그 순간 오차가 0이라 갑자기 북쪽으로 도는 일이 없다
float magYawReferenceOffset = 0.0f;

// 캘리브레이션 중 축별 최소/최대 (span 출력용. 실제 보정값은 PC의 magcal_fit.py가 계산)
float magCalMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
float magCalMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
uint32_t magCalSampleCount = 0;
uint32_t lastMagReadMs = 0;   // 마지막 BMM350 읽기 시각 (20ms 간격 유지)

float iTermRoll = 0.0f, iTermPitch = 0.0f, iTermYaw = 0.0f;  // 적분 기여 (모터 µs)

float gyro_bias1[3] = {0,0,0};   // 부팅 캘리브레이션으로 구한 자이로 영점 오차 (dps). 매 샘플에서 뺀다
float gyro_bias2[3] = {0,0,0};
// 두 센서의 gain 오차는 독립이고 단일-IMU 폴백도 있으므로 각각 보정한다.
float accel_scale1 = 1.0f;
float accel_scale2 = 1.0f;

volatile uint32_t lastRcMs        = 0;       // 마지막 rc/rcr 명령 수신 시각 (RC 워치독 기준)
volatile bool     fault_rc        = false;   // RC 끊김 발생
volatile bool     fault_imu1      = false;   // IMU1 freeze
volatile bool     fault_imu2      = false;   // IMU2 freeze
volatile bool     fault_disagree  = false;   // 두 IMU 불일치 (중재 불가)
volatile bool     fault_attitude  = false;   // 과도 기울기
volatile int      active_imus     = 2;       // 현재 사용 중인 IMU 수 (telemetry)
volatile bool     mixer_scaled    = false;   // 자세 mixer가 축소됐는지
volatile float    mixer_yaw_scale = 1.0f;    // 다음 tick yaw authority 입력
AllocationTelemetrySample allocationTelemetry = {
    false, 1.0f, 1.0f, 1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    YAW_AUTH_NORMAL};
portMUX_TYPE allocationTelemetryMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool     calibration_ok  = false;   // 부팅 자이로 캘리브레이션 성공. false면 start 거부

// 재시동 판단용 현재 센서 상태. fault_imu* / fault_disagree는 비행 중 latch된다.
volatile bool     imu1_frozen_now = false;
volatile bool     imu2_frozen_now = false;
volatile bool     imu_disagree_now = false;

// scripts/control_dualsense.py 프로토콜 호환용
volatile uint32_t lastRcSeq       = 0;       // 마지막으로 받아들인 rc 패킷 번호
volatile bool     rcSeqValid      = false;   // lastRcSeq가 유효한가 (start 후 첫 패킷 전엔 false)
volatile uint32_t rcTotalPkts     = 0;       // 받은 rc 패킷 수
volatile uint32_t rcDroppedPkts   = 0;       // 번호가 건너뛰거나 늦게 온 패킷 수 (UDP 손실 추정)

// ---------- Core 0 ↔ Core 1 공유 상태 접근 함수 ----------
// portENTER_CRITICAL/EXIT: 스핀락으로 임계구역을 만든다. 그 사이 코드는 다른 코어와 동시에 실행되지 않는다
//  안에서는 블로킹 작업(I2C, 시리얼 출력) 없이 변수 복사만 짧게 한다 → pid_task가 오래 기다리지 않게

// "stop": 시동 해제 요청. 대기 중인 다른 요청은 모두 취소 (disarm이 항상 우선)

static inline void requestSafetyDisarm() {
  portENTER_CRITICAL(&safetyRequestMux);
  safety_disarm_requested = true;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  hover_reset_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
}

// "start": 시동 요청. 이미 disarm 요청이 대기 중이면 무시 (stop이 이긴다). 호버 추정 초기화도 함께 요청
static inline void requestSafetyArm() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (!safety_disarm_requested) {
    hover_reset_requested = true;
    safety_arm_requested = true;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

// 게인/트림/자기 보정 같은 설정 변경 허용 여부. 자동착륙 하강 중에는 거부 (하강 도중 동작이 바뀌지 않게)
static inline bool configChangeAllowed(const char *setting) {
  if (fs_phase != FS_DESCENDING) return true;
  Serial.printf(">>> %s refused (auto-land descending)\n", setting);
  return false;
}

// [Core 1, 매 tick] 대기 중인 요청을 실제 safety_lock에 반영한다. disarm 요청이 있으면 arm보다 먼저 처리
static inline void applyPendingSafetyRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (safety_disarm_requested) {
    safety_lock = true;
    safety_disarm_requested = false;
    safety_arm_requested = false;
    hover_reset_requested = false;
  } else if (safety_arm_requested) {
    if (hover_reset_requested) {
      // Core 1이 reset을 끝낸 뒤에만 safety_lock=false를 공개한다.
      hover_valid = false;
      hover_est = 0.0f;
      hoverTracker = {};
    }
    hover_reset_requested = false;
    safety_lock = false;
    safety_arm_requested = false;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

// "resume" 요청을 걸어 둔다. 실제 허용 여부는 pid_task가 resumeRefusalReason()으로 다시 확인한다
static inline void requestFailsafeResume() {
  portENTER_CRITICAL(&safetyRequestMux);
  failsafe_resume_requested = true;
  portEXIT_CRITICAL(&safetyRequestMux);
}

// 요청이 있었는지 읽으면서 동시에 지운다 (한 요청이 한 번만 처리되게: "take")
static inline bool takeFailsafeResumeRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  const bool requested = failsafe_resume_requested;
  failsafe_resume_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
  return requested;
}

// resume을 허용할지 검사. 위에서부터 순서대로 확인해 처음 걸린 거부 사유를 돌려준다 (전부 통과하면 ALLOWED)
static inline ResumeRefusalReason resumeRefusalReason(uint32_t nowMs) {
  if (fs_phase != FS_DESCENDING) return RESUME_REFUSED_PHASE;
  if (fs_first_enter_valid &&
      (uint32_t)(nowMs - fs_first_enter_ms) >= FS_RESUME_MAX_MS) {
    return RESUME_REFUSED_CUMULATIVE;
  }
  if (rcTimedOut(nowMs, lastRcMs)) return RESUME_REFUSED_RC;
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    return RESUME_REFUSED_TILT;
  }
  if (active_imus <= 0
      || (imu1_frozen_now && imu2_frozen_now)
      || imu_disagree_now) {
    return RESUME_REFUSED_IMU;
  }
  if (!hover_valid) return RESUME_REFUSED_HOVER;
  return RESUME_ALLOWED;
}

static inline void logResumeRefusal(ResumeRefusalReason reason) {
  Serial.printf(">>> RESUME REFUSED %s\n", resumeRefusalName(reason));
}

// [Core 0 → Core 1] 보정된 자기장 최신값을 게시 / 읽기 (publish = 쓰는 쪽, read = 읽는 쪽)
static inline void publishMagSample(
    float x, float y, float z, uint32_t sample_ms) {
  // I2C read happens before this lock. The critical section only publishes
  // four words, so pid_task never waits on the blocking transaction.
  portENTER_CRITICAL(&magSnapshotMux);
  magFieldX = x;
  magFieldY = y;
  magFieldZ = z;
  magSampleMs = sample_ms;
  magSampleValid = true;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline bool readMagSnapshot(MagSnapshot &snapshot) {
  portENTER_CRITICAL(&magSnapshotMux);
  bool valid = magSampleValid;
  snapshot.x = magFieldX;
  snapshot.y = magFieldY;
  snapshot.z = magFieldZ;
  snapshot.sample_ms = magSampleMs;
  portEXIT_CRITICAL(&magSnapshotMux);
  return valid;
}

// 텔레메트리로 보낼 자기장 값을 게시. calibration_raw=true면 캘리브레이션용 원시값, false면 간섭 보정까지 끝난 값
static inline void publishMagTelemetry(
    float x, float y, float z, bool calibration_raw) {
  portENTER_CRITICAL(&magSnapshotMux);
  // Core 1 may already have entered its normal-domain block when Core 0
  // starts calibration. Do not let that late writer overwrite a raw snapshot.
  if (calibration_raw || !mag_calibrating) {
    magTelemX = x;
    magTelemY = y;
    magTelemZ = z;
    magTelemCalActive = calibration_raw;
  }
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline void readMagTelemetry(
    float &x, float &y, float &z, bool &calibration_raw) {
  portENTER_CRITICAL(&magSnapshotMux);
  x = magTelemX;
  y = magTelemY;
  z = magTelemZ;
  calibration_raw = magTelemCalActive;
  portEXIT_CRITICAL(&magSnapshotMux);
}

// 다음 자기계 융합 때 magYawReferenceOffset을 다시 잡으라고 요청 / 그 요청을 꺼내며 지우기
static inline void requestMagReferenceUpdate() {
  portENTER_CRITICAL(&magSnapshotMux);
  mag_reference_pending = true;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline bool takeMagReferenceRequest() {
  portENTER_CRITICAL(&magSnapshotMux);
  const bool requested = mag_reference_pending;
  mag_reference_pending = false;
  portEXIT_CRITICAL(&magSnapshotMux);
  return requested;
}

// [Core 1 → Core 0] IMU별 텔레메트리 스냅샷 쓰기 / 읽기
static inline void publishImuSample(const ImuTelemetrySample &sample) {
  portENTER_CRITICAL(&imuSampleMux);
  imuSampleSnapshot = sample;
  portEXIT_CRITICAL(&imuSampleMux);
}

static inline ImuTelemetrySample readImuSampleSnapshot() {
  portENTER_CRITICAL(&imuSampleMux);
  const ImuTelemetrySample sample = imuSampleSnapshot;
  portEXIT_CRITICAL(&imuSampleMux);
  return sample;
}

// [Core 1, 매 tick] 레지스터 원값을 ZIMU 링 버퍼에 넣는다. 절대 시각 대신 직전 샘플과의 간격(dt_us, 2B)만 저장해 크기를 줄인다
static inline void publishImuRawRegisters(
    const inv_imu_sensor_event_t &e1, const inv_imu_sensor_event_t &e2,
    uint32_t sampleUs, uint8_t failsafePhase) {
  uint16_t dtUs = 0;   // 첫 샘플은 0
  if (rawProducerTimeValid) {
    const uint32_t elapsedUs = sampleUs - rawProducerLastUs;
    // uint16 최대(65535µs ≈ 65ms)를 넘으면 0xFFFF로 포화 → 디코더가 "간격이 비정상적으로 컸다"로 알 수 있다
    dtUs = elapsedUs > 65535UL
        ? 0xFFFFU
        : static_cast<uint16_t>(elapsedUs);
  }
  const ImuRawSample sample = makeImuRawSample(dtUs, e1, e2, failsafePhase);
  if (imuRawRingPush(imuRawRing, sample, sampleUs)) {
    rawProducerLastUs = sampleUs;
    rawProducerTimeValid = true;
  }
}

// BMM350 자기계 초기화 (처음 한 번만). delay(100)와 I2C 통신으로 블로킹되므로 Core 0에서, 가능하면 시동 해제 상태에서 호출
static bool initMagnetometer() {
  if (mag_ready) return true;

  pinMode(LDO_SHD, OUTPUT);
  digitalWrite(LDO_SHD, HIGH);   // 자기계 전원 켜기
  delay(100);                    // 전원 안정화 대기
  Wire.begin(BMM_SDA, BMM_SCL);
  uint8_t status = bmm.begin();
  if (status != 0) {
    Serial.printf("[MAG] BMM350 init failed (%u)\n", (unsigned)status);
    return false;
  }
  bmm.setOperationMode(eBmm350NormalMode);
  bmm.setRate(BMM350_DATA_RATE_50HZ);
  mag_ready = true;
  lastMagReadMs = millis() - MAG_SAMPLE_PERIOD_MS;   // 다음 udp_task 반복에서 바로 첫 샘플을 읽도록
  Serial.println("[MAG] BMM350 ready @50Hz");
  return true;
}

// "magcal 1": 캘리브레이션 시작. 융합을 끄고 원시값을 텔레메트리로 내보낸다
//  기체를 여러 방향으로 돌린 뒤 "magcal 0" → PC에서 CSV를 magcal_fit.py로 타원 피팅해 MAG_HARD_IRON/SOFT_IRON을 구한다
static void startMagCalibration() {
  portENTER_CRITICAL(&magSnapshotMux);
  const bool alreadyActive = mag_calibrating;
  portEXIT_CRITICAL(&magSnapshotMux);
  if (alreadyActive) {
    Serial.println("[MAGCAL] already active");
    return;
  }
  mag_enabled = false;
  requestMagReferenceUpdate();
  portENTER_CRITICAL(&magSnapshotMux);
  mag_calibrating = true;
  portEXIT_CRITICAL(&magSnapshotMux);
  magCalMin[0] = FLT_MAX;
  magCalMin[1] = FLT_MAX;
  magCalMin[2] = FLT_MAX;
  magCalMax[0] = -FLT_MAX;
  magCalMax[1] = -FLT_MAX;
  magCalMax[2] = -FLT_MAX;
  magCalSampleCount = 0;
  Serial.println("[MAGCAL] logging raw XYZ; rotate drone, then send 'magcal 0'");
}

// "magcal 0": 캘리브레이션 종료. 텔레메트리를 보정값으로 되돌리고 축별 span(최대-최소)을 출력한다
static void stopMagCalibration() {
  MagSnapshot calibrated = {};
  const bool have_sample =
      magCalSampleCount > 0U && readMagSnapshot(calibrated);
  if (have_sample) {
    const float magThrDelta =
        (float)(base_throttle - MAG_THROTTLE_REF_US);
    calibrated.x -= mag_comp_x * magThrDelta;
    calibrated.y -= mag_comp_y * magThrDelta;
    calibrated.z -= mag_comp_z * magThrDelta;
  }

  portENTER_CRITICAL(&magSnapshotMux);
  mag_calibrating = false;
  if (have_sample) {
    magTelemX = calibrated.x;
    magTelemY = calibrated.y;
    magTelemZ = calibrated.z;
  }
  magTelemCalActive = false;
  portEXIT_CRITICAL(&magSnapshotMux);
  if (magCalSampleCount == 0U) {
    Serial.println("[MAGCAL] no samples");
    return;
  }
  Serial.printf("[MAGCAL] samples=%lu\n", (unsigned long)magCalSampleCount);
  Serial.printf("[MAGCAL] span X=%.6f uT Y=%.6f uT Z=%.6f uT\n",
                magCalMax[0] - magCalMin[0],
                magCalMax[1] - magCalMin[1],
                magCalMax[2] - magCalMin[2]);
  Serial.println(
      "[MAGCAL] save the telemetry CSV and run scripts/magcal_fit.py <flight_log.csv>");
}

// 자기계 보정: out = body_sign × (SOFT_IRON 행렬 × (raw - HARD_IRON))
//  1) hard-iron을 빼 타원의 중심을 원점으로 옮기고  2) soft-iron 행렬로 타원을 구로 편 뒤  3) 축 부호를 body에 맞춘다
static inline void applyMagCalibration(
    const float raw[3], const float hard_iron[3],
    const float soft_iron[3][3], float out[3]) {
  const float centered[3] = {
    raw[0] - hard_iron[0],
    raw[1] - hard_iron[1],
    raw[2] - hard_iron[2],
  };
  const float body_sign[3] = {
    MAG_BODY_SIGN_X, MAG_BODY_SIGN_Y, MAG_BODY_SIGN_Z,
  };
  for (int axis = 0; axis < 3; axis++) {
    float transformed = 0.0f;
    for (int source = 0; source < 3; source++) {
      transformed += soft_iron[axis][source] * centered[source];
    }
    out[axis] = body_sign[axis] * transformed;
  }
}

// [Core 0] 20ms(50Hz)마다 BMM350을 읽어 보정 후 Core 1에 게시. 캘리브레이션 중이면 원시값도 함께 내보낸다
static void sampleMagnetometer(uint32_t now_ms) {
  if (!mag_ready || now_ms - lastMagReadMs < MAG_SAMPLE_PERIOD_MS) return;
  lastMagReadMs = now_ms;

  // Blocking I2C read: caller is udp_task on core 0, never pid_task.
  sBmm350MagData_t data = bmm.getGeomagneticData();
  const float raw[3] = {data.float_x, data.float_y, data.float_z};
  if (!isfinite(raw[0]) || !isfinite(raw[1]) || !isfinite(raw[2])) return;

  if (mag_calibrating) {
    publishMagTelemetry(raw[0], raw[1], raw[2], true);
    for (int axis = 0; axis < 3; axis++) {
      if (raw[axis] < magCalMin[axis]) magCalMin[axis] = raw[axis];
      if (raw[axis] > magCalMax[axis]) magCalMax[axis] = raw[axis];
    }
    magCalSampleCount++;
  }

  float calibrated[3];
  applyMagCalibration(raw, MAG_HARD_IRON, MAG_SOFT_IRON, calibrated);
  publishMagSample(calibrated[0], calibrated[1], calibrated[2], now_ms);
}

// ==========================================================
// 5. 모터
// ==========================================================
// 펄스 폭(µs)을 LEDC 듀티 값으로 바꿔 출력. 예) 1500µs → 1500 × 16383 / 2500 ≈ 9830
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);   // ESC가 이해하는 범위 밖은 자른다
  uint32_t duty = ((uint32_t)us * ESC_MAXDUTY) / ESC_PERIOD;
  ledcWrite(pin, duty);
}
// 모터 4개 모두 1000µs(정지 신호)
void stopMotors() {
  writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// Pure allocator keeps this compatibility entrypoint for sketch callers and native
// integration tests. It assigns roll/pitch first, then fits yaw in the residual span.
static MotorMix mixAndDesaturate(float roll, float pitch, float yaw,
                                 int throttle, int minMotor, int maxMotor) {
  return allocateControl(roll, pitch, yaw, throttle, minMotor, maxMotor);
}

// ==========================================================
// 6. Gyro bias 캘리브레이션
// ==========================================================
// sign[]은 각 IMU를 drone(IMU1) frame으로 맞추는 부호.
// 정지 상태에서 1ms 간격으로 2000개를 읽어
//  bias_out       : 축별 자이로 평균 (정지 중이면 진짜 각속도는 0이므로 평균 = 영점 오차)
//  sd_out         : 축별 표준편차 (크면 캘리브레이션 중 기체가 움직였다는 뜻)
//  accel_mean_out : |accel| 평균 (정지 중이면 1g여야 함 → 가속도 배율 보정에 사용)
// 분산 = E[x²] - (E[x])² 으로 한 번 훑으면서 평균과 표준편차를 같이 구한다
static bool measure_imu_bias(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float sd_out[3],
                             float &accel_mean_out) {
  double sum[3] = {0,0,0}, sum_sq[3] = {0,0,0};
  double accel_mag_sum = 0.0;
  inv_imu_sensor_event_t e = {};
  int samples = 0;
  int attempts = 0;
  const int maxAttempts = BIAS_CALIB_SAMPLES + 100;
  while (samples < BIAS_CALIB_SAMPLES && attempts < maxAttempts) {
    attempts++;
    if (imu.getDataFromRegisters(e) != 0) {
      delayMicroseconds(1000);
      continue;
    }
    float g[3] = {
      sign[0] * e.gyro[0] * GYRO_SCALE,
      sign[1] * e.gyro[1] * GYRO_SCALE,
      sign[2] * e.gyro[2] * GYRO_SCALE
    };
    for (int k = 0; k < 3; k++) { sum[k] += g[k]; sum_sq[k] += (double)g[k]*g[k]; }
    const double ax = (double)e.accel[0] * ACCEL_SCALE;
    const double ay = (double)e.accel[1] * ACCEL_SCALE;
    const double az = (double)e.accel[2] * ACCEL_SCALE;
    accel_mag_sum += sqrt(ax*ax + ay*ay + az*az);
    samples++;
    delayMicroseconds(1000);
  }
  if (samples != BIAS_CALIB_SAMPLES) return false;
  accel_mean_out = (float)(accel_mag_sum / samples);
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / samples;
    double var  = sum_sq[k] / samples - mean*mean;
    if (var < 0) var = 0;
    bias_out[k] = (float)mean;
    sd_out[k]   = (float)sqrt(var);
  }
  return true;
}

// 정지 시 |accel| 평균이 1g가 되도록 하는 배율 = 1 / 평균. 0.8~1.2g 밖이면 센서 이상으로 보고 보정하지 않음(1.0)
static float guarded_accel_scale(float accel_mean_g, const char *imu_name) {
  if (accel_mean_g < ACCEL_CAL_MIN_G || accel_mean_g > ACCEL_CAL_MAX_G) {
    Serial.printf(
        "[CALIB] WARN %s accel |a| %.4fg outside [0.8,1.2], scale=1.0\n",
        imu_name, accel_mean_g);
    return 1.0f;
  }
  return 1.0f / accel_mean_g;
}

// 부팅 시 두 IMU의 자이로 bias와 가속도 배율을 측정. 움직임(표준편차 > 1dps)이 감지되면 최대 3번 다시 시도
// 실패하면 calibration_ok = false → start 명령이 거부된다
static bool calibrate_bias() {
  float sd1[3], sd2[3];
  accel_scale1 = 1.0f;
  accel_scale2 = 1.0f;
  for (int a = 1; a <= BIAS_CALIB_RETRIES; a++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", a, BIAS_CALIB_RETRIES);
    float accel_mean1 = 0.0f, accel_mean2 = 0.0f;
    bool read1 = measure_imu_bias(
        IMU1, IMU1_SIGN, gyro_bias1, sd1, accel_mean1);
    bool read2 = measure_imu_bias(
        IMU2, IMU2_SIGN, gyro_bias2, sd2, accel_mean2);
    if (!read1 || !read2) {
      Serial.printf("[CALIB] sensor read failed (imu1=%d imu2=%d)\n", (int)read1, (int)read2);
      continue;
    }
    float max_sd = max(max(max(sd1[0],sd1[1]),sd1[2]),
                       max(max(sd2[0],sd2[1]),sd2[2]));
    Serial.printf("[CALIB] IMU1 %.3f %.3f %.3f | IMU2 %.3f %.3f %.3f (maxSD %.3f)\n",
                  gyro_bias1[0],gyro_bias1[1],gyro_bias1[2],
                  gyro_bias2[0],gyro_bias2[1],gyro_bias2[2], max_sd);
    if (max_sd <= BIAS_MOVEMENT_THRESH) {
      accel_scale1 = guarded_accel_scale(accel_mean1, "IMU1");
      accel_scale2 = guarded_accel_scale(accel_mean2, "IMU2");
      Serial.printf(
          "[CALIB] accel mean IMU1 %.4fg scale %.6f | "
          "IMU2 %.4fg scale %.6f\n",
          accel_mean1, accel_scale1, accel_mean2, accel_scale2);
      Serial.println("[CALIB] OK");
      return true;
    }
    Serial.println("[CALIB] movement detected, retry");
  }
  Serial.println("[CALIB] FAIL: reboot and calibrate on a stationary surface");
  return false;
}

// ==========================================================
// 7. 자세 추정용 적응 alpha (가속도 신뢰도 기반)
// ==========================================================
// [상보필터 alpha] angle = alpha × (자이로 적분 각도) + (1 - alpha) × (가속도로 계산한 각도)
//  가속도는 중력 방향을 알려 주지만 기체가 가감속하면 틀린다 → |accel|이 1g에서 멀수록 가속도를 덜 믿는다
//  |accel| - 1g 편차 < 0.10g : ALPHA_STATIC / < 0.30g : ALPHA_NORMAL / 그 이상 : ALPHA_DYN (가속도 거의 무시)
const float ACC_DEV_SOFT = 0.10f, ACC_DEV_HARD = 0.30f;
// 1kHz 루프 기준 시정수 tau ≈ dt/(1-alpha): 0.999→1s, 0.9995→2s, 0.9998→5s.
// 이전 값(0.99, tau 0.1s)은 250Hz legacy 루프에서 온 것으로, 1kHz에서는
// 비행 중 가속도(추력 방향+진동)에 자세 추정이 끌려가 호버가 불안정해진다.
const float ALPHA_STATIC = 0.999f, ALPHA_NORMAL = 0.9995f, ALPHA_DYN = 0.9998f;
static inline float compute_alpha(float ax, float ay, float az) {
  float dev = fabsf(sqrtf(ax*ax+ay*ay+az*az) - 1.0f);
  if (dev < ACC_DEV_SOFT) return ALPHA_STATIC;
  if (dev < ACC_DEV_HARD) return ALPHA_NORMAL;
  return ALPHA_DYN;
}

// 잠금 진입 시 1회만 실행되는 정리. 세 잠금 경로가 모두 이 함수를 통해서만
// wasLocked를 세운다. 매 tick 지우면 시동 해제 상태에서 `yaw 1`을 켤 수 없다.
static inline void enterLockedState(bool &wasLocked) {
  if (!wasLocked) yaw_hold_override = false;
  wasLocked = true;
}

// allocator 텔레메트리를 "포화 없음" 기본값으로 되돌린다 (잠금/고장 시)
static inline void resetAllocationTelemetry() {
  portENTER_CRITICAL(&allocationTelemetryMux);
  mixer_scaled = false;
  mixer_yaw_scale = 1.0f;
  allocationTelemetry = {
      false, 1.0f, 1.0f, 1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      YAW_AUTH_NORMAL};
  portEXIT_CRITICAL(&allocationTelemetryMux);
}

// [Core 1 → Core 0] 이번 tick allocator 결과와 PID 값을 텔레메트리 스냅샷으로 게시
static inline void publishAllocationTelemetry(
    const MotorMix &mix, float pidRoll, float pidPitch, float pidYaw,
    float iRoll, float iPitch, float iYaw, int yawAuthorityState) {
  const AllocationTelemetrySample sample = {
      mix.scaled, mix.rp_scale, mix.yaw_scale, mix.collective_us,
      pidRoll, pidPitch, pidYaw, iRoll, iPitch, iYaw, yawAuthorityState};
  portENTER_CRITICAL(&allocationTelemetryMux);
  mixer_scaled = sample.scaled;
  allocationTelemetry = sample;
  portEXIT_CRITICAL(&allocationTelemetryMux);
}

static inline AllocationTelemetrySample readAllocationTelemetrySnapshot() {
  portENTER_CRITICAL(&allocationTelemetryMux);
  const AllocationTelemetrySample sample = allocationTelemetry;
  portEXIT_CRITICAL(&allocationTelemetryMux);
  return sample;
}

// ==========================================================
// 8. PID 태스크 (Core 1, 1kHz)
// ==========================================================
// [pid_task 한 tick(1ms) 흐름 정리]
//  1) 대기 중인 start/stop 요청 반영 → 2) 두 IMU 읽기, freeze 검사, 원시값 게시
//  3) bias/부호/LPF 보정 → 두 IMU 융합(평균 또는 한쪽만) → body 축 변환
//  4) 자세 추정(상보필터) + 250Hz로 자기계 yaw 보정
//  5) 안전 검사(과도 기울기, resume, RC 끊김 → 자동착륙) → 잠금이면 모터 끄고 다음 tick
//  6) 바깥 루프(250Hz): 각도 오차 → 목표 각속도 / yaw 잠금·권한 감시
//  7) 안쪽 루프(1kHz): 각속도 PID → allocator로 모터 분배 → 조건부 적분 → PWM 출력
void pid_task(void *pv) { // 26.10.30 기준 여기까지 읽음 -> 26.10.30보다 이후의 날짜가 주석으로 달려있지 않고 사용자가 어디까지 읽었는지 요청할 경우 이 부분을 기준으로 답변
  // 모터 정지 수단(stopMotors, RC timeout 검사)이 모두 이 태스크 안에 있으므로,
  // SPI 행업 등으로 태스크가 블로킹되면 모터가 마지막 PWM으로 고정된다.
  // 태스크 워치독이 panic 재부팅을 강제하고, 재부팅 후 ESC는 PWM 신호
  // 소실로 정지하며 setup()의 stopMotors()가 정지 신호를 복원한다.
  esp_task_wdt_add(NULL);

  // TickType_t: FreeRTOS가 시간을 tick 수로 저장하는 정수 타입. ESP32에서는 uint32_t, Arduino 기본 설정에서 1 tick = 1ms
  // xTaskGetTickCount(): 스케줄러 시작 후 지난 tick 수를 돌려준다. millis()와 비슷하며 정밀도는 1 tick (ISR 안에서는 FromISR 버전 사용)
  const TickType_t period = pdMS_TO_TICKS(1);   // vTaskDelayUntil 기반 (busy-wait 제거). pdMS_TO_TICKS: ms → tick 변환
  TickType_t wake = xTaskGetTickCount();        // 다음 기상 시각을 계산할 기준. vTaskDelayUntil이 매번 갱신한다
  const float dt = 0.001f;

  // 아래 변수들은 이 태스크 안에서만 쓰는 지역 상태 (while 루프 동안 계속 유지된다)
  LowPassFilter lpfD_Roll(40, dt), lpfD_Pitch(40, dt), lpfD_Yaw(40, dt);   // D항(미분) 노이즈 필터 40Hz
  LowPassFilter lpfG1[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};   // IMU1 자이로 x,y,z 80Hz
  LowPassFilter lpfG2[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};   // IMU2 자이로 x,y,z 80Hz
  float prevGyroX = 0.0f, prevGyroY = 0.0f, prevGyroZ = 0.0f;   // 직전 tick 각속도 (D항 미분용)

  FreezeMon fm1, fm2;           // IMU별 freeze 감시 상태
  uint32_t disagreeSince = 0;   // 두 IMU 불일치가 시작된 시각 (0 = 불일치 아님)

  float targetRateRoll = 0, targetRatePitch = 0, targetRateYaw = 0;   // 바깥 루프가 정한 목표 각속도 (dps)
  uint8_t outerCnt = 0;         // 0,1,2,3 순환. 0일 때만 바깥 루프 계산 → 1kHz/4 = 250Hz
  uint8_t magFusionCnt = 0;     // 자기계 융합용 별도 카운터 (잠금 중에도 250Hz 유지)
  uint32_t lastMicros = micros();   // 실제 dt 측정용
  uint32_t loopCount = 0;           // pidLoopHz 계산용
  uint32_t loopMarkerMs = millis();
  bool wasLocked = true;        // 직전 tick이 잠금 상태였나 → 잠금 해제 첫 tick 감지
  LandDetector landDet = {};    // 자동착륙 착지 감지기 상태
  LandProbeConfig landProbeConfig = FS_LAND_PROBE_CONFIG;
  int fs_probe_dip_us = FS_PROBE_DIP_MIN_US;   // 이번 하강에서 쓸 프로브 딥 크기
  uint32_t fs_enter_ms = 0;     // 이번 자동착륙 진입 시각
  float fs_hold_yaw = 0.0f;     // 자동착륙 동안 유지할 heading
  bool fs_probe_blocked_logged = false;   // BLOCKED 경고를 하강당 한 번만 찍기 위한 플래그
  YawAuthorityTracker yawAuthority = {};  // yaw 권한 상태 머신

  inv_imu_sensor_event_t e1 = {}, e2 = {};   // IMU 레지스터 읽기 결과 (원값)

  while (true) {
    vTaskDelayUntil(&wake, period);   // 직전 기상 시각 + 1ms까지 잠든다 → 실행 시간과 무관하게 일정한 1kHz
    esp_task_wdt_reset();             // 워치독에 "살아 있음" 보고. 500ms 동안 못 하면 재부팅
    TickType_t afterWake = xTaskGetTickCount();   // 실제로 깨어난 시각. wake(예정 시각)와의 차이가 지연 시간
    // 부호 없는 정수 뺄셈이라 tick 카운터가 약 49.7일 만에 0으로 돌아가도 경과 tick이 맞게 나온다
    if ((TickType_t)(afterWake - wake) > period) {
      // 큰 지연 뒤 밀린 tick을 연속 실행하지 않는다(센서 stale read/D항 spike 방지).
      wake = afterWake;
    }
    uint32_t nowMs = millis();
    bool checkProbeDelivery = false;   // 이번 tick 프로브 딥이 실제로 전달됐는지 확인해야 하나
    int probeReferenceThrottle = 0;    // 딥을 빼기 전 하강 스로틀
    uint32_t probeElapsedMs = 0;
    applyPendingSafetyRequest();       // 1) Core 0이 걸어 둔 start/stop 요청 반영
    // 1초마다 실제 루프 주파수 계산 (텔레메트리 pidLoopHz)
    loopCount++;
    uint32_t loopElapsedMs = nowMs - loopMarkerMs;
    if (loopElapsedMs >= 1000) {
      pidLoopHz = (int)((loopCount * 1000UL) / loopElapsedMs);
      loopCount = 0;
      loopMarkerMs = nowMs;
    }
    uint32_t nowUs = micros();
    float realDt = (nowUs - lastMicros) / 1e6f;   // 실제 경과 시간 (초). 적분/미분에 dt 상수 대신 사용
#if WIFI_LATENCY_DEBUG
    uint32_t rawDtUs = nowUs - lastMicros;
    if (rawDtUs > WIFI_LATENCY_PID_THRESHOLD_US) {
      recordLatEvent(pidLatRing, nowMs, rawDtUs, LAT_PID);
    }
#endif
    lastMicros = nowUs;
    // 지연 후 vTaskDelayUntil이 catch-up할 때 지나치게 작은 dt로 D항이 튀는 것을 방지.
    if (realDt < 0.0002f || realDt > 0.01f) realDt = dt;

    // ---------- 센서 읽기 ----------
    int readStatus1 = IMU1.getDataFromRegisters(e1);
    int readStatus2 = IMU2.getDataFromRegisters(e2);

    // 읽기 오류 시 event가 갱신되지 않으므로 동일 프레임으로 취급되어 freeze로 귀결된다.
    // fault는 비행 중 자동 복구하지 않고 다음 수동 start 때만 재평가/해제한다.
    bool frozen1 = checkFreeze(fm1, e1, nowMs);
    bool frozen2 = checkFreeze(fm2, e2, nowMs);
    imu1_frozen_now = frozen1;
    imu2_frozen_now = frozen2;
    if (frozen1 && !fault_imu1) Serial.printf("[FAULT] IMU1 FROZEN (read=%d)\n", readStatus1);
    if (frozen2 && !fault_imu2) Serial.printf("[FAULT] IMU2 FROZEN (read=%d)\n", readStatus2);
    if (frozen1) fault_imu1 = true;
    if (frozen2) fault_imu2 = true;

    // OFF hot path는 이 bool 검사 하나뿐이다. ON일 때만 micros()와 27B 복사를 한다.
    // freeze 판정 직후, 부호/scale/bias/LPF/body 변환 전의 레지스터 원값을 게시한다.
    if (__atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE)
        && connectionEstablished) {
      // fs_phase 는 이 함수보다 뒤에서 갱신되므로 직전 tick 값이다(1ms).
      publishImuRawRegisters(e1, e2, micros(), (uint8_t)fs_phase);
    }

    // IMU1 sensor frame 기준 각속도 (IMU2 축 정렬 + 개별 bias 보정)
    // 실제값(dps) = 부호 × 원값 × GYRO_SCALE - bias
    float g1[3] = {
      e1.gyro[0]*GYRO_SCALE - gyro_bias1[0],
      e1.gyro[1]*GYRO_SCALE - gyro_bias1[1],
      e1.gyro[2]*GYRO_SCALE - gyro_bias1[2] };
    float g2[3] = {
      IMU2_SIGN_X*e2.gyro[0]*GYRO_SCALE - gyro_bias2[0],
      IMU2_SIGN_Y*e2.gyro[1]*GYRO_SCALE - gyro_bias2[1],
      IMU2_SIGN_Z*e2.gyro[2]*GYRO_SCALE - gyro_bias2[2] };
    // 소프트웨어 LPF. 제어와 IMU 불일치 판정 모두 필터된 각속도를 쓴다.
    // (진동으로 순간 차이가 15dps를 넘어 비행 중 disagree 컷이 나는 것을 방지)
    for (int k = 0; k < 3; k++) {
      g1[k] = lpfG1[k].update(g1[k]);
      g2[k] = lpfG2[k].update(g2[k]);
    }
    // 가속도는 아직 LSB 단위 (× 부팅 배율 보정만). g 변환(ACCEL_SCALE)은 body 변환할 때 한다
    float a1[3] = {
      (float)e1.accel[0] * accel_scale1,
      (float)e1.accel[1] * accel_scale1,
      (float)e1.accel[2] * accel_scale1 };
    float a2[3] = {
      IMU2_SIGN_X*e2.accel[0] * accel_scale2,
      IMU2_SIGN_Y*e2.accel[1] * accel_scale2,
      IMU2_SIGN_Z*e2.accel[2] * accel_scale2 };

    // LPF 후 gyro와 LPF 없는 accel을 같은 1kHz 프레임에서 body 좌표로 게시한다.
    // 변환 계산은 락 밖에서 끝내고 임계구역에서는 float 12개만 복사한다.
    const ImuTelemetrySample currentImuSample =
        makeImuTelemetrySample(g1, a1, g2, a2);
    publishImuSample(currentImuSample);

    // 현재 샘플 불일치는 재시동 거부 판단에도 사용한다.
    bool disagreeSample = false;
    if (!frozen1 && !frozen2) {
      for (int k = 0; k < 3; k++) {
        if (fabsf(g1[k] - g2[k]) > GYRO_DISAGREE_DPS) disagreeSample = true;
      }
    }
    imu_disagree_now = disagreeSample;

    // latch된 센서는 비행 중 값이 다시 움직여도 자동 재투입하지 않는다.
    bool h1 = !fault_imu1, h2 = !fault_imu2;   // h = healthy (사용 가능)

    // ---------- IMU 융합 (진짜 redundancy) ----------
    // 둘 다 정상: 불일치 검사 후 평균 / 하나만 정상: 그것만 / 중재 불가: fault
    if (h1 && h2) {
      if (disagreeSample) {
        if (disagreeSince == 0) disagreeSince = nowMs;
        else if (nowMs - disagreeSince >= IMU_DISAGREE_MS) {
          if (!fault_disagree) Serial.println("[FAULT] DUAL IMU DISAGREEMENT");
          fault_disagree = true;
        }
      } else {
        disagreeSince = 0;
      }
    } else {
      disagreeSince = 0;
    }

    float fg[3], fa[3];   // 융합(fused) 자이로/가속도 (IMU1 센서 축)
    if (h1 && h2 && !fault_disagree) {
      for (int k = 0; k < 3; k++) { fg[k] = (g1[k]+g2[k])*0.5f; fa[k] = (a1[k]+a2[k])*0.5f; }
      active_imus = 2;
    } else if (h1 && !h2) {
      for (int k = 0; k < 3; k++) { fg[k] = g1[k]; fa[k] = a1[k]; }
      active_imus = 1;
    } else if (h2 && !h1) {
      for (int k = 0; k < 3; k++) { fg[k] = g2[k]; fa[k] = a2[k]; }
      active_imus = 1;
    } else {
      // 둘 다 freeze 또는 서로 안 맞음 -> 신뢰 불가
      active_imus = 0;
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
      safety_lock = true;
      mixer_scaled = false;
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      resetYawAuthority(yawAuthority);
      yaw_authority_state = YAW_AUTH_NORMAL;
      resetAllocationTelemetry();
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      enterLockedState(wasLocked);
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      continue;   // 이번 tick의 나머지(자세 추정, PID)는 건너뛴다
    }

    // [센서 축 → body 축 변환] IMU1 센서 축과 기체(body) 축이 서로 달라 축을 바꿔 부른다
    //  body x(roll) = 센서 y, body y(pitch) = -센서 x, body z(yaw) = -센서 z (gyro). accel z는 + 그대로
    //  gyro z만 부호가 반대인 이유는 아래 영어 주석 참고 (yaw 부호는 벤치 재검증 전까지 그대로 둠)
    // Roll/pitch gyro now matches the validated accel transform; the inverted
    // signs caused estimator lag and rate-loop anti-damping.
    // bodyGz stays unchanged pending separate bench yaw re-verification.
    const float bodyGx =  fg[1];
    const float bodyGy = -fg[0];
    const float bodyGz = -fg[2];
    const float bodyAx =  fa[1] * ACCEL_SCALE;
    const float bodyAy = -fa[0] * ACCEL_SCALE;
    const float bodyAz =  fa[2] * ACCEL_SCALE;
    gyroX = bodyGx; gyroY = bodyGy; gyroZ = bodyGz;
    accX = bodyAx; accY = bodyAy; accZ = bodyAz;

    // ---------- 자세 추정 (상보필터) ----------
    // ① 자이로 적분: 각도 += 각속도 × dt (빠르고 부드럽지만 bias 때문에 천천히 틀어진다)
    angleX += bodyGx * realDt;
    angleY += bodyGy * realDt;
    if (fabsf(bodyGz) > YAW_DEADZONE) angleZ += bodyGz * realDt;   // yaw는 가속도로 보정할 수 없어 아주 작은 값은 버림

    // ② 가속도로 구한 기울기: 정지 시 가속도 = 중력 방향 → atan2로 roll/pitch 계산 (안 틀어지지만 진동에 약하다)
    float alpha = compute_alpha(bodyAx, bodyAy, bodyAz);
    float accAngleX = atan2f(bodyAy, sqrtf(bodyAx*bodyAx + bodyAz*bodyAz)) * 180.0f / PI;
    float accAngleY = atan2f(-bodyAx, sqrtf(bodyAy*bodyAy + bodyAz*bodyAz)) * 180.0f / PI;
    // ③ 섞기: 대부분(alpha ≈ 0.999)은 자이로, 아주 조금만 가속도 쪽으로 끌어당겨 드리프트를 없앤다
    angleX = alpha * angleX + (1.0f - alpha) * accAngleX;
    angleY = alpha * angleY + (1.0f - alpha) * accAngleY;

    // outerCnt는 safety-lock 경로에서 매 tick reset되므로 독립 divider를 쓴다.
    // 이로써 armed/disarmed 모두 정확히 250Hz, K_MAG tau≈4s를 유지한다.
    if (magFusionCnt == 0 && mag_enabled) {
      MagSnapshot mag;
      if (readMagSnapshot(mag)
          // 코어0 갱신이 코어1의 nowMs보다 미래일 수 있어 부호 있는 차이로 비교한다.
          && (int32_t)(nowMs - mag.sample_ms) <= (int32_t)MAG_STALE_MS
          && mag.x*mag.x + mag.y*mag.y + mag.z*mag.z > 1e-6f) {
        // 모터 전류 자기 간섭 보정 (스로틀에 비례하는 성분 빼기)
        const float magThrDelta = (float)(base_throttle - MAG_THROTTLE_REF_US);
        mag.x -= mag_comp_x * magThrDelta;
        mag.y -= mag_comp_y * magThrDelta;
        mag.z -= mag_comp_z * magThrDelta;
        publishMagTelemetry(mag.x, mag.y, mag.z, false);
        // 기울기 보정 heading → 기준 오프셋 적용 → angleZ를 K_MAG만큼 그쪽으로 끌어당김
        float heading = tiltCompensatedMagHeadingDeg(
            mag.x, mag.y, mag.z, angleX, angleY);
        if (isfinite(heading)) {
          magHeading = heading;
          const bool update_reference = takeMagReferenceRequest();
          if (update_reference) {
            // Relative heading: enabling mag or resetting angleZ must not
            // command a turn toward magnetic north.
            magYawReferenceOffset = wrapDeg(angleZ - heading);
          }
          float referencedHeading =
              wrapDeg(heading + magYawReferenceOffset);
          angleZ += magYawCorrectionDeg(angleZ, referencedHeading, K_MAG);
        }
      }
    }
    magFusionCnt++;
    if (magFusionCnt >= OUTER_DIV) magFusionCnt = 0;

    // ---------- 안전 검사 ----------
    // 과도 기울기(60° 초과): 뒤집히는 중으로 보고 즉시 잠금. fault_attitude는 다음 start까지 유지
    if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
      if (!fault_attitude) Serial.printf("[FAULT] OVER-TILT R:%.1f P:%.1f\n", angleX, angleY);
      fault_attitude = true;
      safety_lock = true;
    }
    // resume: 하강을 멈추고 호버 스로틀로 조종을 돌려준다 (스로틀 창도 호버 ± CTRL_MARGIN으로 재설정)
    if (takeFailsafeResumeRequest()) {
      const ResumeRefusalReason refusal = resumeRefusalReason(nowMs);
      if (refusal != RESUME_ALLOWED) {
        logResumeRefusal(refusal);
      } else {
        fs_phase = FS_NONE;
        fault_rc = false;
        base_throttle = (int)lroundf(hover_est);
        min_throttle = max(1050, base_throttle - CTRL_MARGIN);
        max_throttle = min(1900, base_throttle + CTRL_MARGIN);
        Serial.printf(">>> RESUME hover=%d\n", base_throttle);
      }
    }
    // 잠금 해제 첫 tick의 D kick과 이전 목표 rate 잔류를 제거한다.
    // 이 블록은 RC timeout 진입 가드보다 먼저 실행해야 한다. 뒤에 두면 같은
    // tick에서 새로 진입한 FS_DESCENDING까지 FS_NONE으로 덮어쓰게 된다.
    if (wasLocked && !safety_lock) {
      // start가 남긴 이전 비행의 terminal phase와 start/abort 경합 결과를
      // Core 1에서 해제해 fs_phase의 런타임 writer를 pid_task 하나로 유지한다.
      fs_phase = FS_NONE;
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      outerCnt = 0;
      fs_first_enter_ms = 0;
      fs_first_enter_valid = false;
      resetYawAuthority(yawAuthority);
      yaw_authority_state = YAW_AUTH_NORMAL;
      resetAllocationTelemetry();
      wasLocked = false;
    }

    // 호버 스로틀 학습: 시동 중 + 정상 비행 + 수평 + 수직 가속 없음(|a|≈1g) + 이륙 스로틀 이상일 때만 샘플로 인정
    const float accelMag =
        sqrtf(accX*accX + accY*accY + accZ*accZ);   // |accel| (g)
    const bool hoverSampleEligible =
        !safety_lock && fs_phase == FS_NONE &&
        fabsf(angleX) < HOVER_LEVEL_MAX_DEG &&
        fabsf(angleY) < HOVER_LEVEL_MAX_DEG &&
        fabsf(accelMag - 1.0f) <= HOVER_ACCEL_TOL_G &&
        base_throttle > HOVER_MIN_THROTTLE_US;
    updateHoverThrottleEstimator(
        hoverTracker, hoverSampleEligible, base_throttle, nowMs, realDt,
        HOVER_LPF_TAU_S, HOVER_VALID_MS);
    hover_est = hoverTracker.estimate_us;
    hover_valid = hoverTracker.valid;

    // RC 타임아웃은 유효 호버 추정치가 없거나 보수적 절대 지상 스로틀이면
    // 즉시 컷하고, 그 외에는 추정 호버를 기준으로 자동착륙한다. hover_est는
    // 하강 목표에만 쓰며, 오염된 높은 값이 공중 즉시컷 경계를 올리지 못한다.
    // 진입 블록 전체를 fs_phase 가드 안에 둔다 — 가드가 없으면 하강 내내
    // rcTimedOut이 참이라 이 블록이 1kHz로 재실행되고, Serial.println이 TX
    // 버퍼를 포화시켜 pid_task를 블로킹하면 500ms 태스크 워치독이 비행 중
    // 재부팅을 일으킨다.
    // [RC 끊김 처리 3갈래] 호버 추정 없음 → 즉시 컷 / 스로틀 ≤ 1150 (지상) → 즉시 컷 / 그 외 → 자동착륙 진입
    if (fs_phase == FS_NONE && !safety_lock && rcTimedOut(nowMs, lastRcMs)) {
      fault_rc = true;
      if (!hover_valid) {
        safety_lock = true;
        Serial.println("[FAULT] RC TIMEOUT -> CUT (NO HOVER EST)");  // 한 번만
      } else if (base_throttle <= FS_GROUND_CUT_MAX_US) {
        safety_lock = true;
        Serial.println("[FAULT] RC TIMEOUT -> GROUND CUT");  // 한 번만
      } else {
        // 자동착륙 진입: 현재 heading 유지, 진입 시각 기록, 착지 감지기 초기화, 프로브 딥 크기 계산
        fs_phase = FS_DESCENDING;
        fs_hold_yaw = angleZ;
        fs_enter_ms = nowMs;
        if (!fs_first_enter_valid) {
          fs_first_enter_ms = nowMs;
          fs_first_enter_valid = true;
        }
        landDet = {};
        landProbeConfig = FS_LAND_PROBE_CONFIG;
        bool dipClamped = false;
        fs_probe_dip_us = failsafeProbeDipUs(
            hover_est, FS_PROBE_DIP_FRAC, FS_PROBE_DIP_MIN_US,
            CTRL_MARGIN, dipClamped);
        landProbeConfig.dip_us = fs_probe_dip_us;
        if (dipClamped) {
          Serial.printf(
              "[WARN] AUTO-LAND PROBE DIP CLAMPED hover=%.1f applied=%d\n",
              hover_est, fs_probe_dip_us);
        }
        fs_probe_state = FS_PROBE_WAIT;
        fs_probe_no_response = 0;
        fs_probe_response_g = 0.0f;
        fs_probe_blocked_logged = false;
        base_throttle = failsafeDescentThrottle(
            (int)lroundf(hover_est), FS_DESCENT_DELTA_US);
        Serial.println("[FAULT] RC TIMEOUT -> AUTO-LAND");   // 한 번만
      }
    }

    // 하강 중에는 목표를 매 tick 덮어쓴다. rc/rcr 파서도 하강 중 목표 변경을
    // 막으며, 링크 복구만으로는 하강을 끝내지 않는다(명시적 resume 필요).
    if (fs_phase == FS_DESCENDING) {
      // 목표 자세 = 수평(트림만큼) + 진입 때 heading, yaw 회전 명령 없음
      targetAngleX = trim_roll;
      targetAngleY = trim_pitch;
      targetAngleZ = fs_hold_yaw;
      targetYawRate = 0.0f;
      const int descentThrottle = failsafeDescentThrottle(
          (int)lroundf(hover_est), FS_DESCENT_DELTA_US);

      const uint32_t elapsed = nowMs - fs_enter_ms;
      // 프로브는 계속 돌리되 **착지 판정에는 쓰지 않는다.**
      //
      // 2026-08-01의 "010251 공중 CUT_LANDED" 해석은 철회됐다. 20Hz 단일
      // 샘플을 자유낙하로 오독했으며 1kHz 기록과 조종자 확인은 접지를 지지한다.
      // 따라서 "지면 응답이 공중보다 큼 / 판별 방향 반대 / 오분류 9/18"도
      // 근거가 없다. 유효한 실패 관측은 015448의 접지 응답 10회가 임계 0.03
      // 위여서 착지를 확정하지 못했다는 것이다. 023904의 접지 전 응답 2회만으로
      // 공중 오검출률을 검증할 수 없어 검출기는 제어에 복귀시키지 않았다.
      //
      // 따라서 자동착륙은 **시간 기반 하강**이다: FS_MAX_MS 동안 내려간 뒤
      // 백스톱이 자른다. 이 경로는 CUT_LANDED를 선택하지 않지만, 접지 전에
      // 예산이 끝나면 TIMEOUT 공중 컷은 가능하다. 접지 후에는 남은 시간만큼
      // 프롭이 돈다. 고도·수직속도 피드백이나 접지 확인은 하지 않는다.
      //
      // 프로브 딥은 실제 스로틀에 계속 적용된다 — 텔레메트리(41~43)로 기록해 다음 판별식을
      // 오프라인 데이터로 설계할 수 있다. 분석은 docs/failsafe_land_research.md.
      const bool probe_says_landed = updateLandDetector(
          landDet, accelMag, elapsed, descentThrottle,
          landProbeConfig);
      (void)probe_says_landed;
      const bool landed = false;
      fs_probe_state = landDet.probe_state;
      fs_probe_no_response = landDet.no_response_count;
      fs_probe_response_g = landDet.last_response_g;
      if (landDet.probe_state == FS_PROBE_BLOCKED &&
          !fs_probe_blocked_logged) {
        fs_probe_blocked_logged = true;
        Serial.println("[WARN] AUTO-LAND PROBE BLOCKED");  // 하강당 최대 1회
      }

      const bool probeActive = landDetectorProbeActive(landDet);
      checkProbeDelivery = probeActive;
      probeReferenceThrottle = descentThrottle;
      probeElapsedMs = elapsed;
      base_throttle =
          descentThrottle -
          (probeActive ? fs_probe_dip_us : 0);
      // 딥 tick에만 collective 하한을 요청 딥만큼 넓힌다. 자세 차동이 기존
      // 하한을 밀어 올려 딥을 자르면 검출기가 공중을 지면으로 오판할 수 있다.
      // 상한은 그대로 두고 1050us 절대 하한도 기존 clamp로 보존한다.
      min_throttle = max(
          1050,
          descentThrottle - CTRL_MARGIN - (probeActive ? fs_probe_dip_us : 0));
      max_throttle = min(1900, descentThrottle + CTRL_MARGIN);
      const FailsafePhase next = failsafeStep(landed, elapsed, FS_MAX_MS);
      if (next != FS_DESCENDING) {
        fs_phase = next;
        safety_lock = true;      // 아래 safety_lock 블록이 이번 tick에 정리한다
        Serial.printf(">>> AUTO-LAND END phase=%d\n", (int)next);
      }
    }
    // [잠금 상태] 모든 제어 상태(적분, 목표, 필터)를 비우고 모터 정지. 다음 시동 때 이전 값이 튀지 않게 한다
    if (safety_lock) {
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
      mixer_scaled = false;
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      targetYawRate = 0.0f;
      targetAngleZ = angleZ;
      yaw_hold_now = false;
      resetYawAuthority(yawAuthority);
      yaw_authority_state = YAW_AUTH_NORMAL;
      resetAllocationTelemetry();
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      outerCnt = 0;
      enterLockedState(wasLocked);
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      continue;
    }

    // ---------- Outer loop (250Hz): 각도 -> 목표 각속도 ----------
    // yaw는 각속도 명령 + 자동 heading 잠금. hold가 아닌 동안 setpoint를
    // 매 tick 현재 heading으로 슬레이빙해 stale setpoint를 원천 차단한다.
    // ① yaw 권한 감시: 직전 tick의 yaw_scale로 상태 갱신. 여기까지 왔으면 시동 중·잠금 아님이라 armed=true, locked=false
    updateYawAuthority(
        yawAuthority, mixer_yaw_scale, targetYawRate, bodyGz,
        true, fs_phase != FS_NONE, false, nowMs,
        YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS);
    yaw_authority_state = static_cast<int>(yawAuthority.state);

    // ② 정상 yaw 명령(잠금/각속도) 계산 → ③ 권한 상태가 NORMAL이 아니면 "회전 멈추기"로 대체
    const YawOuter yawOuter = updateYawOuter(
        targetYawRate, bodyGz, angleZ, targetAngleZ,
        yaw_hold_now, yaw_hold_override,
        YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS);
    const float normalTargetRateYaw = yawTargetRateDps(
        yawOuter, targetYawRate, angleZ, Kp_Angle_Yaw, MAX_TARGET_RATE_YAW);
    const YawAuthorityCommand yawCommand = applyYawAuthority(
        yawAuthority.state, angleZ, yawOuter.target_angle_deg,
        normalTargetRateYaw, yawOuter.hold);
    targetAngleZ = yawCommand.target_angle_deg;
    yaw_hold_now = yawCommand.hold;

    // 4 tick에 한 번(250Hz): 각도 오차 × Kp_Angle = 목표 각속도 (±300dps 제한)
    //  예) 목표 0°, 현재 5° 기울어짐, Kp 6 → -30dps로 되돌리라는 명령
    // 권한 부족 상태에서는 나머지 tick에서도 yaw 목표 각속도를 0으로 유지한다
    if (outerCnt == 0) {
      targetRateRoll = constrain((targetAngleX - angleX) * Kp_Angle_Roll,
                                 -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRatePitch = constrain((targetAngleY - angleY) * Kp_Angle_Pitch,
                                  -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRateYaw = yawCommand.target_rate_dps;
    } else if (yawAuthority.state != YAW_AUTH_NORMAL) {
      targetRateYaw = 0.0f;
    }
    outerCnt++;
    if (outerCnt >= OUTER_DIV) outerCnt = 0;

    // ---------- Inner loop (1kHz): 각속도 PID ----------
    // 오차 = 목표 각속도 - 실제 각속도 (dps)
    float eRoll  = targetRateRoll  - bodyGx;
    float ePitch = targetRatePitch - bodyGy;
    float eYaw   = targetRateYaw   - bodyGz;

    // D는 gyro 미분에만 건다 (setpoint/outer loop 오염 제거)
    float dRoll  = lpfD_Roll.update((bodyGx - prevGyroX) / realDt);
    float dPitch = lpfD_Pitch.update((bodyGy - prevGyroY) / realDt);
    float dYaw   = lpfD_Yaw.update((bodyGz - prevGyroZ) / realDt);
    prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;

    // 이번 tick 출력에 쓰는 I항 (적분 갱신은 allocator 결과를 본 뒤 아래에서 한다)
    const float iRollUsed = iTermRoll;
    const float iPitchUsed = iTermPitch;
    const float iYawUsed = iTermYaw;
    // PID 출력(µs) = Kp × 오차 + I항 - Kd × (각속도 변화율). D는 오차가 아니라 측정값 미분이라 부호가 -
    float pidRoll  = Kp_Rate_Roll *eRoll  + iRollUsed  - Kd_Rate_Roll *dRoll;
    float pidPitch = Kp_Rate_Pitch*ePitch + iPitchUsed - Kd_Rate_Pitch*dPitch;
    // yaw 각도 유지가 꺼져 있어도 rate 감쇠(target 0)는 항상 동작한다.
    // 모터 토크 불균형과 롤·피치 보정의 반작용으로 생기는 자유 회전을 억제한다.
    float pidYaw   = Kp_Rate_Yaw*eYaw + iYawUsed - Kd_Rate_Yaw*dYaw;

    // ---------- 모터 desaturation + 포화 기반 anti-windup ----------
    const int throttle = base_throttle; 
    // PID 출력 3개 + 스로틀 → 모터 4개 (범위를 넘으면 roll/pitch > yaw > 스로틀 순으로 지킨다)
    MotorMix mix = mixAndDesaturate(pidRoll, pidPitch, pidYaw,
                                    throttle, min_throttle, max_throttle);
    mixer_yaw_scale = mix.yaw_scale;   // 다음 tick updateYawAuthority()의 입력
    publishAllocationTelemetry(
        mix, pidRoll, pidPitch, pidYaw,
        iRollUsed, iPitchUsed, iYawUsed, yaw_authority_state);
    // 프로브 딥 중이면: 실제로 내려간 스로틀(기준 - 적용 collective)이 요청 딥의 80% 이상인지 확인
    if (checkProbeDelivery) {
      const float deliveredUs =
          (float)probeReferenceThrottle - mix.collective_us;
      const bool deliveryValid =
          deliveredUs >= FS_PROBE_MIN_DELIVERY_FRAC * fs_probe_dip_us;
      recordLandDetectorProbeDelivery(
          landDet, probeElapsedMs, deliveryValid, landProbeConfig);
    }

    // scale은 자세 명령이 실제로 잘린 경우다. collective 이동만 일어난 경우에는
    // 자세 authority가 보존되므로 적분을 계속한다.
    // 적분은 모터 출력 기여(µs) 단위로 누적하고 I_TERM_MAX_US로 제한한다.
    // (이전 errorSum ±200deg 클램프는 기본 Ki에서 최대 기여 1µs라 적분이
    //  사실상 없는 것과 같아 CG 치우침 같은 정상상태 오차를 못 잡았다.)
    iTermRoll = conditionalAxisIntegral(
        iTermRoll, Ki_Rate_Roll * eRoll * realDt, pidRoll,
        mix.rp_scale, throttle, I_TERM_MAX_US);
    iTermPitch = conditionalAxisIntegral(
        iTermPitch, Ki_Rate_Pitch * ePitch * realDt, pidPitch,
        mix.rp_scale, throttle, I_TERM_MAX_US);
    // yaw 적분은 rate/hold 두 모드 모두에서 돈다. 안쪽 루프의 임무가
    // "목표 각속도 추종"이고, P 단독으로는 모터 토크 불균형 같은 일정
    // 외란에서 정상상태 각속도 오차가 남아 정착 임계치 아래로 내려가지
    // 않는다(2026-07-27 실측 최대 +16.6dps). 그러면 자동 잠금이 영영
    // 걸리지 않는다.
    iTermYaw = conditionalAxisIntegral(
        iTermYaw, Ki_Rate_Yaw * eYaw * realDt, pidYaw,
        mix.yaw_scale, throttle, I_TERM_MAX_US);

    // 모터 PWM의 단일 writer는 PID task로 유지한다.
    if (safety_lock) {
      resetAllocationTelemetry();
      yaw_authority_state = YAW_AUTH_NORMAL;
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      enterLockedState(wasLocked);
      continue;
    }
    // 최종 PWM 출력 (FL, RR, FR, RL) + 텔레메트리용 복사
    writeMotor(pinM1, mix.motor[0]);
    writeMotor(pinM2, mix.motor[1]);
    writeMotor(pinM3, mix.motor[2]);
    writeMotor(pinM4, mix.motor[3]);
    motorOut[0] = mix.motor[0]; motorOut[1] = mix.motor[1];
    motorOut[2] = mix.motor[2]; motorOut[3] = mix.motor[3];
    tgtRate[0] = targetRateRoll; tgtRate[1] = targetRatePitch; tgtRate[2] = targetRateYaw;
  }
}

// ==========================================================
// 9. UDP 태스크 (Core 0) — char* 파싱 (String heap 회피)
// ==========================================================
// [명령 파싱 공통] Arduino String은 힙 메모리를 계속 할당/해제해 장시간 실행 시 조각화될 수 있어,
//  고정 버퍼(packetBuffer) 위에서 char* 포인터로 직접 자른다
// 앞뒤 공백/개행을 제거한 문자열의 시작 포인터를 돌려준다 (버퍼를 직접 수정)
static char *trimCommand(char *s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  char *end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) end--;
  *end = '\0';
  return s;
}

// 문자열 전체가 숫자일 때만 성공 ("1.5abc", "", "nan" 같은 입력은 거부). strtof가 멈춘 위치(end) 뒤에 공백 외 글자가 있으면 실패
static bool parseFloatStrict(const char *text, float &out) {
  char *end;
  float value = strtof(text, &end);
  if (end == text || !isfinite(value)) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

// 정수 버전 (10진수)
static bool parseIntStrict(const char *text, long &out) {
  char *end;
  long value = strtol(text, &end, 10);
  if (end == text) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

// rc/rcr이 받은 목표 자세를 반영하고 RC 워치독(lastRcMs)을 갱신한다
//  x, y: 목표 roll/pitch (°), z: 목표 yaw 각도 (hasYaw일 때만)
static bool setRcTargets(float x, float y, float z, bool hasYaw) {
  if (!isfinite(x) || !isfinite(y) || (hasYaw && !isfinite(z))) return false;
  if (fs_phase == FS_DESCENDING) {
    // 하강 중엔 목표는 바꾸지 않고 "링크 살아 있음"만 기록 (resume 판정에 필요)
    lastRcMs = millis();
    return true;
  }
  // 트림을 더한 뒤 클램프해야 총합이 ±30°로 제한된다.
  targetAngleX = constrain(x + trim_roll,  -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetAngleY = constrain(y + trim_pitch, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  if (hasYaw) targetAngleZ = z;
  lastRcMs = millis();
  return true;
}

// 표준 형식: rc <seq> <roll> <pitch> <yaw>
// 벤치 수동 테스트용으로 rc <roll> <pitch> [yaw]도 허용한다.
// strtok_r: 문자열을 공백/탭 기준으로 하나씩 잘라 낸다. save에 다음 위치를 기억해 여러 번 이어서 호출
static void handleRcCommand(char *buf) {
  char *save = nullptr;
  char *token = strtok_r(buf, " \t", &save); // "rc"
  (void)token;
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (count == 4) {
    // seq는 부호 없는 정수만. strtoul은 "-1"도 큰 양수로 바꿔 버리므로 부호 문자를 먼저 거부한다
    if (arg[0][0] == '-' || arg[0][0] == '+') return;
    char *seqEnd;
    errno = 0;
    unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
    if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
        seqLong > 0xFFFFFFFFUL) return;

    float x, y, z;
    if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
        !parseFloatStrict(arg[3], z)) return;

    // [패킷 번호 검사] UDP는 순서가 뒤바뀌거나 중복될 수 있다 → 직전보다 큰 번호만 받아들인다
    //  번호가 2 이상 건너뛰면 그 사이 개수만큼 손실로 센다
    uint32_t seq = (uint32_t)seqLong;
    rcTotalPkts = rcTotalPkts + 1;
    if (rcSeqValid) {
      int32_t advance = (int32_t)(seq - lastRcSeq); // uint32 wrap도 정상 처리
      if (advance <= 0) {
        rcDroppedPkts = rcDroppedPkts + 1;
        return; // 지연 도착/중복 패킷 폐기
      }
      if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
    }
    lastRcSeq = seq;
    rcSeqValid = true;
    setRcTargets(x, y, z, true);
    return;
  }

  if (count == 2 || count == 3) {
    float x, y, z = 0.0f;
    if (!parseFloatStrict(arg[0], x) || !parseFloatStrict(arg[1], y)) return;
    if (count == 3 && !parseFloatStrict(arg[2], z)) return;
    setRcTargets(x, y, z, count == 3);
  }
}

// rcr <seq> <roll> <pitch> <yaw_rate>
// yaw는 각속도(dps) 명령이다. seq 처리와 워치독 급이는 rc와 상태를 공유한다.
static void handleRcrCommand(char *buf) {
  char *save = nullptr;
  (void)strtok_r(buf, " \t", &save); // "rcr"
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (count != 4) return;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (arg[0][0] == '-' || arg[0][0] == '+') return;
  char *seqEnd;
  errno = 0;
  unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
  if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
      seqLong > 0xFFFFFFFFUL) return;

  float x, y, rate;
  if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
      !parseFloatStrict(arg[3], rate)) return;

  uint32_t seq = (uint32_t)seqLong;
  rcTotalPkts = rcTotalPkts + 1;
  if (rcSeqValid) {
    int32_t advance = (int32_t)(seq - lastRcSeq);
    if (advance <= 0) {
      rcDroppedPkts = rcDroppedPkts + 1;
      return;
    }
    if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
  }
  lastRcSeq = seq;
  rcSeqValid = true;

  if (!setRcTargets(x, y, 0.0f, false)) return;   // roll/pitch만, yaw 각도는 건드리지 않는다
  if (fs_phase == FS_DESCENDING) return;
  targetYawRate = constrain(rate, -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW);
}

// 게인 명령: 앞 2글자 = 어떤 게인, 나머지 = 값 (예: "rp0.6" → roll/pitch 각속도 Kp = 0.6). 0~100만 허용
static void handleGainCommand(const char *buf) {
  if (strlen(buf) < 3) return;
  float value;
  if (!parseFloatStrict(buf + 2, value) || value < 0.0f || value > 100.0f) return;
  if (!configChangeAllowed("PID gain")) return;

  // Cascade 전용 공통 명령
  if      (strncmp(buf, "rp", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ri", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "rd", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "ap", 2) == 0) { Kp_Angle_Roll = value; Kp_Angle_Pitch = value; }
  else if (strncmp(buf, "ar", 2) == 0) Kp_Angle_Roll = value;
  else if (strncmp(buf, "at", 2) == 0) Kp_Angle_Pitch = value;
  else if (strncmp(buf, "ay", 2) == 0) Kp_Angle_Yaw = value;
  else if (strncmp(buf, "yp", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "yi", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "yd", 2) == 0) Kd_Rate_Yaw = value;

  // scripts/tune_pid.py 명령 호환: P/I/D는 inner rate PID에 대응
  else if (strncmp(buf, "pa", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ia", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "da", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "pr", 2) == 0) Kp_Rate_Roll = value;
  else if (strncmp(buf, "ir", 2) == 0) Ki_Rate_Roll = value;
  else if (strncmp(buf, "dr", 2) == 0) Kd_Rate_Roll = value;
  else if (strncmp(buf, "pp", 2) == 0) Kp_Rate_Pitch = value;
  else if (strncmp(buf, "ip", 2) == 0) Ki_Rate_Pitch = value;
  else if (strncmp(buf, "dp", 2) == 0) Kd_Rate_Pitch = value;
  else if (strncmp(buf, "py", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "iy", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "dy", 2) == 0) Kd_Rate_Yaw = value;
}

// 첫 14개 필드는 기존 PC 스크립트와 호환된다. 뒤 필드는 cascade 진단 확장:
// fault_imu1,fault_imu2,fault_disagree,active_imus,scaled,fault_attitude,
// calibration_ok,armed. 그 뒤 Tier1 8개와 MagHeading, Mag_X/Y/Z, Yaw_Hold,
// Failsafe_Phase, Trim_Roll/Pitch, Hover_Est/Valid와 프로브 상태/연속
// 무반응/최근 차분 반응, IMU1 gyro/accel XYZ, IMU2 gyro/accel XYZ를
// 보낸 뒤 목표 roll/pitch/yaw 각도와 필드 59 Mag_Enabled, 3901-L0X 5개
// 필드, 필드 65 Mag_Cal_Active, 필드 66~75 allocator/PID/I/yaw authority를
// append-only로 보낸다. IMU별 값은 융합값과 같은 body frame이다.
// 20Hz CSV 텔레메트리 한 줄 전송. Core 1이 쓰는 값은 먼저 스냅샷으로 복사한 뒤 출력한다
static void sendTelemetry() {
  if (!connectionEstablished) return;
  bool criticalFault = (active_imus == 0) || fault_attitude || !calibration_ok;
  const float hoverEstSnapshot = hover_est;
  const bool hoverValidSnapshot = hover_valid;
  const uint8_t probeStateSnapshot = fs_probe_state;
  const uint8_t probeNoResponseSnapshot = fs_probe_no_response;
  const float probeResponseSnapshot = fs_probe_response_g;
  const AllocationTelemetrySample allocationSample =
      readAllocationTelemetrySnapshot();
  const ImuTelemetrySample imuSample = readImuSampleSnapshot();
  float magTelemXSnapshot = 0.0f;
  float magTelemYSnapshot = 0.0f;
  float magTelemZSnapshot = 0.0f;
  bool magTelemCalActiveSnapshot = false;
  readMagTelemetry(
      magTelemXSnapshot, magTelemYSnapshot, magTelemZSnapshot,
      magTelemCalActiveSnapshot);
  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%ld,%d,%ld,%ld,%d,%d,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
             angleX, angleY, angleZ,
             gyroX, gyroY, gyroZ,
             accX, accY, accZ,
             base_throttle,
             (int)fault_rc, (int)criticalFault,
             (unsigned long)rcTotalPkts, (unsigned long)rcDroppedPkts,
             (int)fault_imu1, (int)fault_imu2, (int)fault_disagree,
             active_imus, (int)allocationSample.scaled, (int)fault_attitude,
             (int)calibration_ok, (int)!safety_lock,
             motorOut[0], motorOut[1], motorOut[2], motorOut[3], pidLoopHz,
             tgtRate[0], tgtRate[1], tgtRate[2], magHeading,
             magTelemXSnapshot, magTelemYSnapshot, magTelemZSnapshot,
             (int)yaw_hold_now,
             (int)fs_phase, trim_roll, trim_pitch,
             hoverEstSnapshot, (int)hoverValidSnapshot,
             (int)probeStateSnapshot, (int)probeNoResponseSnapshot,
             probeResponseSnapshot,
             imuSample.imu1GyroX, imuSample.imu1GyroY, imuSample.imu1GyroZ,
             imuSample.imu1AccelX, imuSample.imu1AccelY, imuSample.imu1AccelZ,
             imuSample.imu2GyroX, imuSample.imu2GyroY, imuSample.imu2GyroZ,
             imuSample.imu2AccelX, imuSample.imu2AccelY, imuSample.imu2AccelZ,
             targetAngleX, targetAngleY, targetAngleZ, (int)mag_enabled,
             (long)msp_range_mm, msp_range_quality,
             (long)msp_flow_x, (long)msp_flow_y, msp_flow_quality,
             (int)magTelemCalActiveSnapshot,
             allocationSample.rp_scale, allocationSample.yaw_scale,
             allocationSample.collective_us,
             allocationSample.pid_roll_us, allocationSample.pid_pitch_us,
             allocationSample.pid_yaw_us,
             allocationSample.i_roll_us, allocationSample.i_pitch_us,
             allocationSample.i_yaw_us,
             allocationSample.yaw_authority_state);
  udp.endPacket();
}

// 링 버퍼에서 최대 50개를 꺼내 ZIMU 패킷 하나로 전송
//  각 샘플의 절대 시각은 첫 샘플 시각(first_t_us)에 dt_us를 누적해 복원하고, 패킷 첫 샘플 시각을 헤더 base_t_us에 넣는다
static bool sendImuRawBatch() {
  if (!connectionEstablished) return false;
  uint32_t queued = imuRawRingCount(imuRawRing);
  if (queued == 0) return false;
  const uint8_t nSamples = static_cast<uint8_t>(
      min(queued, static_cast<uint32_t>(IMU_RAW_BATCH_MAX)));
  uint32_t baseTUs = 0;
  uint8_t copied = 0;
  for (; copied < nSamples; copied++) {
    ImuRawSample sample = {};
    if (!imuRawRingPop(imuRawRing, sample)) break;
    uint32_t sampleUs;
    if (!rawConsumerTimeValid) {
      sampleUs = __atomic_load_n(&imuRawRing.first_t_us, __ATOMIC_ACQUIRE);
    } else {
      sampleUs = rawConsumerLastUs + sample.dt_us;
    }
    if (copied == 0) baseTUs = sampleUs;
    rawConsumerLastUs = sampleUs;
    rawConsumerTimeValid = true;
    imuRawBatch[copied] = sample;
  }
  if (copied == 0) return false;

  const uint32_t dropped =
      __atomic_load_n(&imuRawRing.dropped, __ATOMIC_RELAXED);
  const size_t packetSize = buildImuRawDatagram(
      imuRawDatagram, rawBatchSeq, baseTUs, dropped, imuRawBatch, copied);
  udp.beginPacket(laptopIP, laptopPort);
  udp.write(reinterpret_cast<const uint8_t *>(&imuRawDatagram), packetSize);
  udp.endPacket();
  rawBatchSeq++;
  return true;
}

// ZCAL 패킷 전송: PC가 ZIMU 원값을 물리 단위로 바꾸는 데 필요한 보정 상수
static bool sendImuCalibration() {
  if (!connectionEstablished) return false;
  memcpy(imuCalDatagram.magic, "ZCAL", 4);
  imuCalDatagram.version = IMU_RAW_VERSION;
  memset(imuCalDatagram.reserved, 0, sizeof(imuCalDatagram.reserved));
  for (int axis = 0; axis < 3; axis++) {
    imuCalDatagram.gyro_bias1[axis] = gyro_bias1[axis];
    imuCalDatagram.gyro_bias2[axis] = gyro_bias2[axis];
    imuCalDatagram.imu2_sign[axis] = IMU2_SIGN[axis];
  }
  imuCalDatagram.accel_scale1 = accel_scale1;
  imuCalDatagram.accel_scale2 = accel_scale2;
  imuCalDatagram.gyro_scale = GYRO_SCALE;
  imuCalDatagram.accel_scale = ACCEL_SCALE;
  udp.beginPacket(laptopIP, laptopPort);
  udp.write(reinterpret_cast<const uint8_t *>(&imuCalDatagram),
            sizeof(imuCalDatagram));
  udp.endPacket();
  return true;
}

// 3901-L0X UART 폴링. **논블로킹**: available() 만큼만, 반복당 상한까지.
// 어떤 것도 제어하지 않는다 — 텔레메트리 기록 전용이다.
static void pollMspSensor(uint32_t nowMs) {
  int budget = MSP_SENSOR_MAX_BYTES_PER_TICK;
  while (budget-- > 0 && Serial1.available() > 0) {
    const MspParseResult result =
        mspSensorParseByte(mspSensor, (uint8_t)Serial1.read());
    if (result == MSP_PARSE_RANGEFINDER) {
      msp_range_mm = mspSensor.range.distance_mm;
      msp_range_quality = (int)mspSensor.range.quality;
      msp_last_range_ms = nowMs;
    } else if (result == MSP_PARSE_OPTIC_FLOW) {
      msp_flow_x = mspSensor.flow.motion_x;
      msp_flow_y = mspSensor.flow.motion_y;
      msp_flow_quality = (int)mspSensor.flow.quality;
      msp_last_flow_ms = nowMs;
    } else if (result == MSP_PARSE_UNKNOWN &&
               mspSensor.last_unknown_cmd != msp_reported_unknown) {
      // function 상수가 틀렸다면 "값이 안 나온다"가 아니라 이 줄로 드러난다.
      msp_reported_unknown = mspSensor.last_unknown_cmd;
      Serial.printf("[MSP] unknown function 0x%04X\n",
                    mspSensor.last_unknown_cmd);
    }
  }
  // 신선하지 않으면 quality 를 -1 로. 값 자체는 건드리지 않는다 —
  // distance_mm 는 모듈이 범위 밖에서 음수를 쓰므로 센티넬로 못 쓴다.
  if ((int32_t)(nowMs - msp_last_range_ms) > (int32_t)MSP_SENSOR_STALE_MS) {
    msp_range_quality = -1;
  }
  if ((int32_t)(nowMs - msp_last_flow_ms) > (int32_t)MSP_SENSOR_STALE_MS) {
    msp_flow_quality = -1;
  }
}

// [udp_task 한 반복 흐름 정리] (Core 0, 약 1ms마다)
//  1) UDP 명령 1개 수신 → 종류별 처리 (rc/rcr/start/stop/th/yaw/mag/raw/trim/gains/게인)
//  2) 자기계 샘플링(50Hz), 광류/라이다 UART 폴링
//  3) 10ms마다 ZIMU/ZCAL 바이너리 전송 검사 → 4) 50ms마다 CSV 텔레메트리 (한 반복에 패킷은 최대 1개)
void udp_task(void *pv) {
  uint32_t lastSend = 0;       // 마지막 CSV 텔레메트리 시각
  uint32_t lastRawCheck = 0;   // 마지막 ZIMU 전송 검사 시각
#if WIFI_LATENCY_DEBUG
  uint32_t prevTopUs = micros();
#endif
  while (true) {
#if WIFI_LATENCY_DEBUG
    uint32_t topUs = micros();
    uint32_t loopGapUs = topUs - prevTopUs;
    prevTopUs = topUs;
    if (loopGapUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), loopGapUs, LAT_UDP_LOOP);
    }
    uint32_t parseStartUs = micros();
#endif
    int packetSize = udp.parsePacket();
#if WIFI_LATENCY_DEBUG
    uint32_t parseDurUs = micros() - parseStartUs;
    if (parseDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), parseDurUs, LAT_UDP_PARSE);
    }
#endif
    if (packetSize) {
      // 보낸 쪽 주소를 기억 → 텔레메트리 목적지 (별도 접속 절차 없이 첫 패킷이 연결)
      laptopIP = udp.remoteIP();
      laptopPort = udp.remotePort();
      connectionEstablished = true;

      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1); // off-by-one 방지
      if (len > 0) {
        packetBuffer[len] = '\0';
        char *buf = trimCommand(packetBuffer);

        // "rcr"을 "rc"보다 먼저 검사해야 한다 ("rcr ..."도 앞 2글자는 "rc"라서)
        if (strncmp(buf, "rcr", 3) == 0 && (buf[3] == ' ' || buf[3] == '\t')) {
          handleRcrCommand(buf);
        }
        else if (strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
          handleRcCommand(buf);
        }
        // start: 하강 중 / 이미 시동 / 센서·캘리브레이션 문제면 거부, 통과하면 비행 상태 초기화 후 시동 요청
        else if (strcmp(buf, "start") == 0) {
          bool overTilt = fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE;
          bool noUsableImu = imu1_frozen_now && imu2_frozen_now;
          if (fs_phase == FS_DESCENDING) {
            Serial.println(">>> START REFUSED (auto-land descending)");
          }
          else if (!safety_lock || safety_arm_requested) {
            // 이미 시동 상태. 지상국은 start를 여러 번 재전송하므로, 지연
            // 도착한 중복 start가 비행 중 fault latch를 지우고 스로틀 창을
            // 리셋하는 것을 막는다.
            Serial.println(">>> START ignored (already armed)");
          }
          else if (!calibration_ok || overTilt || noUsableImu
                   || imu_disagree_now || mag_calibrating) {
            Serial.printf(">>> START REFUSED calib=%d tilt=%d imu=%d disagree=%d magcal=%d\n",
                          (int)calibration_ok, (int)overTilt,
                          (int)noUsableImu, (int)imu_disagree_now,
                          (int)mag_calibrating);
          } else {
            // safety_lock=true인 동안 Core 0 소유 비행 상태를 초기화한다.
            // hover tracker는 requestSafetyArm()이 Core 1 reset을 함께 요청한다.
            fault_rc = false;
            fault_imu1 = imu1_frozen_now;
            fault_imu2 = imu2_frozen_now;
            fault_disagree = false;
            fault_attitude = false;
            lastRcSeq = 0;
            rcSeqValid = false;
            lastRcMs = millis();
            base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;   // 시동 직후 낮은 스로틀 창 (이륙 전)
            targetAngleX = 0.0f;
            targetAngleY = 0.0f;
            targetAngleZ = 0.0f;
            angleZ = 0.0f;                  // 시동 순간 방향을 yaw 0°로
            requestMagReferenceUpdate();    // 자기계 기준도 새 0°에 맞춰 다시 잡는다
            fs_probe_state = FS_PROBE_WAIT;
            fs_probe_no_response = 0;
            fs_probe_response_g = 0.0f;
            requestSafetyArm();
            Serial.println(">>> START");
          }
        }
        else if (strcmp(buf, "stop") == 0) {
          requestSafetyDisarm();
          base_throttle = 1000;
          Serial.println(">>> STOP");
        }
        else if (strcmp(buf, "resume") == 0) {
          const ResumeRefusalReason refusal = resumeRefusalReason(millis());
          if (refusal != RESUME_ALLOWED) {
            logResumeRefusal(refusal);
          } else {
            requestFailsafeResume();
          }
        }
        // th<µs>: 스로틀 설정 (1000~1900). 모터 범위도 스로틀 ± CTRL_MARGIN으로 같이 옮긴다
        else if (strncmp(buf, "th", 2) == 0) {
          long parsed;
          if (parseIntStrict(buf + 2, parsed)) {
            int nb = constrain((int)parsed, 1000, 1900);
            base_throttle = nb;
            min_throttle = max(1050, nb - CTRL_MARGIN);
            max_throttle = min(1900, nb + CTRL_MARGIN);
          }
        }
        else if (strncmp(buf, "yaw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              yaw_hold_override = false;
              Serial.println(">>> Yaw hold OFF (auto)");
            } else if (!safety_lock) {
              // 비행 중 임의로 켜면 지상국 setpoint와 어긋난 채 최대 권한
              // 슬램이 날 수 있다. 시동 해제 상태에서만 켤 수 있게 한다.
              Serial.println(">>> Yaw hold refused (armed)");
            } else {
              targetAngleZ = angleZ;
              yaw_hold_override = true;
              Serial.println(">>> Yaw hold ON (override)");
            }
          }
        }
        // 비교 순서 주의: "magcal" → "magc" → "mag" (긴 이름부터 검사해야 짧은 이름에 잘못 걸리지 않는다)
        else if (strncmp(buf, "magcal", 6) == 0) {
          long enabled;
          if (parseIntStrict(buf + 6, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              stopMagCalibration();
            } else if (!safety_lock) {
              Serial.println(">>> Magcal refused (armed)");
            } else if (initMagnetometer()) {
              startMagCalibration();
            }
          }
        }
        // magc <x> <y> <z>: 모터 전류 자기 간섭 계수 설정 (µT/µs)
        else if (strncmp(buf, "magc", 4) == 0) {
          char *save = nullptr;
          char *token = strtok_r(buf, " \t", &save); // "magc"
          (void)token;
          char *arg[3] = {nullptr, nullptr, nullptr};
          int count = 0;
          while (count < 3 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
          if (count == 3 && strtok_r(nullptr, " \t", &save) == nullptr) {
            float x, y, z;
            if (parseFloatStrict(arg[0], x) && parseFloatStrict(arg[1], y) &&
                parseFloatStrict(arg[2], z)) {
              if (configChangeAllowed("Mag comp")) {
                mag_comp_x = x;
                mag_comp_y = y;
                mag_comp_z = z;
                Serial.printf(">>> mag comp = %.4f %.4f %.4f uT/us\n",
                              (double)mag_comp_x, (double)mag_comp_y, (double)mag_comp_z);
              }
            }
          }
        }
        // mag 0/1: 자기계 yaw 융합 끄기/켜기
        else if (strncmp(buf, "mag", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              mag_enabled = false;
              requestMagReferenceUpdate();
              Serial.println(">>> Mag OFF");
            } else if (mag_calibrating) {
              Serial.println(">>> Mag refused (magcal active)");
            } else if (!mag_ready && !safety_lock) {
              // First-time init does blocking I2C/delay(100); only when disarmed.
              Serial.println(">>> Mag refused (armed, not initialized)");
            } else if (initMagnetometer()) {
              requestMagReferenceUpdate();
              mag_enabled = true;
              Serial.println(">>> Mag ON");
            }
          }
        }
        // raw 0/1: ZIMU 원시 스트림 끄기/켜기. 켤 때는 쌓여 있던 옛 샘플을 버리고(tail = head) 시각 기준을 새로 잡는다
        else if (strncmp(buf, "raw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            const bool wasEnabled =
                __atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE);
            if (enabled == 0) {
              __atomic_store_n(&raw_stream_enabled, false, __ATOMIC_RELEASE);
              Serial.println(">>> Raw IMU OFF");
            } else {
              if (!wasEnabled) {
                const uint32_t head =
                    __atomic_load_n(&imuRawRing.head, __ATOMIC_ACQUIRE);
                __atomic_store_n(&imuRawRing.tail, head, __ATOMIC_RELEASE);
                rawProducerTimeValid = false;
                rawConsumerTimeValid = false;
                rawLastSendMs = millis();
                rawLastCalMs = rawLastSendMs - 1000U;
                __atomic_store_n(
                    &raw_stream_enabled, true, __ATOMIC_RELEASE);
              }
              Serial.println(">>> Raw IMU ON");
            }
          }
        }
        // trim <roll> <pitch>: 기체 수평 보정 (±10°)
        else if (strncmp(buf, "trim", 4) == 0 &&
                 (buf[4] == ' ' || buf[4] == '\t')) {
          char *save = nullptr;
          (void)strtok_r(buf, " \t", &save);          // "trim"
          char *arg0 = strtok_r(nullptr, " \t", &save);
          char *arg1 = strtok_r(nullptr, " \t", &save);
          if (arg0 != nullptr && arg1 != nullptr &&
              strtok_r(nullptr, " \t", &save) == nullptr) {
            float r, p;
            if (parseFloatStrict(arg0, r) && parseFloatStrict(arg1, p)) {
              if (configChangeAllowed("Trim")) {
                trim_roll  = constrain(r, -TRIM_MAX_DEG, TRIM_MAX_DEG);
                trim_pitch = constrain(p, -TRIM_MAX_DEG, TRIM_MAX_DEG);
                Serial.printf(">>> Trim R:%.2f P:%.2f\n", trim_roll, trim_pitch);
              }
            }
          }
        }
        // gains: 현재 PID 게인 12개를 한 줄로 회신
        else if (strcmp(buf, "gains") == 0) {
          if (connectionEstablished) {
            udp.beginPacket(laptopIP, laptopPort);
            udp.printf("GAINS,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                       Kp_Angle_Roll, Kp_Angle_Pitch, Kp_Angle_Yaw,
                       Kp_Rate_Roll, Kp_Rate_Pitch, Kp_Rate_Yaw,
                       Ki_Rate_Roll, Ki_Rate_Pitch, Ki_Rate_Yaw,
                       Kd_Rate_Roll, Kd_Rate_Pitch, Kd_Rate_Yaw);
            udp.endPacket();
          }
        }
        else {
          handleGainCommand(buf);   // 위 어느 것도 아니면 게인 명령으로 해석 (모르는 접두어는 그냥 무시됨)
        }
      }
    }

    uint32_t now = millis();
    if (mag_enabled || mag_calibrating) sampleMagnetometer(now);
    pollMspSensor(now);
    bool sentImuPacket = false;
    // ZIMU: 50개가 모였거나, 하나라도 있고 50ms가 지났으면 전송 / ZCAL: 1초마다
    if (now - lastRawCheck >= 10U) {
      lastRawCheck = now;
      bool sentRawPacket = false;
      if (__atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE)
          && connectionEstablished) {
        const uint32_t queued = imuRawRingCount(imuRawRing);
        if (queued >= IMU_RAW_BATCH_MAX
            || (queued > 0 && now - rawLastSendMs >= 50U)) {
          sentImuPacket = sendImuRawBatch();
          sentRawPacket = sentImuPacket;
          if (sentImuPacket) rawLastSendMs = now;
        }
        // 한 udp_task 반복에서는 ZIMU 또는 ZCAL 중 최대 하나만 추가 전송한다.
        if (!sentRawPacket && now - rawLastCalMs >= 1000U) {
          sentImuPacket = sendImuCalibration();
          if (sentImuPacket) rawLastCalMs = now;
        }
      }
    }
    // 새 바이너리 경로는 한 반복에 최대 한 데이터그램만 보낸다. 같은 시각의
    // 기존 ASCII 텔레메트리는 다음 반복으로 미뤄 20Hz 주기를 유지한다.
    if (!sentImuPacket && now - lastSend >= 50) {
      lastSend = now;
#if WIFI_LATENCY_DEBUG
      uint32_t sendStartUs = micros();
#endif
      sendTelemetry();
#if WIFI_LATENCY_DEBUG
      uint32_t sendDurUs = micros() - sendStartUs;
      if (sendDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
        recordLatEvent(udpLatRing, millis(), sendDurUs, LAT_UDP_SEND);
      }
#endif
    }
    vTaskDelay(1);   // 1 tick 양보 → 같은 코어의 WiFi 스택/idle 태스크가 돌 수 있게
  }
}

// ==========================================================
// 10. setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SPI_CS1, OUTPUT); pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH); digitalWrite(SPI_CS2, HIGH);   // float CS 로 인한 버스 경합 방지
  delay(100);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);   // 드론이 AP가 된다
  WiFi.setSleep(false);                    // 절전 끔 → 패킷 지연 감소
  WiFi.setTxPower(WIFI_POWER_19_5dBm);     // 최대 송신 출력
  udp.begin(UDP_PORT);

  // 3901-L0X 수신 전용 UART. TX 핀 -1 로 두어 핀 하나만 점유한다.
  Serial1.begin(MSP_SENSOR_BAUD, SERIAL_8N1, MSP_SENSOR_RX_PIN, -1);

  SPI.begin(12, 13, 11, -1);   // SCK, MISO, MOSI; CS는 각 IMU 객체가 관리

  // 모터 핀 4개에 400Hz/14비트 PWM 연결 → 바로 정지 신호(1000µs) 출력 (ESC가 시동 신호를 받아 대기 상태로)
  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) while (1) { Serial.println("[FAULT] ESC attach FAIL"); delay(1000); }
  stopMotors();

  if (IMU1.begin() < 0) while (1) { Serial.println("[FAULT] IMU1 FAIL"); delay(1000); } 
  if (IMU2.begin() < 0) while (1) { Serial.println("[FAULT] IMU2 FAIL"); delay(1000); }
  // IMU 설정: 출력 1600Hz, 가속도 ±16g, 자이로 ±2000dps (GYRO_SCALE/ACCEL_SCALE과 짝) + 하드웨어 LPF
  // 실패 코드를 |=로 모아 하나라도 실패하면 0이 아니게 된다
  // IMUx.begin(): starts communication with the ICM42670
  // 초기화 성공 -> return 0, 실패 -> return < 0
  int sensorStartStatus = 0;
  sensorStartStatus |= IMU1.startAccel(1600, 16);
  sensorStartStatus |= IMU1.startGyro(1600, 2000);
  sensorStartStatus |= IMU2.startAccel(1600, 16);
  sensorStartStatus |= IMU2.startGyro(1600, 2000);
  sensorStartStatus |= IMU1.setLowPassFilters();
  sensorStartStatus |= IMU2.setLowPassFilters();
  if (sensorStartStatus != 0) {
    while (1) { Serial.printf("[FAULT] IMU START FAIL (%d)\n", sensorStartStatus); delay(1000); }
  }
  delay(500);

  calibration_ok = calibrate_bias();   // 약 4초 이상 (IMU 2개 × 2초). 이때 기체를 움직이면 안 된다

  // pid_task 전용 태스크 워치독. idle task는 감시하지 않고, timeout 시
  // panic 재부팅으로 마지막 PWM 고정 상태에서 벗어난다.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = PID_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK &&
      esp_task_wdt_init(&wdt_config) != ESP_OK) {
    while (1) { Serial.println("[FAULT] TASK WDT INIT FAIL"); delay(1000); }
  }

  // FreeRTOS 태스크를 코어에 고정해 생성: (함수, 이름, 스택 4096B, 인자, 우선순위, 핸들, 코어 번호)
  //  PID = 우선순위 2, Core 1 / UDP = 우선순위 1, Core 0 (WiFi 스택도 Core 0에서 돈다)
  BaseType_t pidTaskOk = xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 2, NULL, 1);
  BaseType_t udpTaskOk = xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 1, NULL, 0);
  if (pidTaskOk != pdPASS || udpTaskOk != pdPASS) {
    safety_lock = true;
    while (1) { Serial.println("[FAULT] TASK CREATE FAIL"); delay(1000); }
  }

  Serial.println("DUAL_IMU_CASCADE READY");
}

// Arduino 기본 loop(): 실제 일은 위 두 태스크가 하고, 여기서는 지연 디버그 출력만 (디버그 꺼지면 1초씩 잠만 잔다)
void loop() {
#if WIFI_LATENCY_DEBUG
  drainLatRing(udpLatRing);
  drainLatRing(pidLatRing);

  uint32_t nowMs = millis();
  if (nowMs - latLastSummaryMs >= 2000U) {
    latLastSummaryMs = nowMs;
    uint32_t dropped = __atomic_load_n(&udpLatRing.dropped, __ATOMIC_RELAXED)
                     + __atomic_load_n(&pidLatRing.dropped, __ATOMIC_RELAXED);
    Serial.printf("[LATSUM] up=%lu udpMaxUs=%lu pidMaxUs=%lu "
                  "udpLoopOver30=%lu udpLoopOver100=%lu udpLoopOver300=%lu "
                  "pidOver5=%lu dropped=%lu\n",
                  (unsigned long)nowMs,
                  (unsigned long)latUdpMaxUs, (unsigned long)latPidMaxUs,
                  (unsigned long)latUdpLoopOver30,
                  (unsigned long)latUdpLoopOver100,
                  (unsigned long)latUdpLoopOver300,
                  (unsigned long)latPidOver5, (unsigned long)dropped);
  }
  delay(100);
#else
  // UDP 객체는 udp_task 한 곳에서만 접근해 cross-core race를 피한다.
  delay(1000);
#endif
}
