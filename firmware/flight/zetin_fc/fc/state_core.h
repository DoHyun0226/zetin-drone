// zetin_fc/fc/state_core.h — 4. 시스템 변수: 네트워크, IMU·자기계 객체, 안전 요청, failsafe, MSP 센서 상태.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ==========================================================
// 4. 시스템 변수
// ==========================================================
WiFiUDP   udp;
char      packetBuffer[256];
IPAddress laptopIP;
int       laptopPort            = 0;
volatile bool connectionEstablished = false;

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

ICM42670WithLPF IMU1(SPI, SPI_CS1);
ICM42670WithLPF IMU2(SPI, SPI_CS2);
DFRobot_BMM350_I2C bmm(&Wire, 0x14);

volatile bool  safety_lock  = true;
// 런타임 safety_lock 전이는 Core 1(pid_task)만 수행한다. Core 0은 요청만
// 게시하며, 두 요청이 함께 대기하면 disarm이 arm보다 우선한다.
volatile bool  safety_disarm_requested = false;
volatile bool  safety_arm_requested = false;
volatile bool  failsafe_resume_requested = false;
volatile bool  hover_reset_requested = false;
portMUX_TYPE safetyRequestMux = portMUX_INITIALIZER_UNLOCKED;
// 런타임 쓰기는 pid_task만 담당한다. Core 0은 텔레메트리/명령 처리에서 읽기만 한다.
volatile uint8_t fs_phase = FS_NONE;   // 텔레메트리 Failsafe_Phase
volatile uint8_t fs_probe_state = FS_PROBE_WAIT;
volatile uint8_t fs_probe_no_response = 0;
volatile float fs_probe_response_g = 0.0f;
// 한 비행에서 최초 자동착륙에 들어간 시각. resume은 지우지 않으며 새 start가
// 실제로 Core 1에서 arm 전이될 때만 초기화한다.
volatile uint32_t fs_first_enter_ms = 0;
volatile bool fs_first_enter_valid = false;
// 3901-L0X 상태. udp_task 만 쓰고 sendTelemetry 가 읽는다(같은 코어).
MspSensorParser mspSensor;
volatile int32_t msp_range_mm = 0;
volatile int     msp_range_quality = -1;   // -1 = 신선한 프레임 없음
volatile int32_t msp_flow_x = 0, msp_flow_y = 0;
volatile int     msp_flow_quality = -1;
uint32_t msp_last_range_ms = 0;
uint32_t msp_last_flow_ms = 0;
uint16_t msp_reported_unknown = 0;         // 같은 ID 를 반복해 찍지 않기 위해

