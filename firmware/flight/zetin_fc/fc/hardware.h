// zetin_fc/fc/hardware.h — UDP 소켓·명령 버퍼, IMU 두 개(ICM42670 + 하드웨어 LPF)와 BMM350 객체.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ==========================================================
// 4. 시스템 변수
// ==========================================================
// [코어 분담] Core 1 = pid_task(1kHz 제어), Core 0 = udp_task(명령 수신/텔레메트리/자기계/광류)
//  두 코어가 같이 쓰는 변수는 volatile 또는 portMUX(임계구역)로 보호한다
// UDP 소켓 (udp_task만 사용)
WiFiUDP   udp;
// 받은 명령 문자열 버퍼
char      packetBuffer[256];
// 마지막으로 명령을 보낸 노트북 주소 → 텔레메트리 목적지
IPAddress laptopIP;
int       laptopPort            = 0;
// 노트북에서 패킷을 한 번이라도 받았는가 (그 전엔 보낼 곳이 없음)
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

// 같은 SPI 버스, CS 핀으로 구분
ICM42670WithLPF IMU1(SPI, SPI_CS1);
ICM42670WithLPF IMU2(SPI, SPI_CS2);
// BMM350 자기계, I2C 주소 0x14
DFRobot_BMM350_I2C bmm(&Wire, 0x14);
