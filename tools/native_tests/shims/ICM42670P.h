#pragma once

#include <cstdint>

#include "SPI.h"

struct inv_imu_sensor_event_t {
  int16_t gyro[3] = {0, 0, 0};
  int16_t accel[3] = {0, 0, 0};
};

// 펌웨어의 ICM42670WithLPF 서브클래스가 직접 접근하는 저수준 드라이버 인터페이스.
// 호스트 시험은 IMU 를 건드리지 않으므로 컴파일만 되면 되는 동작 없는 stub 이다.
// 실제 라이브러리에서 `icm_driver` 는 protected 다. 여기서도 protected 로 두어
// 타깃에서처럼 서브클래스가 접근하게 한다.
struct inv_imu_device_t {};

enum {
  GYRO_CONFIG1_GYRO_FILT_BW_121 = 0,
  ACCEL_CONFIG1_ACCEL_FILT_BW_25 = 0,
};

inline int inv_imu_set_gyro_ln_bw(inv_imu_device_t *, int) { return 0; }
inline int inv_imu_set_accel_ln_bw(inv_imu_device_t *, int) { return 0; }

class ICM42670 {
public:
  ICM42670(SPIClass &, int) {}

  int begin() { return 0; }
  int startAccel(int, int) { return 0; }
  int startGyro(int, int) { return 0; }

  int getDataFromRegisters(inv_imu_sensor_event_t &event) {
    // keep_event_on_error 는 라이브러리의 첫 읽기(INT_STATUS) 실패를 흉내낸다: 호출자의
    // event 를 건드리지 않고 오류만 돌려준다.
    if (keep_event_on_error && read_status != 0) return read_status;
    event = next_event;
    return read_status;
  }

  inv_imu_sensor_event_t next_event = {};
  int read_status = 0;
  bool keep_event_on_error = false;

protected:
  inv_imu_device_t icm_driver;
};
