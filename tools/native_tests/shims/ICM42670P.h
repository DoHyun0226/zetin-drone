#pragma once

#include <cstdint>

#include "SPI.h"

struct inv_imu_sensor_event_t {
  int16_t gyro[3] = {0, 0, 0};
  int16_t accel[3] = {0, 0, 0};
};

// Low-level driver surface the firmware's ICM42670WithLPF subclass reaches into.
// The host tests never touch the IMU, so these are inert stubs that only need to
// compile. `icm_driver` is protected in the real library; keep it protected here
// so the subclass can access it just as it does on target.
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
