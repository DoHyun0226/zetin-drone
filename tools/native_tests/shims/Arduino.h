#pragma once

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <sstream>
#include <deque>
#include <string>
#include <type_traits>
#include <unordered_map>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

using std::abs;
using std::isfinite;
using std::max;
using std::min;

template <typename T>
constexpr T constrain(const T &value, const T &low, const T &high) {
  return value < low ? low : (value > high ? high : value);
}

constexpr int OUTPUT = 0x03;
constexpr int HIGH = 0x01;
constexpr int LOW = 0x00;

using TickType_t = uint32_t;
using BaseType_t = int;
using TaskFunction_t = void (*)(void *);
struct portMUX_TYPE {};

constexpr BaseType_t pdPASS = 1;
inline constexpr portMUX_TYPE portMUX_INITIALIZER_UNLOCKED = {};

namespace arduino_fake {

inline std::string serial_output;
// Serial1 은 3901-L0X 수신 전용이다. 테스트가 여기에 바이트를 넣어두면
// 펌웨어의 pollMspSensor 가 읽어간다.
inline std::deque<uint8_t> serial1_rx;
inline uint32_t millis_value = 0;
inline uint32_t micros_value = 0;
inline uint32_t us_per_tick = 0;
inline TickType_t tick_count = 0;
inline std::unordered_map<int, uint32_t> ledc_duty_by_pin;
inline std::unordered_map<int, bool> ledc_attached_by_pin;
inline std::unordered_map<int, int> digital_level_by_pin;
inline std::unordered_map<int, int> pin_mode_by_pin;
inline bool stop_on_task_delay = false;
inline std::function<void(uint32_t)> pre_tick_hook = nullptr;
inline uint32_t tick_index = 0;
inline uint32_t tick_limit = 0;

struct TaskDelayExit {};

// tools/native_tests/diff_harness 의 lockstep 스케줄러 훅. 기본은 모두 null 이고, 그러면
// 다른 native 시험이 기대하는 동작 그대로다. sleep_us: 부른 태스크를 가상 시간으로 그만큼
// 멈춘다(delay, delayMicroseconds, vTaskDelay). sleep_until_tick: 그 1 kHz tick 까지 멈춘다
// (vTaskDelayUntil). task_created: xTaskCreatePinnedToCore 가 불렸다. ledc_write: 모든
// ledcWrite 를 부른 순서대로.
inline void (*lockstep_sleep_us)(uint64_t) = nullptr;
inline void (*lockstep_sleep_until_tick)(TickType_t) = nullptr;
inline void (*lockstep_task_created)(TaskFunction_t, const char *) = nullptr;
inline void (*lockstep_ledc_write)(int, uint32_t) = nullptr;

inline void reset() {
  serial_output.clear();
  serial1_rx.clear();
  millis_value = 0;
  micros_value = 0;
  us_per_tick = 0;
  tick_count = 0;
  ledc_duty_by_pin.clear();
  ledc_attached_by_pin.clear();
  digital_level_by_pin.clear();
  pin_mode_by_pin.clear();
  pre_tick_hook = nullptr;
  tick_index = 0;
  tick_limit = 0;
}

inline void appendFormatted(std::string &destination, const char *format,
                            va_list arguments) {
  va_list measure_arguments;
  va_copy(measure_arguments, arguments);
  const int length = std::vsnprintf(nullptr, 0, format, measure_arguments);
  va_end(measure_arguments);
  if (length <= 0) return;

  std::string formatted(static_cast<std::size_t>(length), '\0');
  std::vsnprintf(formatted.data(), formatted.size() + 1, format, arguments);
  destination += formatted;
}

}  // namespace arduino_fake

class FakeSerial {
public:
  void begin(unsigned long) {}

  template <typename T>
  void print(const T &value) {
    std::ostringstream stream;
    stream << value;
    arduino_fake::serial_output += stream.str();
  }

  void println() { arduino_fake::serial_output.push_back('\n'); }

  template <typename T>
  void println(const T &value) {
    print(value);
    println();
  }

  int printf(const char *format, ...) {
    const std::size_t before = arduino_fake::serial_output.size();
    va_list arguments;
    va_start(arguments, format);
    arduino_fake::appendFormatted(
        arduino_fake::serial_output, format, arguments);
    va_end(arguments);
    return static_cast<int>(arduino_fake::serial_output.size() - before);
  }
};

inline FakeSerial Serial;

#define SERIAL_8N1 0x800001c

// 수신 전용 UART shim. begin() 은 아무것도 하지 않고, available()/read() 가
// arduino_fake::serial1_rx 를 소비한다.
class FakeSerial1 {
 public:
  void begin(unsigned long, int = 0, int = -1, int = -1) {}
  int available() { return (int)arduino_fake::serial1_rx.size(); }
  int read() {
    if (arduino_fake::serial1_rx.empty()) return -1;
    const uint8_t value = arduino_fake::serial1_rx.front();
    arduino_fake::serial1_rx.pop_front();
    return (int)value;
  }
};

inline FakeSerial1 Serial1;

inline uint32_t millis() { return arduino_fake::millis_value; }
inline uint32_t micros() { return arduino_fake::micros_value; }
inline void delay(uint32_t ms) {
  if (arduino_fake::lockstep_sleep_us) arduino_fake::lockstep_sleep_us(1000ULL * ms);
}
inline void delayMicroseconds(uint32_t us) {
  if (arduino_fake::lockstep_sleep_us) arduino_fake::lockstep_sleep_us(us);
}

inline void pinMode(int pin, int mode) {
  arduino_fake::pin_mode_by_pin[pin] = mode;
}

inline void digitalWrite(int pin, int level) {
  arduino_fake::digital_level_by_pin[pin] = level;
}

inline bool ledcAttach(int pin, uint32_t, uint8_t) {
  arduino_fake::ledc_attached_by_pin[pin] = true;
  return true;
}

inline bool ledcWrite(int pin, uint32_t duty) {
  arduino_fake::ledc_duty_by_pin[pin] = duty;
  if (arduino_fake::lockstep_ledc_write) arduino_fake::lockstep_ledc_write(pin, duty);
  return true;
}

inline TickType_t pdMS_TO_TICKS(uint32_t milliseconds) {
  return milliseconds;
}

inline TickType_t xTaskGetTickCount() {
  return arduino_fake::tick_count;
}

inline void vTaskDelayUntil(TickType_t *wake, TickType_t period) {
  using namespace arduino_fake;
  if (lockstep_sleep_until_tick) {
    *wake += period;
    lockstep_sleep_until_tick(*wake);
    return;
  }
  if (pre_tick_hook) pre_tick_hook(tick_index);
  const uint64_t elapsed_us =
      static_cast<uint64_t>(micros_value % 1000U) + us_per_tick;
  micros_value += us_per_tick;
  millis_value += static_cast<uint32_t>(elapsed_us / 1000U);
  tick_index++;
  if (tick_limit && tick_index > tick_limit) throw TaskDelayExit{};
}
inline void vTaskDelay(TickType_t ticks) {
  if (arduino_fake::lockstep_sleep_us) {
    arduino_fake::lockstep_sleep_us(1000ULL * ticks);
    return;
  }
  if (arduino_fake::stop_on_task_delay) throw arduino_fake::TaskDelayExit{};
}
inline void portENTER_CRITICAL(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL(portMUX_TYPE *) {}

inline BaseType_t xTaskCreatePinnedToCore(
    TaskFunction_t task, const char *name, uint32_t, void *, uint32_t, void **,
    BaseType_t) {
  if (arduino_fake::lockstep_task_created) arduino_fake::lockstep_task_created(task, name);
  return pdPASS;
}
