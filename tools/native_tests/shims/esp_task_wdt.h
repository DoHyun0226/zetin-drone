#pragma once

// ESP-IDF 태스크 watchdog 의 호스트 시험용 stub. 비행 펌웨어는 pid_task 를
// TWDT 에 등록하고 루프마다 리셋한다. 시험 대상인 순수 제어 수학 로직과는
// 무관하므로, 컴파일만 통과시키는 아무 동작 없는 no-op 이다. zetin_fc 가 쓰는
// 실제 esp_task_wdt.h 의 인터페이스를 그대로 따른다.

#include <cstdint>

typedef int esp_err_t;

#ifndef ESP_OK
#define ESP_OK 0
#endif
#ifndef ESP_FAIL
#define ESP_FAIL (-1)
#endif

typedef struct {
  uint32_t timeout_ms;
  uint32_t idle_core_mask;
  bool trigger_panic;
} esp_task_wdt_config_t;

// NULL 은 호출한 태스크를 등록한다. 호스트 시험은 pid_task 를 부르지 않으므로
// no-op 으로 충분하다.
inline esp_err_t esp_task_wdt_add(void *) { return ESP_OK; }
inline esp_err_t esp_task_wdt_reset(void) { return ESP_OK; }
inline esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t *) { return ESP_OK; }
inline esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t *) { return ESP_OK; }
