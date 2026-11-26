// zetin_fc/fc/failsafe_state.h — 무장·해제·resume 요청 우편함(safetyRequestMux)과 failsafe 단계·프로브 상태. 쓰기는 Core 1.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
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
