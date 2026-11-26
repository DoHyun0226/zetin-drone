// zetin_fc/fc/failsafe_state.h — 무장·해제·resume 요청 우편함(safetyRequestMux)과 failsafe 단계·프로브 상태. 쓰기는 Core 1.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// true = 시동 꺼짐(모터 정지). 부팅 시 잠금 상태로 시작
volatile bool  safety_lock  = true;
// [safety 요청 구조] udp_task(Core 0)는 "start/stop 해 달라"는 요청 플래그만 세우고,
//  실제 safety_lock 변경은 pid_task(Core 1)가 매 tick applyPendingSafetyRequest()에서 한다
//  → 모터를 쓰는 쪽과 잠금을 바꾸는 쪽이 같은 코어라 한 tick 안에서 상태가 엇갈리지 않는다

// 런타임 safety_lock 전이는 Core 1(pid_task)만 수행한다. Core 0은 요청만
// 게시하며, 두 요청이 함께 대기하면 disarm이 arm보다 우선한다.
// "stop" 요청
volatile bool  safety_disarm_requested = false;
// "start" 요청
volatile bool  safety_arm_requested = false;
// "resume" 요청 (자동착륙 중 조종 복귀)
volatile bool  failsafe_resume_requested = false;
// start 시 호버 추정치 초기화 요청
volatile bool  hover_reset_requested = false;
// 위 요청 플래그들을 보호하는 스핀락
portMUX_TYPE safetyRequestMux = portMUX_INITIALIZER_UNLOCKED;
// 런타임 쓰기는 pid_task만 담당한다. Core 0은 텔레메트리/명령 처리에서 읽기만 한다.
volatile uint8_t fs_phase = FS_NONE;   // 텔레메트리 Failsafe_Phase
// 착지 프로브 상태 (텔레메트리 41)
volatile uint8_t fs_probe_state = FS_PROBE_WAIT;
// "반응 없음" 연속 횟수 (텔레메트리 42)
volatile uint8_t fs_probe_no_response = 0;
// 마지막 프로브 반응 크기 g (텔레메트리 43)
volatile float fs_probe_response_g = 0.0f;
// 한 비행에서 최초 자동착륙에 들어간 시각. resume은 지우지 않으며 새 start가
// 실제로 Core 1에서 arm 전이될 때만 초기화한다.
volatile uint32_t fs_first_enter_ms = 0;
volatile bool fs_first_enter_valid = false;
