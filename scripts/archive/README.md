# 보관된 PC 도구

이 스크립트들은 현행 조종·수신·분석 도구(`drone_control.py`·`drone_video.py`·`drone_logs.py`)가
아니다. 폐기된 실험이나 전용 진단 펌웨어, 이전 벤치 절차를 위한 것이며 현행 UDP
텔레메트리/제어 워크플로의 일부가 아니다. 현행 Python 검증은 벤치 스크립트의
`trim 0 0`·종료 시 `stop` 동작만 시험하고 나머지는 포함하지 않는다.
모두 저장소 루트에서 `python scripts/archive/<이름>.py`로 실행한다. 공유 코드는 `scripts/ground/`에서 가져온다.

## 자동 벤치 스크립트 (실행 범위)

`bench_sign_test.py`·`bench_yaw_test.py`·`bench_thrust_ramp.py`는 이전 벤치의
자동화 도구다. 세 스크립트 모두 **WiFi 전환, `trim 0 0`, `start`, 주기적 RC,
종료 시 `stop`**을 실행한다. 출력 경로와 복귀 SSID가 코드에 고정되어 있고
WiFi 복귀 성공을 충분히 확인하지 않은 채 완료를 출력한다. 새 환경에서
그대로 실행하는 첫 진입점으로 사용하지 말고
[현행 벤치 절차](../../docs/power_on_bench_procedure.md)와 설정을 먼저 대조한다.

| 스크립트 | 용도 | 전제 |
|---|---|---|
| `bench_sign_test.py` | 텔레메트리 roll/pitch 부호 확인 | 모터 전원 차단·프로펠러 제거. 이 전제를 코드가 물리적으로 확인하지 않는다 |
| `bench_yaw_test.py` | yaw 방향 부호 확인 | 모터 전원 차단·프로펠러 제거. 위와 같음 |
| `bench_thrust_ramp.py` | 프로펠러가 회전하는 고정 시험(추력 램프) | EOF에서도 확인 입력을 통과하고, 종료 램프는 고정 `1200 → 1100 → 1000`이어서 지정한 `cap_us`가 1200보다 작아도 그 값을 넘을 수 있다. 20Hz 모터 명령 분산에 근거한 “진동 없음”·다음 시험 권고 출력은 기계 진동·실제 추력·비행 안전의 검증으로 취급하지 않는다. 이 제한을 해결하기 전에는 초보자 실행 경로에서 제외한다 |

## 전용 펌웨어용 도구

| 스크립트 | 용도 | 전제 |
|---|---|---|
| `motor_serial.py` | 모터 식별용 시리얼 조작 | 전용 `motor_id_single` 펌웨어, 프로펠러 제거·PSU 전류 제한, 지정 시리얼 포트 확인. `pyserial` 필요. 모터 회전은 USB 통신만으로 검증되지 않는다 |
| `receive_dual_imu_debug.py` | dual-IMU 루프 진단 수신 | 전용 `icm42670_dual_loop_debug` 펌웨어와 `DUAL_IMU_DEBUG` AP |

## 폐기된 GPS/TCP 실험

`receive_gps_udp_legacy.py`·`test_tcp_legacy.py`는 폐기된 GPS/TCP 실험을 대상으로 하며
역사 목적으로만 보존된다. `test_tcp_legacy.py`는 한쪽 엔드포인트가 수신 서버 역할을 하는
상태에서 드론→PC TCP 패킷 지연과 무결성을 확인하려던 도구였다. 현행 UDP 4210
워크플로 이전 것이며 지원되는 프로토콜 테스트가 아니다.
