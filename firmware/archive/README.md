# 보관된 펌웨어

이 디렉터리 아래의 모든 코드는 역사 보존 목적으로만 유지된다. 지원되는
ESP32-S3 + ICM42670 + PWM 빌드 경로에 속하지 않으며, 컴파일 검증되지
않았고, 누락된 의존성·구식 핀 배치·위험한 동작을 포함할 수 있다.

보관된 모터 제어 코드는 전체를 검토하고 프로펠러를 제거하기 전에는 절대
플래시하지 않는다. 현행 펌웨어는 [`../README.md`](../README.md)에서 연결된다.

`legacy_flight/` 의 `dual_imu_cascade_pwm`(2026-11-13 까지 주력)과 `dual_imu_flix_quat_pwm`(보류
후보)은 비행 펌웨어를 [`zetin_fc`](../flight/zetin_fc/README.md) 하나로 모으면서 옮겼다.
`zetin_fc` 는 `dual_imu_cascade_pwm` 의 기능을 모두 포함한다.
