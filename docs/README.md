# 문서

## 목적에 따라 시작하기

| 알고 싶은 것 | 시작 문서 |
|---|---|
| 어디까지 했고 다음에 무엇을 하나 | [현재 상태와 다음 작업](current_status.md) |
| 처음 참여하거나 작업을 인수인계한다 | [작업 순서](workflow.md) → [프로젝트 개요](project_overview.md) |
| 어떤 기체·코드로 시험하나 | [하드웨어 구성](hardware_configuration.md) → [펌웨어 카탈로그](firmware_catalog.md) |
| 실제로 실행·조종·기록한다 | [벤치 절차](power_on_bench_procedure.md) → [지상국](../scripts/README.md) → [로그](../logs/README.md) |
| 명령이나 데이터 해석을 바꾼다 | [UDP 프로토콜](udp_protocol.md) |

## 유지 문서

- [현재 상태와 다음 작업](current_status.md)
- [작업 순서와 인수인계](workflow.md)
- [하드웨어 구성과 확인할 항목](hardware_configuration.md)
- [프로젝트 개요](project_overview.md)
- [펌웨어 카탈로그](firmware_catalog.md)
- [UDP 프로토콜](udp_protocol.md)
- [전원 인가 벤치 절차 (첫 호버 전 안전 관문)](power_on_bench_procedure.md)
- [지상국 링크 요구사항과 RC 타임아웃 진단](ground_station_link.md)
- 단일 PID와 캐스케이드 PID 비교 — [PDF](cascade_vs_single_pid.pdf) · [Typst 원본](cascade_vs_single_pid.typ)
- [BMM350 yaw 융합 벤치 시험](bmm350_yaw_bench_test.md)
- [자동착륙 착지 감지 연구](failsafe_land_research.md)
- [노션 문서 컨벤션](notion_doc_convention.md)
- [저장소 마이그레이션 맵](migration_map.md)

## 설계·계획 기록

날짜가 박힌 기록이다. 사후 수정하지 않고, 틀린 것으로 밝혀지면 정정을 병기한다.

- [`superpowers/specs/`](superpowers/specs/) — 기능별 설계 문서 (자동착륙, yaw 각속도
  명령, 자기계 캘리브레이션, Tier 1 관측성)
- [`superpowers/plans/`](superpowers/plans/) — 그 설계의 구현 계획
- [저장소 정리 설계](design/2026-07-13-repository-cleanup-design.md) ·
  [구현 계획](plans/2026-07-13-repository-cleanup.md)

## 과거 문서

아래 및 날짜가 붙은 계획의 완료 체크·명령·상수는 작성 당시 상태다.
현재 실행에는 위 유지 문서를 사용한다. 특히 프로브 착지컷·옛 스키마·
분리 전 발표자료 경로를 현행으로 옮기지 않는다.

- [2026-05-14 듀얼 IMU PID 설계](history/2026-05-14-dual-imu-pid-design.md)
- [2026-05-14 듀얼 IMU PID 구현 계획](history/2026-05-14-dual-imu-pid-implementation-plan.md)
- [2026-09-07 코드·문서·노션 정합성 점검](reports/2026-09-07-documentation-audit.md)

## 보관 문서

- [PID/DShot 발표용 코드 조각](archive/pid_dshot_presentation_snippet.c)
- [과거 커밋 메시지 스냅샷](archive/commit_message_snapshot.txt)

## 발표자료는 여기 없다

2026-08-24에 별도 저장소로 분리했다 —
[lightminn/zetin-drone-presentations](https://github.com/lightminn/zetin-drone-presentations),
공개본은 <https://lightminn.github.io/zetin-drone-presentations/>다. 옛 주소
`lightminn.github.io/zetin-drone/`와 `/10min/`은 리다이렉트로 살려 두었다.
Oracle 웹 호스팅 가이드와 모바일 실습 랩도 그쪽으로 갔다.
