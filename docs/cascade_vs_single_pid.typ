// README에서 링크하는 유지 문서 원본. PDF는 이 파일에서 재생성한다.
#set document(
  title: "단일 PID와 캐스케이드 PID 비교",
  author: "ZETIN Drone",
)
#set page(paper: "a4", margin: (x: 2.2cm, y: 2.4cm), numbering: "1")
#set text(lang: "ko", font: ("Libertinus Serif", "NanumMyeongjo"), size: 10.5pt)
#set par(
  justify: true,
  leading: 0.72em,
  justification-limits: (
    tracking: (min: -0.012em, max: 0.012em),
    spacing: (min: 75%, max: 120%),
  ),
)
#set heading(numbering: "1.")
#show heading: set text(font: ("Libertinus Serif", "NanumBarunGothic"))
#show heading.where(level: 1): it => block(above: 1.6em, below: 0.9em)[
  #text(size: 1.15em, fill: rgb("#1a3a5c"))[#it]
]
#show raw: set text(font: ("D2Coding", "DejaVu Sans Mono"), size: 0.88em)
#set table(stroke: (x, y) => (
  top: if y <= 1 { 0.7pt } else { 0pt },
  bottom: 0.7pt,
))
#show table.cell.where(y: 0): strong

#let accent = rgb("#1a3a5c")
#let src(body) = text(size: 0.82em, fill: rgb("#6b6b6b"))[#raw(body)]

#let note(title: none, body) = block(
  width: 100%,
  breakable: false,
  fill: rgb("#f2f5f8"),
  stroke: (left: 2.5pt + accent),
  inset: (x: 12pt, y: 10pt),
  radius: (right: 3pt),
)[
  #if title != none [#text(weight: "bold", fill: accent)[#title] \ ]
  #body
]

#let warn(title: none, body) = block(
  width: 100%,
  breakable: false,
  fill: rgb("#fdf4ec"),
  stroke: (left: 2.5pt + rgb("#b5651d")),
  inset: (x: 12pt, y: 10pt),
  radius: (right: 3pt),
)[
  #if title != none [#text(weight: "bold", fill: rgb("#8a4a12"))[#title] \ ]
  #body
]

#let bx(body, fill: white) = rect(
  inset: (x: 8pt, y: 7pt),
  radius: 3pt,
  stroke: 0.6pt + rgb("#556"),
  fill: fill,
  align(center, text(size: 0.85em)[#body]),
)
#let ar = box(inset: (x: 4pt), baseline: -0.15em)[#text(
  fill: accent,
  size: 1.1em,
)[→]]

#title[단일 PID와 캐스케이드 PID 비교]

#align(center)[
  #text(size: 0.9em, fill: rgb("#666"))[
    현재 `firmware/flight/dual_imu_cascade_pwm` ·
    레거시 `firmware/archive/legacy_flight/dual_imu_pid_pwm` \
    main 소스 기준 · 2026-09-07 정비
  ]
]

#v(0.5em)

#note(title: "먼저 알아둘 점")[
  캐스케이드는 *목표 각속도를 별도 신호로 두어* 명령 제한, 각속도 추종,
  단계별 튜닝을 명시적으로 다룬다. 레거시에도 자이로 감쇠와 적분 제한이 있다.
  구조와 기본 게인의 차이만으로 실제 비행이 더 빠르거나 안정적이라고
  판정할 수는 없다. 아래 수치 비교는 *roll·pitch의 소스 기본값*이며,
  yaw 전환 동작은 별도로 설명한다.
]

= 두 구조: 각도에서 모터까지

*단일 PID*는 각도 오차의 P·I 항과 측정 자이로의 감쇠항을 합쳐 모터 차동
명령을 만든다. 따라서 모든 항이 각도 추정을 거치는 구조는 아니다.

#align(center)[
  #stack(
    dir: ltr,
    spacing: 0pt,
    bx[$theta_"cmd"$],
    ar,
    bx(fill: rgb("#fdf3e7"))[각도 PID \ #text(0.9em)[2.5 / 0.005 / 1.2]],
    ar,
    bx[믹서],
    ar,
    bx[모터],
  )
]

*캐스케이드 PID*는 각도 P 루프가 목표 각속도를 만들고, 내부 PID가 목표와
측정 각속도의 차이를 줄인다. 아래 그림은 roll·pitch의 명령 흐름이다.

#align(center)[
  #stack(
    dir: ltr,
    spacing: 0pt,
    bx[$theta_"cmd"$],
    ar,
    bx(fill: rgb("#eef4ea"))[각도 P \ #text(0.9em)[250 Hz · $K_(p a)$ = 6.0]],
    ar,
    bx(fill: rgb("#fce9e9"))[제한 \ #text(0.9em)[±300 dps]],
    ar,
    bx(
      fill: rgb("#eaf0f7"),
    )[각속도 PID \ #text(0.9em)[1 kHz · 0.5 / 0.05 / 0.015]],
    ar,
    bx[믹서 \ #text(0.9em)[+desat]],
    ar,
    bx[모터],
  )
]

두 펌웨어의 PID 태스크는 명목 1 kHz다. 현재 코드의 외부 각도 루프는
`OUTER_DIV = 4`에 따라 그중 네 번에 한 번 갱신하므로, *모든 제어 신호의
갱신 주기가 같은 것은 아니다.* 실제 주기와 지연은 실행 중 측정해야 한다.

= 현재 구현에서 달라진 점

== 목표 각속도에 제한을 둔다

#src("cascade: Kp_Angle_Roll/Pitch, MAX_TARGET_RATE_RP, targetRateRoll/Pitch")

각도 오차에 6.0을 곱하고 결과를 ±300 deg/s로 제한한다. 제한 전 계산에서
5° 오차는 30 deg/s, 50° 오차는 300 deg/s다. 이 값은 *요구하는 회전 속도*이며
실제 기체가 그 속도로 돌 수 있다는 보장은 아니다.

이 제한은 큰 각도 오차가 내부 루프에 만드는 요구량을 제한한다. 그러나 현재
각속도, I·D 항, 모터의 남은 출력 범위에 따라 믹서는 여전히 포화될 수 있다.
내부 루프의 선형 동작이나 windup 제거를 보장하는 장치는 아니다.

== 적분을 각속도 오차에 건다

#src("cascade: eRoll/Pitch/Yaw, iTermRoll/Pitch/Yaw, I_TERM_MAX_US")

레거시는 각도 오차를 적분한다. roll·pitch 기본값에서 적분 상태 ±15에
$K_i = 0.005$를 곱하므로 최대 출력 기여는 *0.075 µs*다. 현재는 각속도
오차를 적분하면서 게인을 곱해, 출력 기여 자체를 *±50 µs*로 제한한다.
단위와 누적 대상이 달라 두 적분 게인의 숫자만 직접 비교하면 안 된다.

각속도 적분은 지속적인 토크 불균형에 대응할 수 있지만, 외란 제거 속도와
최종 자세 오차는 게인·적분 조건·외부 루프·기체에 달려 있다. 항상 더 빨리
외란을 지운다고 단정할 수 없다. `Ki_Rate_Roll/Pitch` 주석의 호스트 SIL 근거는
기본값 선택 기록이며, 현재 기체의 비행 검증을 대신하지 않는다.

#pagebreak()

== 믹서 포화와 적분 누적을 연결한다

#src("cascade: mixAndDesaturate, mix.scaled, throttle, safety_lock")

믹서는 먼저 네 모터의 공통 출력(collective)을 이동해 자세 차동 명령을
수용하고, 차동 명령의 폭도 허용 범위를 넘으면 같은 비율로 축소한다.
`mix.scaled`는 이 축소를 나타낸다. 이는 PWM 명령의 비율을 다루는 것으로,
실제 추력·토크 비율까지 보장하지 않는다.

현재 제어 루프의 적분 조건은 다음과 같다.

- 스로틀이 *1100 µs 초과*이고 `mix.scaled`가 거짓이면 누적하며, 축마다 ±50 µs로 제한한다.
- 스로틀이 *1100 µs 이하*이면 세 축 적분값을 0으로 초기화한다.
- 스로틀이 1100 µs 초과인데 `mix.scaled`가 참이면 기존 적분값을 유지한다.
  포화 중 쌓인 값을 역산해 줄이는 방식은 아니다.
- 안전 잠금 경로는 적분 상태를 초기화하고 출력을 정지한다.

공통 출력 이동만 일어나면 위 스로틀 조건 아래에서 적분을 계속한다. 레거시에도
적분 클램프와 저스로틀 초기화, 큰 각도 오차에서의 누적 제한이 있었으므로,
anti-windup이 전혀 없었다고 설명해서는 안 된다.

== 내부 추종과 외부 자세 응답을 나눠 관찰한다

목표·측정 각속도를 비교해 내부 추종을 먼저 보고, 이후 외부 각도 게인을
조정할 수 있다. 다만 현재 roll·pitch 목표 각속도는 외부 루프가 계속 만들므로,
*독립적인 rate 스텝 시험에는 별도 시험 구성이 필요하다.* 단계별 분석이 쉬워지는
것이지, ESC 지연·진동·관성의 영향이나 최종 결합 시험이 없어지는 것은 아니다.

현재와 레거시 모두 측정 자이로를 직접 피드백한다. 현재 코드도 자세 추정이
틀리면 외부 루프가 잘못된 목표 각속도를 만들 수 있어, 내부 루프만으로 자세
추정 고장을 견딘다고 볼 수 없다.

== yaw는 각속도 명령 후 정착 조건에서 heading을 잠근다

#src("yaw_command.h: updateYawOuter, yawTargetRateDps")

강제 hold가 없는 일반 동작에서는 명령 절댓값이 *3 deg/s 미만*이어야 스틱
중립으로 본다. 아직 잠기지 않았다면 측정 `bodyGz`의 절댓값도 *10 deg/s 미만*일 때
새 heading 잠금에 진입한다. 잠기기 전에는 목표 heading을 현재 추정 heading으로
매 tick 갱신하고, 진입 시 직전 목표를 유지한다. 따라서 스틱을 놓는 순간의
heading을 무조건 잠그는 동작이 아니다.

중립에서 이미 잠겼다면 각속도가 다시 커져도 잠금을 유지한다. 중립을 벗어나면
rate 모드로 풀리며, `yaw_hold_override`는 이 조건들보다 우선한다. yaw 적분은
rate·hold 모두에서 앞의 스로틀·포화 조건에 따라 동작한다. 이 yaw 인터페이스가
있다고 roll·pitch acro 모드까지 구현된 것은 아니다.

== D 항의 측정 대상이 다르다

#src("legacy: pid_roll/pitch  /  cascade: dRoll/Pitch, lpfD_Roll/Pitch")

레거시의 `-lpf_gx * Kd_Roll`은 측정 각속도 감쇠다. 현재의 D는 측정 자이로를
미분한 각가속도에 40 Hz 저역통과 필터를 적용한다. *둘 다 명령을 직접 미분하지
않으므로*, 명령 스텝의 derivative kick을 피하는 성질은 캐스케이드만의 이점이
아니다. D 필터·센서 잡음·액추에이터 지연을 포함한 안정성은 따로 검증해야 하며,
공진 억제나 안정 여유를 게인 숫자만으로 보장할 수 없다.

#pagebreak()

= 기본 게인으로 비교할 수 있는 범위

아래 표는 roll·pitch 기본값이다. 런타임 명령으로 게인이 바뀔 수 있으며,
실기에서 사용한 값은 해당 실험 기록과 함께 확인해야 한다.

#block(breakable: false, table(
  columns: (auto, 1fr, 1fr),
  align: (left, center, center),
  table.header([], [단일 PID (레거시)], [캐스케이드 (현재)]),

  [각도 루프],
  [$K_p$ 2.5 / $K_i$ 0.005 / $K_d$ 1.2 \ (명목 1 kHz)],
  [$K_(p a)$ = 6.0 \ (P만, 명목 250 Hz)],
  [각속도 루프], [별도 목표 없음], [0.50 / 0.05 / 0.015 \ (명목 1 kHz)],
  table.hline(stroke: 0.4pt),
  [각도 오차 계수 $K_theta$], [2.5 µs/deg], [$6.0 times 0.5 =$ *3.0* µs/deg],
  [각속도 감쇠 계수 $K_omega$], [1.2 µs/(deg/s)], [*0.50* µs/(deg/s)],
  table.hline(stroke: 0.4pt),
  [적분기 최대 기여], [*0.075 µs*], [*50 µs*],
  [목표 각속도 제한], [별도 목표 없음], [±300 deg/s],
  [D 항의 측정 대상], [각속도], [각가속도 + 40 Hz LPF],
))

#align(right)[#src(
  "legacy: Kp/Ki/Kd_Roll/Pitch  /  cascade: Kp_Angle_*, Kp/Ki/Kd_Rate_*",
)]

== P 항 중심의 단순 모델에서는 같은 식으로 쓸 수 있다

이 절에서만 두 제어기의 적분, 현재 내부 루프의 D, 명령·출력 제한,
센서·D 필터, 샘플링·계산·액추에이터 지연을 제외한다. 레거시의 자이로 감쇠는
남긴다. 각도 오차를 $e_theta = theta_"cmd" - theta$, 측정 각속도를 $omega$라 두면

$
  u_"single" = K_p e_theta - K_d omega,
  quad
  u_"cascade" = K_(p r) (K_(p a) e_theta - omega)
$

두 식은 모두 $u = K_theta e_theta - K_omega omega$ 형태다. 여기서 $u$는 축의
PWM 차동 명령(µs)이다. *표의 계수는 실제 기체의 물리적 강성·감쇠비가 아니다.*

같은 운용점에서 PWM 차동 명령과 각가속도 사이의 양의 이득을 $b$로 근사해
$dot.double(theta) = b u$라 놓으면, 이 단순 모델 안에서만

$
  dot.double(theta) + b K_omega dot(theta) + b K_theta theta = b K_theta theta_"cmd",
$
$
  omega_n = sqrt(b K_theta), quad zeta = (b K_omega) / (2 sqrt(b K_theta))
$

이다. *두 경우의 $b$도 같다고 가정하면* 고유진동수 비는
$sqrt(3.0 / 2.5) = 1.10$, 감쇠비 비는 $(0.5 / 1.2) sqrt(2.5 / 3.0) = 0.38$이다.
이 계산으로 실제 감쇠비, 과감쇠 여부, 오버슈트, 체감 반응의 원인을 확정할 수는
없다. 제외한 요소와 기체의 실제 응답을 함께 확인해야 한다.

== 250 Hz / 1 kHz 분리는 갱신 간격의 차이다

`OUTER_DIV = 4`는 목표 각속도를 네 내부 tick마다 갱신하고 그 사이에는 유지한다.
명목 갱신 간격은 4 ms다. 이 사실만으로 노이즈 감소가 설계 목적이었다거나,
지연 영향이 무시 가능하다고 단정할 수 없다. 갱신률을 낮추는 것 자체는
저역통과 필터가 아니며, 폐루프 대역폭·위상 지연·실제 주기를 함께 평가해야 한다.

#note(title: "이 문서를 실험에 적용할 때")[
  코드로 확인한 이점은 목표 각속도 제한, 출력 단위의 적분 관리, 포화 시 누적
  정지, 단계별 응답 관찰이다. 실제 성능 비교에는 사용 게인·스로틀·기체 구성과
  함께 목표/측정 각속도, 자세, 모터 포화, 지연을 기록한다.
  이 문서 정비는 새로운 벤치·비행 시험 결과를 추가하지 않는다.
]
