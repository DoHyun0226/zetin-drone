// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
//
// flix의 pid.h를 이 프로젝트에 맞게 수정한 버전:
// - 전역 시간 대신 호출자가 dt를 넘긴다 (1kHz 제어 태스크 기준).
// - D항은 setpoint 변화 kick을 피하기 위해 측정값(각속도) 미분에 건다.
// - 적분은 mixer 포화 시 중단할 수 있도록 별도 integrate() 호출로 분리한다.

#pragma once

#include "lpf.h"

class PID {
public:
	float p, i, d;
	float windup; // 적분항 절대 한계 (출력 단위)
	LowPassFilter<float> dlpf; // D항 저역 필터
	float integralTerm = 0;

	PID(float p, float i, float d, float windup = 0, float dAlpha = 1)
		: p(p), i(i), d(d), windup(windup), dlpf(dAlpha) {}

	float update(float error, float measurement, float dt) {
		float derivative = 0;
		if (isfinite(prevMeasurement) && dt > 0) {
			derivative = dlpf.update((measurement - prevMeasurement) / dt);
		}
		prevMeasurement = measurement;
		lastError = error;
		return p * error + integralTerm - d * derivative;
	}

	// mixer가 포화되지 않은 tick에서만 호출한다 (조건부 적분 anti-windup).
	void integrate(float dt) {
		integralTerm = constrain(integralTerm + i * lastError * dt, -windup, windup);
	}

	void reset() {
		integralTerm = 0;
		lastError = 0;
		prevMeasurement = NAN;
		dlpf.reset();
	}

private:
	float prevMeasurement = NAN;
	float lastError = 0;
};
