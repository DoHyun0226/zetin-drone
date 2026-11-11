// 스트림 송신 바이트율 상한용 토큰 버킷. 조각마다 waitUs → 대기 → consume 순으로 쓴다.
// 부동소수 없이 정수만 쓴다 — 토큰은 "바이트 × 10^6" 단위로 들고 있어서
// 리필이 elapsedUs × rate 한 번의 곱으로 끝난다.
#pragma once

#include <cstdint>

class TokenBucket {
 public:
  TokenBucket(uint32_t rateBytesPerS, uint32_t capacityBytes)
      : rate_(rateBytesPerS), capacityScaled_((int64_t)capacityBytes * kScale) {}

  // 가득 찬 상태로 시작한다.
  void reset(int64_t nowUs) {
    tokensScaled_ = capacityScaled_;
    lastUs_ = nowUs;
  }

  // 지금 bytes 를 보내려면 기다려야 할 µs (올림). 용량보다 큰 요청은 용량으로
  // 클램프해서 대기가 항상 유한하다.
  uint32_t waitUs(int64_t nowUs, uint32_t bytes) {
    refill(nowUs);
    int64_t needScaled = (int64_t)bytes * kScale;
    if (needScaled > capacityScaled_) needScaled = capacityScaled_;
    const int64_t deficit = needScaled - tokensScaled_;
    if (deficit <= 0 || rate_ == 0) return 0;
    // deficit[바이트·10^6] / rate[바이트/s] = µs
    return (uint32_t)((deficit + rate_ - 1) / rate_);
  }

  // 실제로 보낸 바이트만큼 토큰을 쓴다. 클램프하지 않으므로 음수가 될 수 있다
  // (그만큼 다음 대기가 길어진다).
  void consume(int64_t nowUs, uint32_t bytes) {
    refill(nowUs);
    tokensScaled_ -= (int64_t)bytes * kScale;
  }

 private:
  static constexpr int64_t kScale = 1000000;

  void refill(int64_t nowUs) {
    if (nowUs <= lastUs_) return;
    tokensScaled_ += (nowUs - lastUs_) * (int64_t)rate_;
    if (tokensScaled_ > capacityScaled_) tokensScaled_ = capacityScaled_;
    lastUs_ = nowUs;
  }

  uint32_t rate_;
  int64_t capacityScaled_;
  int64_t tokensScaled_ = 0;
  int64_t lastUs_ = 0;
};
