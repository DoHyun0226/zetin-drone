// MJPEG 스트림의 경계·파트 헤더 문자열. Arduino 헤더 없이 호스트에서도 빌드된다
// (tools/native_tests/test_camera_stream.cpp).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

constexpr const char* kStreamBoundary = "zetincamframe";

// "\r\n--zetincamframe\r\n" 을 쓰고 길이를 돌려준다. 넘치면 0.
inline size_t formatBoundary(char* buf, size_t cap) {
  const int n = snprintf(buf, cap, "\r\n--%s\r\n", kStreamBoundary);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

// 파트 헤더를 쓰고 길이를 돌려준다. 넘치면 0.
// captureUs 는 esp_timer 부팅 후 µs(64비트)라 "%d" 로 찍으면 안 된다 — CameraWebServer
// 예제가 그렇게 해서 형식이 어긋난다. 명시 캐스트로 long long / unsigned long 에 맞춘다.
inline size_t formatPartHeader(char* buf, size_t cap, uint32_t jpegLen,
                               int64_t captureUs, uint32_t seq) {
  const long long sec = (long long)(captureUs / 1000000);
  const long usec = (long)(captureUs % 1000000);
  const int n = snprintf(buf, cap,
                         "Content-Type: image/jpeg\r\n"
                         "Content-Length: %lu\r\n"
                         "X-Timestamp: %lld.%06ld\r\n"
                         "X-Frame-Seq: %lu\r\n\r\n",
                         (unsigned long)jpegLen, sec, usec, (unsigned long)seq);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

// 스트림 시작보다 먼저 찍힌 프레임인가. PSRAM 없는 폴백(fb_count=1, GRAB_WHEN_EMPTY)은
// 클라이언트가 붙기 전에 채워진 버퍼를 그대로 내준다 — 보내면 녹화 영상에 그 사이의
// 대기 시간(수십 초일 수 있다)이 첫 프레임 길이로 끼어든다.
inline bool frameIsStale(int64_t captureUs, int64_t streamStartUs) {
  return captureUs < streamStartUs;
}
