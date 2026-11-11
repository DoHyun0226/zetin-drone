#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "stream_format.h"
#include "stream_pacing.h"

static int g_failures = 0;

static void runCase(const std::string &name, const std::function<void()> &body) {
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    g_failures++;
  }
}

#define CHECK(expr)                                                          \
  do {                                                                       \
    if (!(expr)) throw std::runtime_error(#expr);                            \
  } while (0)

#define CHECK_EQ(a, b)                                                       \
  do {                                                                       \
    const auto lhs_ = (a);                                                   \
    const auto rhs_ = (b);                                                   \
    if (!(lhs_ == rhs_)) {                                                   \
      throw std::runtime_error(std::string(#a " == " #b " failed: ") +       \
                               std::to_string(lhs_) + " vs " +               \
                               std::to_string(rhs_));                        \
    }                                                                        \
  } while (0)

#define CHECK_STR(actual, expected)                                          \
  do {                                                                       \
    const std::string lhs_ = (actual);                                       \
    const std::string rhs_ = (expected);                                     \
    if (lhs_ != rhs_) {                                                      \
      throw std::runtime_error("string mismatch:\n  got:  " + lhs_ +         \
                               "\n  want: " + rhs_);                         \
    }                                                                        \
  } while (0)

int main() {
  runCase("part_header_exact_bytes", [] {
    char buf[256];
    const size_t n = formatPartHeader(buf, sizeof(buf), 12345, 3000000123LL, 7);
    const std::string expected =
        "Content-Type: image/jpeg\r\nContent-Length: 12345\r\n"
        "X-Timestamp: 3000.000123\r\nX-Frame-Seq: 7\r\n\r\n";
    CHECK_STR(std::string(buf, n), expected);
    CHECK_EQ(n, expected.size());
  });

  runCase("part_header_large_uptime", [] {
    char buf[256];
    const size_t n = formatPartHeader(buf, sizeof(buf), 1, 5000000000123LL, 0);
    CHECK(n > 0);
    CHECK(std::string(buf, n).find("X-Timestamp: 5000000.000123\r\n") != std::string::npos);
  });

  runCase("part_header_too_small_returns_zero", [] {
    char buf[20];
    CHECK_EQ(formatPartHeader(buf, sizeof(buf), 12345, 3000000123LL, 7), (size_t)0);
  });

  runCase("boundary_bytes", [] {
    char buf[64];
    const size_t n = formatBoundary(buf, sizeof(buf));
    CHECK_STR(std::string(buf, n), "\r\n--zetincamframe\r\n");
    CHECK_EQ(n, (size_t)19);
  });

  runCase("bucket_starts_full", [] {
    TokenBucket b(187500, 8192);
    b.reset(0);
    CHECK_EQ(b.waitUs(0, 8192), (uint32_t)0);
  });

  runCase("bucket_wait_after_drain", [] {
    TokenBucket b(187500, 8192);
    b.reset(0);
    b.consume(0, 8192);
    CHECK_EQ(b.waitUs(0, 4096), (uint32_t)21846);
  });

  runCase("bucket_refills_to_capacity_only", [] {
    TokenBucket b(187500, 8192);
    b.reset(0);
    b.consume(0, 8192);
    CHECK_EQ(b.waitUs(10000000, 8192), (uint32_t)0);
    b.consume(10000000, 8192);
    b.consume(10000000, 8192);
    CHECK(b.waitUs(10000000, 1) > 0);
  });

  runCase("bucket_oversized_request_waits_for_capacity", [] {
    TokenBucket b(187500, 8192);
    b.reset(0);
    b.consume(0, 8192);
    const uint32_t big = b.waitUs(0, 20000);
    CHECK_EQ(big, b.waitUs(0, 8192));
    CHECK_EQ(big, (uint32_t)43691);
  });

  runCase("frame_captured_before_stream_start_is_stale", [] {
    CHECK(frameIsStale(1000000, 61000000));    // sat in the single buffer for a minute
    CHECK(!frameIsStale(61000001, 61000000));  // captured after the client connected
    CHECK(!frameIsStale(61000000, 61000000));
  });

  if (g_failures != 0) {
    std::cerr << g_failures << " case(s) failed\n";
    return 1;
  }
  std::cout << "all camera stream cases passed\n";
  return 0;
}
