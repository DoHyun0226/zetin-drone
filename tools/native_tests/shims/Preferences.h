#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

// In-memory stand-in for the ESP32 Preferences (NVS) API. One store per process;
// tests call preferences_fake::reset() to start from an empty flash.
namespace preferences_fake {
inline std::map<std::string, std::vector<uint8_t>> store;
inline void reset() { store.clear(); }
}  // namespace preferences_fake

class Preferences {
public:
  bool begin(const char *name, bool read_only = false) {
    ns_ = name;
    read_only_ = read_only;
    open_ = true;
    return true;
  }
  void end() { open_ = false; }

  size_t getBytesLength(const char *key) {
    auto it = preferences_fake::store.find(full(key));
    return it == preferences_fake::store.end() ? 0 : it->second.size();
  }
  size_t getBytes(const char *key, void *buf, size_t max_len) {
    auto it = preferences_fake::store.find(full(key));
    if (it == preferences_fake::store.end()) return 0;
    const size_t n = it->second.size() < max_len ? it->second.size() : max_len;
    std::memcpy(buf, it->second.data(), n);
    return n;
  }
  size_t putBytes(const char *key, const void *value, size_t len) {
    if (!open_ || read_only_) return 0;
    const uint8_t *p = static_cast<const uint8_t *>(value);
    preferences_fake::store[full(key)] = std::vector<uint8_t>(p, p + len);
    return len;
  }

private:
  std::string full(const char *key) const { return ns_ + "/" + key; }
  std::string ns_;
  bool read_only_ = false;
  bool open_ = false;
};
