#pragma once
// In-memory stand-in for the ESP32 Preferences (NVS) class, native tests only.
#include <Arduino.h>
#include <map>
#include <vector>

inline std::map<std::string, std::vector<uint8_t>> g_nvs;

class Preferences {
public:
  bool begin(const char* ns, bool = false) { _ns = ns; return true; }
  void end() {}

  uint8_t getUChar(const char* key, uint8_t def = 0) {
    auto it = g_nvs.find(full(key));
    return (it == g_nvs.end() || it->second.empty()) ? def : it->second[0];
  }
  size_t putUChar(const char* key, uint8_t v) { g_nvs[full(key)] = {v}; return 1; }

  size_t putBytes(const char* key, const void* buf, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    g_nvs[full(key)] = std::vector<uint8_t>(p, p + len);
    return len;
  }
  size_t getBytes(const char* key, void* buf, size_t maxLen) {
    auto it = g_nvs.find(full(key));
    if (it == g_nvs.end()) return 0;
    size_t n = it->second.size() < maxLen ? it->second.size() : maxLen;
    memcpy(buf, it->second.data(), n);
    return n;
  }

private:
  std::string _ns;
  std::string full(const char* key) const { return _ns + "/" + key; }
};
