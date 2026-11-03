#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace esphome {
inline uint32_t test_ms = 1000;
inline uint32_t millis() { return test_ms; }
namespace setup_priority { constexpr float DATA = 1.0f; }
class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual float get_setup_priority() const { return 0; }
};
class GPIOPin {
 public:
  bool level{false};
  virtual ~GPIOPin() = default;
  virtual void setup() {}
  virtual bool digital_read() { return level; }
  virtual void digital_write(bool value) { level=value; }
};
namespace uart {
class UARTComponent {
 public:
  std::deque<uint8_t> rx;
  std::vector<std::vector<uint8_t>> tx;
  virtual ~UARTComponent() = default;
  virtual bool available() { return !rx.empty(); }
  virtual bool read_byte(uint8_t *out) {
    if (rx.empty()) return false;
    *out=rx.front(); rx.pop_front(); return true;
  }
  virtual void write_array(const uint8_t *data, size_t len) { tx.emplace_back(data,data+len); }
  void feed(std::initializer_list<uint8_t> frame) { rx.insert(rx.end(),frame.begin(),frame.end()); }
};
}
namespace sensor { class Sensor { public: float state{0}; void publish_state(float v) { state=v; } }; }
namespace binary_sensor { class BinarySensor { public: bool state{false}; void publish_state(bool v) { state=v; } }; }
namespace text_sensor { class TextSensor { public: std::string state; void publish_state(const std::string &v) { state=v; } }; }
namespace button { class Button { protected: virtual void press_action() {} }; }
namespace number { class Number { public: float state{0}; void publish_state(float v) { state=v; } protected: virtual void control(float) {} }; }
namespace cover {
enum CoverOperation { COVER_OPERATION_IDLE, COVER_OPERATION_OPENING, COVER_OPERATION_CLOSING };
class CoverTraits { public: void set_supports_position(bool) {} void set_supports_stop(bool) {} };
class CoverCall {
 public:
  bool stop{false}; std::optional<float> position;
  bool get_stop() const { return stop; }
  std::optional<float> get_position() const { return position; }
};
class Cover {
 public:
  float position{0}; CoverOperation current_operation{COVER_OPERATION_IDLE};
  virtual CoverTraits get_traits() { return {}; }
  void publish_state() {}
 protected: virtual void control(const CoverCall &) {}
};
}
template<typename... Ts> class Action { public: virtual ~Action() = default; virtual void play(Ts...) {} };
}
#define TEMPLATABLE_VALUE(type,name) struct { template<typename... Args> type value(Args...) const { return {}; } } name##_;
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGCONFIG(...) ((void)0)
#define LOG_PIN(...) ((void)0)
