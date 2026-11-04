#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include <array>
#include <cmath>
#include <string>

namespace esphome {
namespace jsdrive2 {

class JSDrive2;

class JSDrive2TargetNumber final : public number::Number {
 public:
  void set_parent(JSDrive2 *parent) { parent_ = parent; }
 protected:
  void control(float value) override;
  JSDrive2 *parent_{nullptr};
};

class JSDrive2Cover final : public cover::Cover {
 public:
  void set_parent(JSDrive2 *parent) { parent_ = parent; }
  cover::CoverTraits get_traits() override;
 protected:
  void control(const cover::CoverCall &call) override;
  JSDrive2 *parent_{nullptr};
};

class JSDrive2CommandButton final : public button::Button {
 public:
  void set_parent(JSDrive2 *parent) { parent_ = parent; }
  void set_mode(uint8_t mode) { mode_ = mode; }
 protected:
  void press_action() override;
  JSDrive2 *parent_{nullptr};
  uint8_t mode_{0};
};

class JSDrive2 : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_remote_uart(uart::UARTComponent *v) { remote_uart_ = v; }
  void set_desk_uart_a(uart::UARTComponent *v) { box_a_.uart = v; }
  void set_desk_uart_b(uart::UARTComponent *v) { box_b_.uart = v; }
  void set_remote_action_pin(GPIOPin *v) { remote_action_pin_ = v; }
  void set_box_a_action_pin(GPIOPin *v) { box_a_.action_pin = v; }
  void set_box_b_action_pin(GPIOPin *v) { box_b_.action_pin = v; }
  void set_box_a_awake_pin(GPIOPin *v) { box_a_.awake_pin = v; }
  void set_box_b_awake_pin(GPIOPin *v) { box_b_.awake_pin = v; }
  void set_remote_awake_pin(GPIOPin *v) { remote_awake_pin_ = v; }
  void set_limits(float low, float high) { min_height_ = low; max_height_ = high; }
  void set_hard_skew(float value) { hard_skew_ = value; }
  void set_height_sensor(sensor::Sensor *v) { height_sensor_ = v; }
  void set_height_a_sensor(sensor::Sensor *v) { height_a_sensor_ = v; }
  void set_height_b_sensor(sensor::Sensor *v) { height_b_sensor_ = v; }
  void set_height_difference_sensor(sensor::Sensor *v) { height_difference_sensor_ = v; }
#define JSD_SET_SENSOR(name) void set_##name##_sensor(sensor::Sensor *v) { name##_sensor_ = v; }
  JSD_SET_SENSOR(remote_frame_age) JSD_SET_SENSOR(box_a_frame_age) JSD_SET_SENSOR(box_b_frame_age)
  JSD_SET_SENSOR(remote_invalid_frames) JSD_SET_SENSOR(box_a_invalid_frames) JSD_SET_SENSOR(box_b_invalid_frames)
  JSD_SET_SENSOR(remote_tx_max_gap) JSD_SET_SENSOR(box_a_tx_max_gap) JSD_SET_SENSOR(box_b_tx_max_gap)
  JSD_SET_SENSOR(alignment_attempts)
#undef JSD_SET_SENSOR
#define JSD_SET_BIN(name) void set_##name##_sensor(binary_sensor::BinarySensor *v) { name##_sensor_ = v; }
  JSD_SET_BIN(in_sync) JSD_SET_BIN(height_known) JSD_SET_BIN(height_live) JSD_SET_BIN(healthy)
  JSD_SET_BIN(remote_connected) JSD_SET_BIN(box_a_connected) JSD_SET_BIN(box_b_connected)
  JSD_SET_BIN(box_a_awake) JSD_SET_BIN(box_b_awake) JSD_SET_BIN(armed) JSD_SET_BIN(moving)
  JSD_SET_BIN(up) JSD_SET_BIN(down) JSD_SET_BIN(memory1) JSD_SET_BIN(memory2)
  JSD_SET_BIN(memory3) JSD_SET_BIN(memory4)
#undef JSD_SET_BIN
#define JSD_SET_TEXT(name) void set_##name##_sensor(text_sensor::TextSensor *v) { name##_sensor_ = v; }
  JSD_SET_TEXT(operation) JSD_SET_TEXT(activity) JSD_SET_TEXT(last_fault) JSD_SET_TEXT(box_a_display)
  JSD_SET_TEXT(box_b_display) JSD_SET_TEXT(error_code)
#undef JSD_SET_TEXT
  void set_cover(JSDrive2Cover *v) { cover_ = v; }

  bool request_height(float height);
  bool recall_preset(uint8_t preset);
  bool save_preset(uint8_t preset);
  bool discover_height();
  void stop(const char *reason = "stopped");
  void arm();
  void disarm();
  void run_button(uint8_t mode);
  float min_height() const { return min_height_; }
  float max_height() const { return max_height_; }
  float confirmed_height() const { return confirmed_height_; }

 protected:
  enum class DisplayState : uint8_t { UNKNOWN, BLANK, HEIGHT, MEMORY, ERROR, OTHER };
  enum class MotionState : uint8_t { IDLE, DISCOVER_WAIT, COARSE, FINE_PULSE, FINE_SETTLE, PRESET, SAVE_M, SAVE_SLOT };
  enum class WakeState : uint8_t { IDLE, ASSERT, HOLD, WAIT_HEIGHT, FAILED };
  enum class AlignState : uint8_t { IDLE, PULSE, SETTLE };
  enum class PresetPhase : uint8_t { IDLE, ANNOUNCEMENT, WAIT_MOTION, MOVING };
  struct Parser { uint8_t start{0}; uint8_t data[5]{}; uint8_t length{0}; uint32_t invalid{0}; };
  struct HeightQueue {
    static constexpr uint8_t CAPACITY = 8;
    std::array<float, CAPACITY> values{};
    std::array<uint32_t, CAPACITY> times{};
    uint8_t count{0};
  };
  struct Box {
    const char *name{nullptr};
    uart::UARTComponent *uart{nullptr}; GPIOPin *action_pin{nullptr}; GPIOPin *awake_pin{nullptr};
    Parser parser{0x5A}; bool seen{false}; bool awake{false}; bool action{false};
    uint32_t last_frame{0}, last_height{0}, last_change{0}, last_tx{0}, tx_count{0}, tx_errors{0}, max_tx_gap{0};
    uint32_t sleep_low_since{0}, wake_started{0}, last_nonidle_tx{0};
    bool wake_attempted{false}; WakeState wake{WakeState::IDLE};
    float live_height{NAN}, last_good{NAN}; DisplayState display{DisplayState::UNKNOWN};
    std::array<uint8_t, 3> glyph{{0,0,0}};
  };

  uart::UARTComponent *remote_uart_{nullptr};
  Box box_a_{"A"}, box_b_{"B"}; Parser remote_parser_{0xA5};
  GPIOPin *remote_action_pin_{nullptr}, *remote_awake_pin_{nullptr};
  bool armed_{true}, remote_seen_{false}, remote_action_{false}, remote_button_down_{false};
  bool remote_wake_consumed_{false};
  uint8_t remote_command_{0}; uint32_t remote_last_frame_{0}, remote_last_tx_{0}, remote_tx_count_{0}, remote_tx_errors_{0}, remote_max_tx_gap_{0};
  uint8_t remote_input_command_{0}, bridge_output_command_{0}, activity_command_{0};
  float min_height_{25.9f}, max_height_{51.5f}, hard_skew_{0.5f}, confirmed_height_{NAN};
  MotionState motion_{MotionState::IDLE}; uint32_t motion_started_{0}, phase_started_{0}, last_command_tx_{0};
  AlignState align_{AlignState::IDLE}; uint32_t align_started_{0};
  uint32_t align_wake_started_{0}; bool align_wake_attempted_{false};
  uint8_t align_high_box_{0xFF}; uint8_t align_duration_{0};
  bool override_align_{false}; uint32_t override_started_{0};
  uint16_t align_gap_before_{0}, align_lower_before_{0}; uint8_t align_no_progress_{0};
  uint32_t recovery_started_{0}, recovery_last_attempt_{0}, last_auto_wake_{0}; bool recovery_attempted_{false};
  float target_height_{NAN}; int8_t direction_{0}; uint8_t fine_taps_{0}, active_preset_{0}, save_preset_{0}, pending_preset_{0}, align_attempts_{0};
  bool preset_motion_seen_{false}, sos_{false}; uint32_t sos_until_{0};
  PresetPhase preset_phase_{PresetPhase::IDLE}; HeightQueue preset_queue_a_{}, preset_queue_b_{};
  float preset_start_{NAN}, preset_target_{NAN}, preset_raw_a_{NAN}, preset_raw_b_{NAN};
  uint32_t preset_last_change_{0}, preset_first_motion_{0}, preset_pair_wait_{0};
  bool preset_from_handset_{false}, preset_announcement_visible_{false};
  std::string fault_{"none"}, error_{"none"}; uint32_t last_publish_{0};

  sensor::Sensor *height_sensor_{nullptr}, *height_a_sensor_{nullptr}, *height_b_sensor_{nullptr}, *height_difference_sensor_{nullptr};
#define JSD_SENSOR(name) sensor::Sensor *name##_sensor_{nullptr};
  JSD_SENSOR(remote_frame_age) JSD_SENSOR(box_a_frame_age) JSD_SENSOR(box_b_frame_age)
  JSD_SENSOR(remote_invalid_frames) JSD_SENSOR(box_a_invalid_frames) JSD_SENSOR(box_b_invalid_frames)
  JSD_SENSOR(remote_tx_max_gap) JSD_SENSOR(box_a_tx_max_gap) JSD_SENSOR(box_b_tx_max_gap) JSD_SENSOR(alignment_attempts)
#undef JSD_SENSOR
#define JSD_BIN(name) binary_sensor::BinarySensor *name##_sensor_{nullptr};
  JSD_BIN(in_sync) JSD_BIN(height_known) JSD_BIN(height_live) JSD_BIN(healthy)
  JSD_BIN(remote_connected) JSD_BIN(box_a_connected) JSD_BIN(box_b_connected)
  JSD_BIN(box_a_awake) JSD_BIN(box_b_awake) JSD_BIN(armed) JSD_BIN(moving)
  JSD_BIN(up) JSD_BIN(down) JSD_BIN(memory1) JSD_BIN(memory2) JSD_BIN(memory3) JSD_BIN(memory4)
#undef JSD_BIN
#define JSD_TEXT(name) text_sensor::TextSensor *name##_sensor_{nullptr};
  JSD_TEXT(operation) JSD_TEXT(activity) JSD_TEXT(last_fault) JSD_TEXT(box_a_display) JSD_TEXT(box_b_display) JSD_TEXT(error_code)
#undef JSD_TEXT
  JSDrive2Cover *cover_{nullptr};

  static constexpr uint32_t STREAM_MS = 10, LINK_MS = 500, LIVE_MS = 150, TARGET_TIMEOUT_MS = 45000;
  static constexpr uint32_t FINE_PULSE_MS = 190, FINE_SETTLE_MS = 500, PRESET_PAIR_MS = 500;
  static constexpr uint32_t ACTION_WATCHDOG_MS = 30, SLEEP_STABLE_MS = 50, WAKE_RETRY_MS = 3000;
  static constexpr float COARSE_RELEASE = 0.3f;
  static const uint8_t CMD_IDLE[5], CMD_UP[5], CMD_DOWN[5], CMD_M[5], CMD_PRESET[4][5];
  bool parse_(Parser &parser, uint8_t byte, uint8_t out[5]);
  void poll_uart_(uart::UARTComponent *uart, Parser &parser, bool remote, Box *box = nullptr);
  void handle_remote_(const uint8_t frame[5]);
  void handle_box_(Box &box, const uint8_t frame[5]);
  void send_box_(Box &box, const uint8_t frame[5]);
  void send_both_(const uint8_t frame[5], bool action);
  void send_remote_status_();
  void set_actions_(bool a, bool b);
  void set_box_action_(Box &box, bool active);
  void poll_action_watchdog_(uint32_t now);
  void poll_box_idle_(Box &box, uint32_t now);
  void start_wake_(Box &box);
  void poll_wake_(Box &box, uint32_t now);
  void poll_automatic_wake_(uint32_t now);
  bool wake_ready_() const;
  bool fresh_height_(const Box &box, uint32_t now) const;
  void start_preset_(uint8_t preset, bool from_handset);
  void clear_preset_();
  void handle_preset_height_(Box &box, float height, uint32_t now);
  void commit_preset_pair_(float height, uint32_t now);
  void poll_preset_(uint32_t now);
  bool recover_();
  bool box_ready_(const Box &box, uint32_t now) const;
  bool trusted_equal_() const;
  bool live_equal_() const;
  bool movement_ready_(bool allow_cached) const;
  void start_command_(uint8_t command, bool from_remote);
  void poll_motion_();
  void poll_alignment_();
  void fail_(const std::string &reason, bool disarm = true, bool show_sos = true);
  void finish_motion_(const char *reason);
  void update_confirmed_();
  void publish_();
  std::string display_text_(const Box &box) const;
  void set_handset_buttons_(uint8_t command);
  void set_bridge_command_(uint8_t command);
  void publish_activity_();
  const uint8_t *command_frame_(uint8_t command) const;
};

template<typename... Ts> class JSDrive2SetHeightAction : public Action<Ts...> {
 public:
  explicit JSDrive2SetHeightAction(JSDrive2 *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(float, height)
  void play(Ts... x) override { parent_->request_height(this->height_.value(x...)); }
 protected: JSDrive2 *parent_;
};

template<typename... Ts> class JSDrive2PresetAction : public Action<Ts...> {
 public:
  explicit JSDrive2PresetAction(JSDrive2 *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint8_t, preset)
  void set_save(bool save) { save_ = save; }
  void play(Ts... x) override { uint8_t p = this->preset_.value(x...); save_ ? parent_->save_preset(p) : parent_->recall_preset(p); }
 protected: JSDrive2 *parent_; bool save_{false};
};

template<typename... Ts> class JSDrive2SimpleAction : public Action<Ts...> {
 public:
  explicit JSDrive2SimpleAction(JSDrive2 *parent) : parent_(parent) {}
  void set_mode(uint8_t mode) { mode_ = mode; }
  void play(Ts... x) override { parent_->run_button(mode_); }
 protected: JSDrive2 *parent_; uint8_t mode_{0};
};

}  // namespace jsdrive2
}  // namespace esphome
