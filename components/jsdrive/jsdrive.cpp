#include "jsdrive.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace jsdrive {

static const char *const TAG = "jsdrive";
static constexpr uint8_t COMMAND_START = 0xa5;
static constexpr uint8_t DISPLAY_START = 0x5a;

const char *jsdrive_operation_to_str(JSDriveOperation op) {
  switch (op) {
    case JSDRIVE_OPERATION_IDLE: return "IDLE";
    case JSDRIVE_OPERATION_RAISING: return "RAISING";
    case JSDRIVE_OPERATION_LOWERING: return "LOWERING";
    default: return "UNKNOWN";
  }
}

static int segs_to_num(uint8_t segments) {
  switch (segments & 0x7f) {
    case 0x3f: return 0;
    case 0x06: return 1;
    case 0x5b: return 2;
    case 0x4f: return 3;
    case 0x66: return 4;
    case 0x6d: return 5;
    case 0x7d: return 6;
    case 0x07: return 7;
    case 0x7f: return 8;
    case 0x6f: return 9;
    default: return -1;
  }
}

static bool valid_checksum(const uint8_t *packet, uint8_t length) {
  if (length == 5)
    return static_cast<uint8_t>(packet[1] + packet[2] + packet[3]) == packet[4];
  return static_cast<uint8_t>(packet[1] + packet[2] + packet[3] + packet[4]) == packet[5];
}

void JSDrive::write_action_(bool active) {
  if (this->box_a_action_pin_ != nullptr)
    this->box_a_action_pin_->digital_write(active);
  if (this->box_b_action_pin_ != nullptr)
    this->box_b_action_pin_->digital_write(active);
}

void JSDrive::setup() {
  this->last_remote_action_ = this->remote_action_pin_ != nullptr && this->remote_action_pin_->digital_read();
  this->write_action_(false);
  if (this->remote_awake_pin_ != nullptr)
    this->remote_awake_pin_->digital_write(false);
}

void JSDrive::forward_command_(const uint8_t *packet) {
  if (this->desk_uart_a_ != nullptr)
    this->desk_uart_a_->write_array(packet, 5);
  if (this->desk_uart_b_ != nullptr)
    this->desk_uart_b_->write_array(packet, 5);
}

void JSDrive::update_sync_() {
  bool fresh_a = this->last_box_a_message_ != 0 && (millis() - this->last_box_a_message_ < 250);
  bool fresh_b = this->last_box_b_message_ != 0 && (millis() - this->last_box_b_message_ < 250);
  bool have_a = fresh_a && !std::isnan(this->height_a_);
  bool have_b = fresh_b && !std::isnan(this->height_b_);
  bool synced = have_a && have_b && std::fabs(this->height_a_ - this->height_b_) <= 0.1f;
  if (this->in_sync_sensor_ != nullptr)
    this->in_sync_sensor_->publish_state(synced);
  if (have_a && have_b && !synced)
    ESP_LOGW(TAG, "control boxes out of sync: %.1f vs %.1f", this->height_a_, this->height_b_);
  if (this->remote_awake_pin_ != nullptr) {
    bool awake_a = this->box_a_awake_pin_ == nullptr || this->box_a_awake_pin_->digital_read();
    bool awake_b = this->box_b_awake_pin_ == nullptr || this->box_b_awake_pin_->digital_read();
    this->remote_awake_pin_->digital_write(awake_a && awake_b);
  }
}

void JSDrive::process_box_byte_(uint8_t c, bool box_a) {
  bool &receiving = box_a ? this->desk_a_rx_ : this->desk_b_rx_;
  std::vector<uint8_t> &buffer = box_a ? this->desk_a_buffer_ : this->desk_b_buffer_;
  if (!receiving) {
    if (c != DISPLAY_START)
      return;
    receiving = true;
    buffer.clear();
    return;
  }
  buffer.push_back(c);
  if (buffer.size() < this->message_length_ - 1)
    return;
  receiving = false;
  const uint8_t *d = buffer.data();
  const uint8_t length = this->message_length_;
  uint8_t packet[6] = {DISPLAY_START, 0, 0, 0, 0, 0};
  for (uint8_t i = 0; i < length - 1; i++)
    packet[i + 1] = d[i];
  if (!valid_checksum(packet, length)) {
    ESP_LOGW(TAG, "box %c display checksum mismatch", box_a ? 'A' : 'B');
    buffer.clear();
    return;
  }
  if (box_a)
    this->last_box_a_message_ = millis();
  else
    this->last_box_b_message_ = millis();

  // Forward one validated response to the handset. Box A is preferred.
  bool box_a_fresh = this->last_box_a_message_ != 0 && (millis() - this->last_box_a_message_ < 100);
  if (this->remote_uart_ != nullptr && (box_a || !box_a_fresh))
    this->remote_uart_->write_array(packet, length);

  const uint8_t *display = packet + 1;
  int d0 = segs_to_num(display[0]);
  int d1 = segs_to_num(display[1]);
  int d2 = segs_to_num(display[2]);
  float *height = box_a ? &this->height_a_ : &this->height_b_;
  sensor::Sensor *sensor = box_a ? this->height_a_sensor_ : this->height_b_sensor_;
  if (d0 >= 0 && d1 >= 0 && d2 >= 0) {
    float value = d0 * 100.0f + d1 * 10.0f + d2;
    if (display[1] & 0x80)
      value /= 10.0f;
    *height = value;
    if (sensor != nullptr)
      sensor->publish_state(value);
    if (this->height_sensor_ != nullptr) {
      float primary = !std::isnan(this->height_a_) ? this->height_a_ : this->height_b_;
      this->current_pos_ = primary;
      this->height_sensor_->publish_state(primary);
    }
    this->update_sync_();
  } else {
    *height = NAN;
    this->update_sync_();
  }
  buffer.clear();
}

void JSDrive::loop() {
  uint8_t c;
  if (this->remote_action_pin_ != nullptr) {
    bool action = this->remote_action_pin_->digital_read();
    this->last_remote_action_ = action;
    this->write_action_(this->last_remote_action_ || this->ha_action_);
  }
  if (this->desk_uart_a_ != nullptr) {
    while (this->desk_uart_a_->available()) {
      this->desk_uart_a_->read_byte(&c);
      this->process_box_byte_(c, true);
    }
  }
  if (this->desk_uart_b_ != nullptr) {
    while (this->desk_uart_b_->available()) {
      this->desk_uart_b_->read_byte(&c);
      this->process_box_byte_(c, false);
    }
  }
  if (this->remote_uart_ != nullptr) {
    static bool receiving = false;
    static std::vector<uint8_t> command;
    while (this->remote_uart_->available()) {
      this->remote_uart_->read_byte(&c);
      if (!receiving) {
        if (c != COMMAND_START)
          continue;
        receiving = true;
        command.clear();
      }
      command.push_back(c);
      if (command.size() < 5)
        continue;
      receiving = false;
      if (valid_checksum(command.data(), 5)) {
        uint8_t buttons = command[2];
        if (this->up_bsensor_ != nullptr)
          this->up_bsensor_->publish_state((buttons & 0x20) != 0);
        if (this->down_bsensor_ != nullptr)
          this->down_bsensor_->publish_state((buttons & 0x40) != 0);
        if (this->memory1_bsensor_ != nullptr)
          this->memory1_bsensor_->publish_state((buttons & 0x02) != 0);
        if (this->memory2_bsensor_ != nullptr)
          this->memory2_bsensor_->publish_state((buttons & 0x04) != 0);
        if (this->memory3_bsensor_ != nullptr)
          this->memory3_bsensor_->publish_state((buttons & 0x08) != 0);
        this->forward_command_(command.data());
      } else {
        ESP_LOGW(TAG, "remote command checksum mismatch");
      }
      command.clear();
    }
  }
  if (this->moving_ && (millis() - this->last_send_ >= 50)) {
    bool reached = this->move_dir_ ? this->current_pos_ >= this->target_pos_ : this->current_pos_ <= this->target_pos_;
    if (reached) {
      this->stop();
    } else {
      uint8_t packet[] = {COMMAND_START, 0x00, static_cast<uint8_t>(this->move_dir_ ? 0x20 : 0x40), 0x00, 0xff};
      packet[3] = static_cast<uint8_t>(0xff - packet[2]);
      this->forward_command_(packet);
      this->last_send_ = millis();
    }
  }
  this->update_sync_();
}

void JSDrive::dump_config() {
  ESP_LOGCONFIG(TAG, "JSDrive dual control-box bridge");
  LOG_SENSOR("  ", "Height", this->height_sensor_);
  LOG_SENSOR("  ", "Box A height", this->height_a_sensor_);
  LOG_SENSOR("  ", "Box B height", this->height_b_sensor_);
  LOG_BINARY_SENSOR("  ", "Boxes in sync", this->in_sync_sensor_);
  LOG_PIN("  Remote action pin: ", this->remote_action_pin_);
  LOG_PIN("  Box A action pin: ", this->box_a_action_pin_);
  LOG_PIN("  Box B action pin: ", this->box_b_action_pin_);
  LOG_PIN("  Box A awake pin: ", this->box_a_awake_pin_);
  LOG_PIN("  Box B awake pin: ", this->box_b_awake_pin_);
  LOG_PIN("  Remote awake pin: ", this->remote_awake_pin_);
}

void JSDrive::move_to(float height) {
  if (this->desk_uart_a_ == nullptr && this->desk_uart_b_ == nullptr)
    return;
  this->target_pos_ = height;
  this->move_dir_ = height > this->current_pos_;
  this->moving_ = true;
  this->ha_action_ = true;
  this->write_action_(true);
  this->current_operation = this->move_dir_ ? JSDRIVE_OPERATION_RAISING : JSDRIVE_OPERATION_LOWERING;
}

void JSDrive::stop() {
  this->moving_ = false;
  this->ha_action_ = false;
  this->write_action_(this->last_remote_action_);
  this->current_operation = JSDRIVE_OPERATION_IDLE;
  uint8_t packet[] = {COMMAND_START, 0x00, 0x00, 0xff, 0xff};
  this->forward_command_(packet);
}

void JSDriveTargetNumber::control(float value) {
  if (this->parent_ != nullptr) {
    this->parent_->move_to(value);
    this->publish_state(value);
  }
}

void JSDriveStopButton::press_action() {
  if (this->parent_ != nullptr)
    this->parent_->stop();
}

}  // namespace jsdrive
}  // namespace esphome
