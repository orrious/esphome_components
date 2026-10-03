#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/gpio.h"
#include <cmath>

namespace esphome {
namespace jsdrive {

enum JSDriveOperation : uint8_t {
  JSDRIVE_OPERATION_IDLE = 0,
  JSDRIVE_OPERATION_RAISING,
  JSDRIVE_OPERATION_LOWERING,
};

const char *jsdrive_operation_to_str(JSDriveOperation op);

class JSDrive : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_remote_uart(uart::UARTComponent *uart) { this->remote_uart_ = uart; }
  void set_desk_uart(uart::UARTComponent *uart) { this->desk_uart_a_ = uart; }
  void set_desk_uart_a(uart::UARTComponent *uart) { this->desk_uart_a_ = uart; }
  void set_desk_uart_b(uart::UARTComponent *uart) { this->desk_uart_b_ = uart; }
  void set_message_length(int length) { this->message_length_ = length; }
  void set_height_sensor(sensor::Sensor *sensor) { height_sensor_ = sensor; }
  void set_up_bsensor(binary_sensor::BinarySensor *sensor) { up_bsensor_ = sensor; }
  void set_down_bsensor(binary_sensor::BinarySensor *sensor) { down_bsensor_ = sensor; }
  void set_memory1_bsensor(binary_sensor::BinarySensor *sensor) { memory1_bsensor_ = sensor; }
  void set_memory2_bsensor(binary_sensor::BinarySensor *sensor) { memory2_bsensor_ = sensor; }
  void set_memory3_bsensor(binary_sensor::BinarySensor *sensor) { memory3_bsensor_ = sensor; }
  void set_height_a_sensor(sensor::Sensor *sensor) { height_a_sensor_ = sensor; }
  void set_height_b_sensor(sensor::Sensor *sensor) { height_b_sensor_ = sensor; }
  void set_in_sync_sensor(binary_sensor::BinarySensor *sensor) { in_sync_sensor_ = sensor; }
  void set_remote_action_pin(GPIOPin *pin) { remote_action_pin_ = pin; }
  void set_box_a_action_pin(GPIOPin *pin) { box_a_action_pin_ = pin; }
  void set_box_b_action_pin(GPIOPin *pin) { box_b_action_pin_ = pin; }
  void set_box_a_awake_pin(GPIOPin *pin) { box_a_awake_pin_ = pin; }
  void set_box_b_awake_pin(GPIOPin *pin) { box_b_awake_pin_ = pin; }
  void set_remote_awake_pin(GPIOPin *pin) { remote_awake_pin_ = pin; }

  void move_to(float height);
  void stop();

  JSDriveOperation current_operation{JSDRIVE_OPERATION_IDLE};

 protected:
  uart::UARTComponent *remote_uart_{nullptr};
  uart::UARTComponent *desk_uart_a_{nullptr};
  uart::UARTComponent *desk_uart_b_{nullptr};
  int message_length_{5};
  sensor::Sensor *height_sensor_{nullptr};
  binary_sensor::BinarySensor *up_bsensor_{nullptr};
  binary_sensor::BinarySensor *down_bsensor_{nullptr};
  binary_sensor::BinarySensor *memory1_bsensor_{nullptr};
  binary_sensor::BinarySensor *memory2_bsensor_{nullptr};
  binary_sensor::BinarySensor *memory3_bsensor_{nullptr};
  sensor::Sensor *height_a_sensor_{nullptr};
  sensor::Sensor *height_b_sensor_{nullptr};
  binary_sensor::BinarySensor *in_sync_sensor_{nullptr};
  GPIOPin *remote_action_pin_{nullptr};
  GPIOPin *box_a_action_pin_{nullptr};
  GPIOPin *box_b_action_pin_{nullptr};
  GPIOPin *box_a_awake_pin_{nullptr};
  GPIOPin *box_b_awake_pin_{nullptr};
  GPIOPin *remote_awake_pin_{nullptr};

  std::vector<uint8_t> rem_buffer_;
  std::vector<uint8_t> desk_a_buffer_;
  std::vector<uint8_t> desk_b_buffer_;
  bool rem_rx_{false};
  bool desk_a_rx_{false};
  bool desk_b_rx_{false};
  bool last_remote_action_{false};
  bool ha_action_{false};
  float height_a_{NAN};
  float height_b_{NAN};
  uint32_t last_box_a_message_{0};
  uint32_t last_box_b_message_{0};
  float current_pos_{0};
  float target_pos_{-1};
  bool moving_{false};
  bool move_dir_;  // true is up
  uint32_t last_send_{0};

  void process_box_byte_(uint8_t c, bool box_a);
  void update_sync_();
  void forward_command_(const uint8_t *packet);
  void write_action_(bool active);
};

class JSDriveTargetNumber final : public number::Number {
 public:
  void set_parent(JSDrive *parent) { parent_ = parent; }

 protected:
  void control(float value) override;
  JSDrive *parent_{nullptr};
};

class JSDriveStopButton final : public button::Button {
 public:
  void set_parent(JSDrive *parent) { parent_ = parent; }

 protected:
  void press_action() override;
  JSDrive *parent_{nullptr};
};

}  // namespace jsdrive
}  // namespace esphome
