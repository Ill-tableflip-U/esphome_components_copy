#include "jsdrive.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jsdrive {

static const char *const TAG = "jsdrive";

void JSDrive::setup() {
  if (move_pin_) {
    move_pin_->digital_write(false);
  }
}

void JSDrive::loop() {
  uint8_t c;
  bool have_data = false;

  if (this->desk_uart_ != nullptr) {
    // Your existing desk UART reading and processing here
    // ...
  }

  if (this->moving_) {
    if ((this->move_dir_ && (this->current_pos_ >= this->target_pos_)) ||
        (!this->move_dir_ && (this->current_pos_ <= this->target_pos_))) {
      this->moving_ = false;
      if (move_pin_)
        move_pin_->digital_write(false);
    } else {
      static uint8_t buf[] = {0xa5, 0, 0, 0, 0xff};
      buf[2] = (this->move_dir_ ? 0x20 : 0x40);
      buf[3] = 0xff - buf[2];
      this->desk_uart_->write_array(buf, 5);
    }
  }

  uint8_t buttons = 0;
  have_data = false;

  if (this->remote_uart_ != nullptr) {
    // Check UART availability and read non-blocking
    while (this->remote_uart_->available()) {
      this->remote_uart_->read_byte(&c);

      // If the first byte is 0xA5, send the response immediately
      if (c == 0xA5) {
        // Send the response 0x5A 00 00 00 00 immediately
        static uint8_t response[] = {0x5A, 0x00, 0x00, 0x00, 0x00};
        this->remote_uart_->write_array(response, 5);
        
        // Manually clear the UART buffer by reading and discarding any leftover bytes
        // This ensures we're only responding to fresh bytes, preventing delays from queued data
        while (this->remote_uart_->available()) {
          this->remote_uart_->read_byte(&c); // Discard bytes
        }
        
        // Return immediately to handle the next byte right away
        return;
      }
      
      // If we encounter an unexpected byte (not 0xA5), process as needed
      // ...
    }

    if (have_data) {
      if (this->up_bsensor_ != nullptr)
        this->up_bsensor_->publish_state(buttons & 0x20);
      if (this->down_bsensor_ != nullptr)
        this->down_bsensor_->publish_state(buttons & 0x40);
      if (this->memory1_bsensor_ != nullptr)
        this->memory1_bsensor_->publish_state(buttons & 2);
      if (this->memory2_bsensor_ != nullptr)
        this->memory2_bsensor_->publish_state(buttons & 4);
      if (this->memory3_bsensor_ != nullptr)
        this->memory3_bsensor_->publish_state(buttons & 8);
      if (this->memory4_bsensor_ != nullptr)
        this->memory4_bsensor_->publish_state(buttons & 16);

      if (!this->moving_ && this->desk_uart_ != nullptr) {
        static uint8_t buf[] = {0xa5, 0, buttons, (uint8_t)(0xff - buttons), 0xff};
        this->desk_uart_->write_array(buf, 5);
      }
    }
  }
}

void JSDrive::dump_config() {
  ESP_LOGCONFIG(TAG, "JSDrive Desk");
  if (this->desk_uart_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Message Length: %d", this->message_length_);
  LOG_PIN("Move Pin: ", move_pin_);
  LOG_SENSOR("", "Height", this->height_sensor_);
  LOG_BINARY_SENSOR("  ", "Up", this->up_bsensor_);
  LOG_BINARY_SENSOR("  ", "Down", this->down_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory1", this->memory1_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory2", this->memory2_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory3", this->memory3_bsensor_);
  LOG_BINARY_SENSOR("  ", "Memory4", this->memory4_bsensor_);
}

void JSDrive::move_to(float height) {
  if (this->desk_uart_ == nullptr)
    return;
  this->moving_ = true;
  this->target_pos_ = height;
  this->move_dir_ = height > this->current_pos_;
  this->current_operation = this->move_dir_ ? JSDRIVE_OPERATION_RAISING : JSDRIVE_OPERATION_LOWERING;

  if (move_pin_)
    move_pin_->digital_write(true);
}

void JSDrive::stop() {
  if (move_pin_)
  {
    move_pin_->digital_write(false);
  }
  this->moving_ = false;
  this->current_operation = JSDRIVE_OPERATION_IDLE;
}

}  // namespace jsdrive
}  // namespace esphome
