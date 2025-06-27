#include "jsdrive.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jsdrive {

static const char *const TAG = "jsdrive";

const char *jsdrive_operation_to_str(JSDriveOperation op) {
  switch (op) {
    case JSDRIVE_OPERATION_IDLE:
      return "IDLE";
    case JSDRIVE_OPERATION_RAISING:
      return "RAISING";
    case JSDRIVE_OPERATION_LOWERING:
      return "LOWERING";
    default:
      return "UNKNOWN";
  }
}

static int segs_to_num(uint8_t segments) {
  switch (segments & 0x7f) {
    case 0x3f:
      return 0;
    case 0x06:
      return 1;
    case 0x5b:
      return 2;
    case 0x4f:
      return 3;
    case 0x66: // New case for standard digit 4
    case 0x67: // Existing case for the non-standard digit 4
      return 4;
    case 0x6d:
      return 5;
    case 0x7d:
      return 6;
    case 0x07:
      return 7;
    case 0x7f:
      return 8;
    case 0x6f:
      return 9;
    case 0x79:
      return -2; // This value is unusual, typically for 'E' or 'F' on some displays, or a custom symbol.
    default:
      ESP_LOGE(TAG, "unknown digit: %02x", segments & 0x7f);
  }
  return -1;
}

void JSDrive::setup()
{
    if (move_pin_)
    {
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
    while (this->remote_uart_->available()) {
      this->remote_uart_->read_byte(&c);

      // Check if the first byte is 0xA5 (start of the 5-byte message)
      if (c == 0xA5) {
        // Send the response 0x5A 00 00 00 00
        static uint8_t response[] = {0x5A, 0x39, 0x00, 0x4F, 0x00};
        this->remote_uart_->write_array(response, 5);

        // Skip further processing of this message
        continue;
      }

      if (!this->rem_rx_) {
        if (c == 0xa5)
          this->rem_rx_ = true;
        continue;
      }
      this->rem_buffer_.push_back(c);
      if (this->rem_buffer_.size() < 4)
        continue;
      this->rem_rx_ = false;
      uint8_t *d = this->rem_buffer_.data();
      uint8_t csum = d[0] + d[1] + d[2];
      if (csum != d[3]) {
        ESP_LOGE(TAG, "remote checksum mismatch: %02x != %02x", csum, d[3]);
        this->rem_buffer_.clear();
        continue;
      }
      buttons = d[1];
      have_data = true;
      this->rem_buffer_.clear();
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
        static uint8_t buf[] = {0xa5, 0, buttons, (uint8_t) (0xff - buttons), 0xff};
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
