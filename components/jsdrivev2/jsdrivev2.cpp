#include "esphome/core/log.h"

namespace esphome {
namespace desktronic {

static const char* TAG = "desktronic";

// UART message to send when receiving 0xA5
static const uint8_t RESPONSE_MESSAGE[] = {0xA5, 0x00, 0x00, 0x00, 0x00};

class Desktronic {
public:
    void setup() {
        // Make sure UART is initialized
        if (!remote_uart_) {
            ESP_LOGE(TAG, "UART not initialized.");
            return;
        }
    }

    void loop() {
        // Respond to UART messages
        read_remote_uart();
    }

private:
    void read_remote_uart() {
        if (!remote_uart_) {
            return;
        }

        uint8_t byte;
        while (remote_uart_->available()) {
            remote_uart_->read_byte(&byte);

            // If we receive 0xA5, respond with the predefined message
            if (byte == 0xA5) {
                ESP_LOGI(TAG, "Received 0xA5, responding with: 0xA5 0x00 0x00 0x00 0x00");
                remote_uart_->write_array(RESPONSE_MESSAGE, sizeof(RESPONSE_MESSAGE));
            }
        }
    }

    // Assume `remote_uart_` is set up elsewhere as part of the system
   // uart::UARTComponent* remote_uart_;
};

}  // namespace desktronic
}  // namespace esphome
