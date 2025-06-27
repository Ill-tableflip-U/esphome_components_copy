#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h" // Include the necessary header for UART

namespace esphome {
namespace desktronic {

static const char* TAG = "desktronic";

// UART message to send when receiving 0xA5
static const uint8_t RESPONSE_MESSAGE[] = {0xA5, 0x00, 0x00, 0x00, 0x00};

class Desktronic {
public:
    // The setup function will initialize the UART and check if it's ready
    void setup(uart::UARTComponent *remote_uart) {
        remote_uart_ = remote_uart;  // Initialize the remote_uart pointer
        if (!remote_uart_) {
            ESP_LOGE(TAG, "UART not initialized.");
        }
    }

    void loop() {
        // Respond to UART messages
        read_remote_uart();
    }

private:
    // Function to read and respond to UART messages
    void read_remote_uart() {
        if (!remote_uart_) {
            return; // If remote_uart is not initialized, exit
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

    uart::UARTComponent *remote_uart_ = nullptr; // Declare the UART component pointer
};

}  // namespace desktronic
}  // namespace esphome
