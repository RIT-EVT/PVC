/**
 * SIM100 Driver Test
 */
#include <EVT/io/CAN.hpp>
#include <EVT/manager.hpp>
#include <EVT/utils/time.hpp>
#include <PreCharge/GFDB.hpp>
#include <PreCharge/PreCharge.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

void canIRQHandler(IO::CANMessage& message, void* priv) {
    IO::UART* uart = (IO::UART*) priv;
    uart->printf("Message received\r\n");
    uart->printf("Message id: %d \r\n", message.getId());
    uart->printf("Message length: %d\r\n", message.getDataLength());
    uart->printf("Message contents: ");

    uint8_t* message_payload = message.getPayload();
    for (int i = 0; i < message.getDataLength(); i++) {
        uart->printf("0x%02X ", message_payload[i]);
    }
    uart->printf("\r\n\r\n");
}

int main() {
    // Initialize system
    EVT::core::platform::init();

    // Get CAN instance with loopback enabled
    IO::CAN& can = IO::getCAN<PreCharge::PreCharge::CAN_TX_PIN, PreCharge::PreCharge::CAN_RX_PIN>();
    IO::UART& uart = IO::getUART<PreCharge::PreCharge::UART_TX_PIN, PreCharge::PreCharge::UART_RX_PIN>(9600, true);
    can.addIRQHandler(canIRQHandler, &uart);

    // Attempt to join the CAN network
    can.connect();

    uart.printf("Starting GFDB tests\r\n");
    GFDB::GFDB gfdb(can);

    int32_t temp = 0;
    uint16_t voltage = 0;
    int32_t vnHigh = 0;
    int32_t vpHigh = 0;

    // Super loop trying to send CAN messages and receive them in the CAN Interrupt
    while (true) {
        // Test temperature reading
        time::wait(1000);
        uart.printf("Testing temperature reading\r\n");
        if (gfdb.requestTemp(&temp) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read temperature\r\n");
        } else {
            uart.printf("Temperature: %d\r\n", temp);
        }

        // Test voltage reading
        time::wait(1000);
        uart.printf("Testing voltage reading\r\n");
        if (gfdb.requestBatteryVoltage(&voltage) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read voltage\r\n");
        } else {
            uart.printf("Voltage: %d\r\n", voltage);
        }

        time::wait(1000);
        // Test Negative Voltage High Res reading
        uart.printf("Testing Vn High Res reading\r\n");
        if (gfdb.requestVoltagePositiveHighRes(&vnHigh) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read Vn High Res\r\n");
        } else {
            uart.printf("Vn High Res: %d\r\n", vnHigh);
        }

        // Test Positive Voltage High Res reading
        time::wait(1000);
        uart.printf("Testing Vp High Res reading\r\n");
        if (gfdb.requestVoltageNegativeHighRes(&vpHigh) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read Vp High Res\r\n");
        } else {
            uart.printf("Vp High Res: %d\r\n", vpHigh);
        }

        // Print out all the values. Messages will most likely be handled by the interrupt though.
        uart.printf("--------------END---------------\r\n");
        uart.printf("Temperature: %d\r\n", temp);
        uart.printf("Voltage: %d\r\n", voltage);
        uart.printf("Vn High Res: %d\r\n", vnHigh);
        uart.printf("Vp High Res: %d\r\n", vpHigh);
    }
}
