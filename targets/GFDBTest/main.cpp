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
    IO::CANf3xx* can = (IO::CANf3xx*) priv;
    can->addCANMessage(message);
}

int main() {
    // Initialize system
    EVT::core::platform::init();

    // Get CAN instance with loopback enabled
    IO::CAN& can = IO::getCAN<PreCharge::PreCharge::CAN_TX_PIN, PreCharge::PreCharge::CAN_RX_PIN>();
    IO::UART& uart = IO::getUART<PreCharge::PreCharge::UART_TX_PIN, PreCharge::PreCharge::UART_RX_PIN>(9600, true);
    can.addIRQHandler(canIRQHandler, &can);

    uart.printf("Starting GFDB tests\r\n");
    GFDB::GFDB gfdb(can);

    int32_t temp = 0;
    uint16_t voltage = 0;
    int32_t vnHigh = 0;
    int32_t vpHigh = 0;
    uint8_t isoState = 0;

    // Attempt to join the CAN network
    IO::CAN::CANStatus result = can.connect();

    if (result != IO::CAN::CANStatus::OK) {
        uart.printf("Failed to connect to the CAN network\r\n");
        return 1;
    }

    while (true) {
        // Test temperature reading
        uart.printf("Testing temperature reading\r\n");
        if (gfdb.requestTemp(&temp) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read temperature\r\n");
        } else {
            uart.printf("Temperature: %d.%03d\r\n", temp / 1000, temp % 1000);
        }

        // Test voltage reading
        uart.printf("Testing voltage reading\r\n");
        if (gfdb.requestBatteryVoltage(&voltage) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read voltage\r\n");
        } else {
            uart.printf("Voltage: %d\r\n", voltage);
        }

        // Test Negative Voltage High Res reading
        uart.printf("Testing Vn High Res reading\r\n");
        if (gfdb.requestVoltagePositiveHighRes(&vnHigh) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read Vn High Res\r\n");
        } else {
            uart.printf("Vn High Res: %d\r\n", vnHigh);
        }

        // Test Positive Voltage High Res reading
        uart.printf("Testing Vp High Res reading\r\n");
        if (gfdb.requestVoltageNegativeHighRes(&vpHigh) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read Vp High Res\r\n");
        } else {
            uart.printf("Vp High Res: %d\r\n", vpHigh);
        }

        // Test Isolation State Reading
        result = gfdb.requestIsolationState(&isoState);
        if (result != IO::CAN::CANStatus::OK) {
            uart.printf("ERROR: %d\r\n", result);
        }
        uart.printf("Isolation State: %d\r\n", isoState);

        uart.printf("--------------END---------------\r\n\r\n");
        time::wait(2000);
    }
}
