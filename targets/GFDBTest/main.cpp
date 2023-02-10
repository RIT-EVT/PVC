/**
 * SIM100 Driver Test
 */
#include "EVT/io/CAN.hpp"
#include "EVT/io/manager.hpp"
#include "EVT/utils/time.hpp"
#include "PreCharge/GFDB.hpp"

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
    IO::init();

    // Get CAN instance with loopback enabled
    IO::CAN& can = IO::getCAN<IO::Pin::PA_12, IO::Pin::PA_11>();
    IO::UART& uart = IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600);
    can.addIRQHandler(canIRQHandler, &uart);

    uart.printf("Starting GFDB tests\r\n");
    GFDB::GFDB gfdb(can);

    // Attempt to join the CAN network
    if (gfdb.init() != IO::CAN::CANStatus::OK) {
        uart.printf("Failed to connect to the CAN network\r\n");
        return 1;
    }

    int32_t temp = 0;
    uint16_t voltage = 0;
    int32_t vnHigh = 0;
    int32_t vpHigh = 0;

    while (true) {
        uart.printf("Testing temperature reading\r\n");
        if (gfdb.requestTemp(&temp) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read temperature\r\n");
        } else {
            uart.printf("Temperature: %d\r\n", temp);
        }

        time::wait(1000);
        uart.printf("Testing voltage reading\r\n");
        if (gfdb.requestBatteryVoltage(&voltage) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read voltage\r\n");
        } else {
            uart.printf("Voltage: %d\r\n", voltage);
        }
        time::wait(1000);

        uart.printf("Testing Vn High Res reading\r\n");
        if (gfdb.requestVnHighRes(&vnHigh) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read Vn High Res\r\n");
        } else {
            uart.printf("Vn High Res: %d\r\n", vnHigh);
        }
        time::wait(1000);

        uart.printf("Testing Vp High Res reading\r\n");
        if (gfdb.requestVpHighRes(&vpHigh) != IO::CAN::CANStatus::OK) {
            uart.printf("Failed to read Vp High Res\r\n");
        } else {
            uart.printf("Vp High Res: %d\r\n", vpHigh);
        }
        time::wait(1000);

        uart.printf("--------------END---------------\r\n");

//        time::wait(1000);
        uart.printf("Temperature: %d\r\n", temp);
        uart.printf("Voltage: %d\r\n", voltage);
        uart.printf("Vn High Res: %d\r\n", vnHigh);
        uart.printf("Vp High Res: %d\r\n", vpHigh);

    }

    return 0;
}
