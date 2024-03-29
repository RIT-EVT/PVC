#include <cstring>

#include <EVT/io/GPIO.hpp>
#include <EVT/io/UART.hpp>
#include <EVT/manager.hpp>
#include <PreCharge/PreCharge.hpp>

namespace IO = EVT::core::IO;

constexpr size_t MAX_BUFF = 100;

void printHelp(IO::UART& uart) {
    uart.printf("Menu\n\r");
    uart.printf("\t1. Read current digital inputs\n\r");
    uart.printf("\t2. Write to MCU_APM_CTL\n\r");
    uart.printf("\t3. Write to MCU_FW_EN_CTL\n\r");
    uart.printf("\t4. Write to MCU_PC_CTL\n\r");
    uart.printf("\t5. Write to MCU_DC_CTL\n\r");
    uart.printf("\t6. Write to MCU_CONT_CTL\n\r");
}

void writeGPIO(IO::UART& uart, IO::GPIO& gpio) {
    char inputBuffer[MAX_BUFF];
    uart.printf("Enter a value (0 or 1): ");
    uart.gets(inputBuffer, MAX_BUFF);
    uart.printf("\n\r");

    uint8_t state = strtol(inputBuffer, nullptr, 10);

    if (state != 0 && state != 1) {
        uart.printf("INVALID STATE: %d\n", state);
        return;
    }

    gpio.writePin(state == 0 ? IO::GPIO::State::LOW : IO::GPIO::State::HIGH);
}

// 4 digital inputs
// 1 ADC input
// 5 outputs

int main() {
    // Initialize system
    EVT::core::platform::init();

    char inputBuffer[MAX_BUFF];

    IO::UART& uart = IO::getUART<PreCharge::PreCharge::UART_TX_PIN, PreCharge::PreCharge::UART_RX_PIN>(9600, true);

    // Outputs
    IO::GPIO& mcuAPMCtl = IO::getGPIO<PreCharge::PreCharge::APM_CTL_PIN>();
    //    IO::GPIO& mcuFWEnCtl = IO::getGPIO<IO::Pin::PA_3>();
    IO::GPIO& mcuPcCtl = IO::getGPIO<PreCharge::PreCharge::PC_CTL_PIN>();
    IO::GPIO& mcuDcCtl = IO::getGPIO<PreCharge::PreCharge::DC_CTL_PIN>();
    IO::GPIO& mcuContCtl = IO::getGPIO<PreCharge::PreCharge::CONT1_PIN>();

    // Inputs
    IO::GPIO& mcuEStopStatus = IO::getGPIO<PreCharge::PreCharge::ESTOP_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& keyInMcu = IO::getGPIO<PreCharge::PreCharge::KEY_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& battery1Ok = IO::getGPIO<PreCharge::PreCharge::BAT_OK_1_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& battery2Ok = IO::getGPIO<PreCharge::PreCharge::BAT_OK_2_PIN>(IO::GPIO::Direction::INPUT);

    // Start with everything at 0
    mcuAPMCtl.writePin(IO::GPIO::State::LOW);
    //    mcuFWEnCtl.writePin(IO::GPIO::State::LOW);
    mcuPcCtl.writePin(IO::GPIO::State::LOW);
    mcuDcCtl.writePin(IO::GPIO::State::LOW);
    mcuContCtl.writePin(IO::GPIO::State::LOW);

    while (true) {
        printHelp(uart);

        uart.printf("Enter selection: ");
        uart.gets(inputBuffer, MAX_BUFF);
        uart.printf("\r\n");

        uint8_t cmd = strtol(inputBuffer, nullptr, 10);

        switch (cmd) {
        case 1:
            uart.printf("Input Values:\n\r");
            uart.printf("\tMCU_ESTOP_STATUS: %d\n\r", mcuEStopStatus.readPin());
            uart.printf("\tKEY_IN_MCU:       %d\n\r", keyInMcu.readPin());
            uart.printf("\tBATTERY_1_OK_MCU: %d\n\r", battery1Ok.readPin());
            uart.printf("\tBATTERY_2_OK_MCU: %d\n\r", battery2Ok.readPin());
            break;
        case 2:
            writeGPIO(uart, mcuAPMCtl);
            break;
        case 3:
            //            writeGPIO(uart, mcuFWEnCtl);
            break;
        case 4:
            writeGPIO(uart, mcuPcCtl);
            break;
        case 5:
            writeGPIO(uart, mcuDcCtl);
            break;
        case 6:
            writeGPIO(uart, mcuContCtl);
            break;

        default:
            uart.printf("Invalid option: %d\n\r", cmd);
        }

        uart.printf("\n\r");
    }

    return 0;
}
