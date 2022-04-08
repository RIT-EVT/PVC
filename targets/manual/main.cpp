#include <string.h>

#include <EVT/io/UART.hpp>
#include <EVT/io/GPIO.hpp>
#include <EVT/io/manager.hpp>

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

    if(state != 0 && state != 1) {
        uart.printf("INVALID STATE: %d\n", state);
        return;
    }

    gpio.writePin(state == 0 ? IO::GPIO::State::LOW : IO::GPIO::State::HIGH);
}


// 4 digital inputs
// 1 ADC input
// 5 outputs

int main() {
    char inputBuffer[MAX_BUFF];

    IO::UART& uart = IO::getUART<IO::Pin::PB_6, IO::Pin::PB_7>(9600);

    // Outputs
    IO::GPIO& mcuAPMCtl = IO::getGPIO<IO::Pin::PA_2>();
    IO::GPIO& mcuFWEnCtl = IO::getGPIO<IO::Pin::PA_3>();
    IO::GPIO& mcuPcCtl = IO::getGPIO<IO::Pin::PA_4>();
    IO::GPIO& mcuDcCtl = IO::getGPIO<IO::Pin::PA_5>();
    IO::GPIO& mcuContCtl = IO::getGPIO<IO::Pin::PA_8>();

    // Inputs
    IO::GPIO& mcuEStopStatus = IO::getGPIO<IO::Pin::PA_0>(IO::GPIO::Direction::INPUT);
    IO::GPIO& keyInMcu = IO::getGPIO<IO::Pin::PA_1>(IO::GPIO::Direction::INPUT);
    IO::GPIO& battery1Ok = IO::getGPIO<IO::Pin::PA_10>(IO::GPIO::Direction::INPUT);
    IO::GPIO& battery2Ok = IO::getGPIO<IO::Pin::PA_9>(IO::GPIO::Direction::INPUT);

    // Start with everything at 0
    mcuAPMCtl.writePin(IO::GPIO::State::LOW);
    mcuFWEnCtl.writePin(IO::GPIO::State::LOW);
    mcuPcCtl.writePin(IO::GPIO::State::LOW);
    mcuDcCtl.writePin(IO::GPIO::State::LOW);
    mcuContCtl.writePin(IO::GPIO::State::LOW);

    while(true) {
        printHelp(uart);

        uart.printf("Enter selection: ");
        uart.gets(inputBuffer, MAX_BUFF);
        uart.printf("\r\n");

        uint8_t cmd = strtol(inputBuffer, nullptr, 10);

        switch(cmd) {
            case 1:
                uart.printf("Input Values:\n\r");
                uart.printf("\tMCU_ESTOP_STATUS: %d\n\r", mcuEStopStatus.readPin());
                uart.printf("\tKEY_IN_MCU: %d\n\r", keyInMcu.readPin());
                uart.printf("\tBATTERY_1_OK_MCU: %d\n\r", battery1Ok.readPin());
                uart.printf("\tBATTERY_2_OK_MCU: %d\n\r", battery2Ok.readPin());
                break;
            case 2:
                writeGPIO(uart, mcuAPMCtl);
                break;
            case 3:
                writeGPIO(uart, mcuFWEnCtl);
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
