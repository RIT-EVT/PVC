/**
 * This is the primary State machine handler for the pre-charge board
 */
#include <EVT/io/manager.hpp>
#include <EVT/io/pin.hpp>

#include <pre-charge/pre_charge.hpp>

namespace IO = EVT::core::IO;

int main() {
    // Initialize system
    IO::init();
    IO::UART& uart = IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600);

    uart.printf("-----TESTING PRE-CHARGE STATE MACHINE-----\n\r");

    // Set up pre_charge
    IO::GPIO& key           = IO::getGPIO<IO::Pin::PA_0>(IO::GPIO::Direction::INPUT);
    IO::GPIO& pc            = IO::getGPIO<IO::Pin::PA_3>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& dc            = IO::getGPIO<IO::Pin::PA_4>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& cont          = IO::getGPIO<IO::Pin::PA_5>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& batteryOne    = IO::getGPIO<IO::Pin::PA_1>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryTwo    = IO::getGPIO<IO::Pin::PA_2>(IO::GPIO::Direction::INPUT);
    IO::GPIO& eStop         = IO::getGPIO<IO::Pin::PA_6>(IO::GPIO::Direction::INPUT);
    pre_charge::pre_charge precharge(key, pc, dc, cont, batteryOne, batteryTwo, eStop);

    uart.printf("pre_charge instantiated\n\r");

    while (1) {
        precharge.handle(); //update state machine

        uart.printf(precharge.printState().c_str()); //print status
    }
}
