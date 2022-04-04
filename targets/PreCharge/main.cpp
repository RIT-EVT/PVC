/**
 * This is the primary State machine handler for the pre-charge board
 */
#include <EVT/io/manager.hpp>
#include <EVT/io/pin.hpp>

#include <PreCharge/PreCharge.hpp>

namespace IO = EVT::core::IO;

int main() {
    // Initialize system
    IO::init();

    // Set up pre_charge
    IO::GPIO& key = IO::getGPIO<IO::Pin::PA_1>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryOne = IO::getGPIO<IO::Pin::PA_10>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryTwo = IO::getGPIO<IO::Pin::PA_9>(IO::GPIO::Direction::INPUT);
    IO::GPIO& eStop = IO::getGPIO<IO::Pin::PA_0>(IO::GPIO::Direction::INPUT);
    IO::GPIO& pc = IO::getGPIO<IO::Pin::PA_4>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& dc = IO::getGPIO<IO::Pin::PA_5>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& cont = IO::getGPIO<IO::Pin::PA_8>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& apm = IO::getGPIO<IO::Pin::PA_2>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& forward = IO::getGPIO<IO::Pin::PA_3>(IO::GPIO::Direction::OUTPUT);
    PreCharge::PreCharge precharge(key, batteryOne, batteryTwo, eStop, pc, dc, cont, apm, forward);

    while (1) {
        precharge.handle();//update state machine
    }
}
