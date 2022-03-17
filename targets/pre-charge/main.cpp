/**
 * This is the primary State machine handler for the pre-charge board
 */
#include <EVT/io/manager.hpp>
#include <EVT/io/pin.hpp>

namespace IO = EVT::core::IO;

int main() {
    // Initialize system
    IO::init();

    // Set up pre_charge and STO handler
    pre_charge::pre_charge precharge = pre_charge::pre_charge(IO::getGPIO<IO::Pin::PA_0>(),
                                                              IO::getGPIO<IO::Pin::PA_3>(),
                                                              IO::getGPIO<IO::Pin::PA_4>(),
                                                              IO::getGPIO<IO::Pin::PA_5>(),
                                                              IO::getGPIO<IO::Pin::PA_1>(),
                                                              IO::getGPIO<IO::Pin::PA_2>(),
                                                              IO::getGPIO<IO::Pin::PA_6>());

    while (1) {
        precharge.handle(); //update state machine
    }
}
