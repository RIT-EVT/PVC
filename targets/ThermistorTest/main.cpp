/**
* This is the primary State machine handler for the pre-charge voltage controller (PVC) board
*/

#include <core/dev/Thermistor.hpp>
#include <core/io/UART.hpp>
#include <core/manager.hpp>

#include <PVC/GFDB.hpp>
#include <PVC/PVC.hpp>

namespace IO = core::io;
namespace DEV = core::dev;
namespace time = core::time;

int main() {
    // Initialize system
    core::platform::init();

    IO::ADC& dcr = IO::getADC<PVC::PVC::DCR_IN>();
    DEV::Thermistor thermistor{dcr, PVC::PVC::solveForTemp};
    IO::UART& uart = IO::getUART<PVC::PVC::UART_TX_PIN, PVC::PVC::UART_RX_PIN>(9600, false);

    uart.printf("Starting thermistor test...");
    while (1) {
        uart.printf("ADC: %d, Thermistor temp: %dmC", static_cast<int>(thermistor.getRawADC()), static_cast<int>(thermistor.getTempCelcius()));
        time::wait(100);
    }
}