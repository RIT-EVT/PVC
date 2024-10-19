/**
* This is the primary State machine handler for the pre-charge voltage controller (PVC) board
*/

#include <EVT/io/UART.hpp>
#include <EVT/manager.hpp>

#include <PreCharge/GFDB.hpp>
#include <PreCharge/PreCharge.hpp>

namespace IO = EVT::core::IO;
namespace DEV = EVT::core::DEV;
namespace time = EVT::core::time;

int main() {
    // Initialize system
    EVT::core::platform::init();

    IO::ADC& dcr = IO::getADC<PreCharge::PreCharge::DCR_IN>();
    DEV::Thermistor thermistor {dcr, PreCharge::PreCharge::solveForTemp};
    IO::UART& uart = IO::getUART<PreCharge::PreCharge::UART_TX_PIN, PreCharge::PreCharge::UART_RX_PIN>(9600, false);

    uart.printf("Starting thermistor test...");
    while (1) {
        uart.printf("ADC: %d, Thermistor temp: %dmC", static_cast<int>(thermistor.getRawADC()), static_cast<int>(thermistor.getTempCelcius()));
        time::wait(100);
    }
}