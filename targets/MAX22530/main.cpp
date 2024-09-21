/**
 * This example prints out voltage reading from the MAX22530 ADC
 */

#include "PVC/PVC.hpp"
#include <EVT/io/UART.hpp>
#include <EVT/io/pin.hpp>
#include <EVT/manager.hpp>
#include <EVT/utils/log.hpp>
#include <PVC/dev/MAX22530.hpp>

namespace IO = EVT::core::IO;
namespace DEV = EVT::core::DEV;
namespace time = EVT::core::time;

constexpr uint32_t SPI_SPEED = SPI_SPEED_125KHZ;
constexpr uint8_t deviceCount = 1;

IO::GPIO* devices[deviceCount];

int main() {
    // Initialize system
    EVT::core::platform::init();

    // Setup IO
    IO::UART& uart = IO::getUART<PVC::PVC::UART_TX_PIN, PVC::PVC::UART_RX_PIN>(9600, true);

    // Setup SPI
    devices[0] = &IO::getGPIO<PVC::PVC::SPI_CS>(EVT::core::IO::GPIO::Direction::OUTPUT);
    devices[0]->writePin(IO::GPIO::State::HIGH);
    IO::SPI& spi = IO::getSPI<PVC::PVC::SPI_SCK, PVC::PVC::SPI_MOSI, PVC::PVC::SPI_MISO>(devices, deviceCount);
    spi.configureSPI(SPI_SPEED, SPI_MODE0, SPI_MSB_FIRST);

    PVC::MAX22530 MAX(spi);

    while (1) {
        for (int reg = 0x01; reg <= 0x04; reg++) {
            uart.printf("Register 0x%x: %d\r\n", reg, MAX.readVoltage(reg));
        }
    }
}
