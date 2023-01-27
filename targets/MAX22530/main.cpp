/**
 * This example prints out voltage reading from the MAX22530 ADC converter
 */

#include <EVT/io/UART.hpp>
#include <EVT/io/manager.hpp>
#include <EVT/io/pin.hpp>
#include <EVT/utils/log.hpp>
#include <PreCharge/dev/MAX22530.hpp>

namespace IO = EVT::core::IO;
namespace DEV = EVT::core::DEV;
namespace time = EVT::core::time;

constexpr uint32_t SPI_SPEED = SPI_SPEED_125KHZ;
constexpr uint8_t deviceCount = 1;

IO::GPIO* devices[deviceCount];

int main() {
    // Setup IO
    IO::UART& uart = IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600);

    // Setup SPI
    devices[0] = &IO::getGPIO<IO::Pin::SPI_CS>(EVT::core::IO::GPIO::Direction::OUTPUT);
    devices[0]->writePin(IO::GPIO::State::HIGH);
    IO::SPI& spi = IO::getSPI<IO::Pin::SPI_SCK, IO::Pin::SPI_MOSI, IO::Pin::SPI_MISO>(devices, deviceCount);
    spi.configureSPI(SPI_SPEED, SPI_MODE0, SPI_MSB_FIRST);

    PreCharge::MAX22530 MAX(spi);

    while (1) {
        for (int reg = 0x01; reg <= 0x04; reg++) {
            uart.printf("Register 0x%x: %d\r\n", reg, MAX.readVoltage(reg << 2));
        }
    }
}
