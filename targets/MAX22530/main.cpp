/**
* This is the primary State machine handler for the pre-charge board
*/
#include <EVT/io/UART.hpp>
#include <EVT/io/manager.hpp>
#include <EVT/io/pin.hpp>
#include <EVT/utils/log.hpp>
#include <PreCharge/dev/MAX22530.hpp>

#define PROD_ID           0x00
#define ADC1              0x01
#define ADC2              0x02
#define ADC3              0x03
#define ADC4              0x04

namespace IO = EVT::core::IO;
namespace DEV = EVT::core::DEV;
namespace time = EVT::core::time;

constexpr uint32_t SPI_SPEED = SPI_SPEED_500KHZ;//
constexpr uint8_t deviceCount = 1;

IO::GPIO* devices[deviceCount];

int main() {
   IO::UART& uart = IO::getUART<IO::Pin::PB_6, IO::Pin::PB_7>(9600);
   devices[0] = &IO::getGPIO<IO::Pin::PB_12>(EVT::core::IO::GPIO::Direction::OUTPUT);
   devices[0]->writePin(IO::GPIO::State::HIGH);

   // Setup SPI
   IO::SPI& spi = IO::getSPI<IO::Pin::SPI_SCK, IO::Pin::SPI_MOSI, IO::Pin::SPI_MISO>(devices, deviceCount);
   spi.configureSPI(SPI_SPEED, SPI_MODE0, SPI_MSB_FIRST);

   PreCharge::MAX22530 MAX = PreCharge::MAX22530(spi);

   while(1){
       uart.printf("Voltage : %s",MAX.ReadVoltage());
       time::wait(500);
   }
}
