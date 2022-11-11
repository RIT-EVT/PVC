#include <PreCharge/dev/MAX22530.hpp>

#include <EVT/io/SPI.hpp>
/**
      @brief Performs reading any given register.
       @param regAddress - The address of the register to read.
       @return data - Register data
       0x10000 – CRC mismatch occured
 */

namespace PreCharge {

    MAX22530::MAX22530(IO::SPI& SPI) : spi(spi){}

    uint8_t ReadVoltage() {
        uint16_t result = 0;
        spi.startTransmission(0);
        result = spi.read();
        spi.endTransmission(0);
        return result;
    }
}