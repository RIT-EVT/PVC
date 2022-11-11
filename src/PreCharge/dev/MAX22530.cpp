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

    uint8_t MAX22530ReadVoltage(uint8_t regAddress) {
        uint16_t result = 0;
        spi.startTransmission(regAddress);
        result = spi.read();
        spi.endTransmission(regAddress);
        return result;
    }



}