#include <PreCharge/dev/MAX22530.hpp>

#include <EVT/io/SPI.hpp>

namespace PreCharge {

    MAX22530::MAX22530(IO::SPI& SPI) : spi(SPI){spi.startTransmission(0);}

    uint8_t MAX22530::ReadVoltage() {
        uint16_t result;
        spi.read(bytes, 2);
        result = bytes[0]<<8;
        result|=bytes[1];
        return result;
    }
}