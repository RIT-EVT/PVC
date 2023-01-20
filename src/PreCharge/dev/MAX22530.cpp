#include <PreCharge/dev/MAX22530.hpp>

#include <EVT/io/SPI.hpp>

namespace PreCharge {

    MAX22530::MAX22530(IO::SPI& SPI) : spi(SPI) {

    }

    uint8_t MAX22530::convertToVoltage(uint16_t count) {
        uint8_t result = ( ((uint32_t) count) * 1098) / 40960; //explain: 1.8 * 61 (* 10 for 1.8)
        return result;
    }

    uint8_t MAX22530::readVoltage(uint16_t reg) {
        uint16_t result;
        spi.readReg(0, reg, bytes, 2);
        result = bytes[0]<<8;
        result |= bytes[1];
        return convertToVoltage(result);
    }

}