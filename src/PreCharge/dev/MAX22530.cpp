#include <PreCharge/dev/MAX22530.hpp>

#include <EVT/io/SPI.hpp>

namespace PreCharge {

MAX22530::MAX22530(IO::SPI& SPI) : spi(SPI) {}

uint8_t MAX22530::convertToVoltage(uint16_t count) {     // count is out of 4096
    uint8_t result = (((uint32_t) count) * 1098) / 40960;// 1.8 * 61 (* 10 for 1.8 to make it fix point) 1.8 is the reference voltage, 61 is voltage divider ratio
    return result;                                       // returns in dV
}

uint8_t MAX22530::readVoltage(uint16_t reg) {
    uint8_t bytes[2];
    spi.readReg(0, reg, bytes, 2);
    return convertToVoltage((bytes[0] << 8) | bytes[1]);
}

}// namespace PreCharge