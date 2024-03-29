#pragma once
#include <EVT/io/SPI.hpp>
#include <cstdint>

namespace IO = EVT::core::IO;

namespace PreCharge {

/**
 * Handles reading voltage data from the MAX22530 ADC
 */
class MAX22530 {
public:
    /**
     * Creates a new MAX22530 which will read a raw ADC voltage and convert it
     * to decivolts.
     *
     * @param[in] spi The SPI to use to communicate with the MAX22530
     */
    explicit MAX22530(IO::SPI& spi);

    /**
     * Returns the voltage in decivolts
     *
     * @return The voltage (decivolts)
     */
    uint8_t readVoltage(uint16_t reg);

private:
    /** The SPI interface to read from */
    IO::SPI& spi;
    /** Function that converts raw ADC values into decivolts */
    static uint8_t convertToVoltage(uint16_t count);
};

}// namespace PreCharge
