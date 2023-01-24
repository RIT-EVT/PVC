#include <cstdint>
#include <EVT/io/SPI.hpp>
#include <PreCharge/PreCharge.hpp>

namespace PreCharge {

/**
 * Handles reading the voltage values & converting it to decivolts from the
 * MAX22530.
 */
class MAX22530{
public:
    /**
     * Creates a new MAX22530 which will read a raw ADC voltage and convert it
     * to decivolts.
     * @param[in] spi
     */
    explicit MAX22530(IO::SPI& spi);

    /**
     * Returns the voltage in decivolts
     *
     * @return The conversion of the ADC value into temperature
     */
    uint8_t readVoltage(uint16_t reg);

private:
    /// The SPI interface to read from
    IO::SPI& spi;
    /// Function that converts raw ADC values into decivolts
    static uint8_t convertToVoltage(uint16_t count);
};

}