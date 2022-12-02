/******************************************************************************/
/* MAX22530      Programming Guide Functions                     */
/******************************************************************************/

#include <stdint.h>
#include <EVT/io/SPI.hpp>
#include <PreCharge/PreCharge.hpp>


namespace PreCharge {

class MAX22530{
public:
    /*Constructor for MAX22530 object.*/
    MAX22530(IO::SPI& SPI);
    /**
     *Reads the voltage values of the device.
     * @return uint16_t of the values read from the device which transmission was started with.
     */
    uint8_t ReadVoltage();

    /******************************************************************************/
/***	Global Variables, Declarations				       ***/
/******************************************************************************/
private:
    IO::SPI& spi;
    uint8_t bytes[2];
};
}