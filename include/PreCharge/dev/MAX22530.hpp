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
    /* Reads the value of the selected register. */
    uint8_t ReadVoltage();



/******************************************************************************/
/***	Global Variables, Declarations				       ***/
/******************************************************************************/
private:
    IO::SPI& spi;
};
}