/******************************************************************************/
/* MAX22530      Programming Guide Functions                     */
/******************************************************************************/

#include <stdint.h>
#include <EVT/io/SPI.hpp>
#include <PreCharge/PreCharge.hpp>
/*MAX22530 Registers*/
#define PROD_ID           0x00
#define ADC1              0x01
#define ADC2              0x02
#define ADC3              0x03
#define ADC4              0x04
//#define FADC1             0x05
//#define FADC2             0x06
//#define FADC3             0x07
//#define FADC4             0x08
//#define COUTHI1           0x09
//#define COUTHI2           0x0a
//#define COUTHI3           0x0b
//#define COUTHI4           0x0c
//#define COUTLO1           0x0d
//#define COUTLO2           0x0e
//#define COUTLO3           0x0f
//#define COUTLO4           0x10
//#define COUT_STATUS       0x11
//#define INTERRUPT_STATUS  0x12
//#define INTERRUPT_ENABLE  0x13
//#define CONTROL           0x14
//
//
//#define MAX22530_ID       0x81
//#define VREF              1.80

/*uC SPI + GPI/O settings*/
#define CS_PIN               " "   // Pin of the uC to which the ADC Chip Select pin is connected.
#define CS_PIN_OUT           " "   // defining the pin Mode on uC side
#define CS_LOW               " "   // digital Write of uC CS_PIN LOW
#define CS_HIGH              " "   // digital Write of uC CS_PIN HIGH
#define GPIO1_PIN            " "   // Pin of the uC to which the ADC Hardware EOC pin INTB Ready pin is connected for determining the end of conversion using a polling sequence.
#define GPIO1_PIN_IN         " "   // Defining the pin mode of GPIO1_PIN
#define GPIO1_STATE          " "   // Configuring the GPIO1_PIN as read state
#define SPI_SETTINGS         " "   // Configure the SPI settings on uC side for. ex: // 8 MHz clock, MSB   first, SPI CPOL 0, CPHA 0

/* MAX22530 HARDWARE INTERRUPT PIN-GPO */
#define MAX22530_INTB_RDY_STATE       //GPIO_STATE used by uC

namespace PreCharge {

class MAX22530{
public:
    /*Constructor for MAX22530 object.*/
    MAX22530(IO::SPI& spi);
    /* Reads the value of the selected register. */
    uint8_t MAX22530_read_register(uint8_t regAddress);
    /* Returns converted binary value to Voltage of ADCx, FADCx, COUTx HI and LO Threshold registers*/
    float Convert_to_Voltage(uint8_t regAddress);


/******************************************************************************/
/***	Global Variables, Declarations				       ***/
/******************************************************************************/
private:
    IO::SPI& spi;
};
}