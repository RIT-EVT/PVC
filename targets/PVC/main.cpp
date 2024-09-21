/**
* This is the primary State machine handler for the pre-charge voltage controller (PVC) board
*/

#include <EVT/io/CANopen.hpp>
#include <EVT/io/UART.hpp>
#include <EVT/io/pin.hpp>
#include <EVT/io/types/CANMessage.hpp>
#include <EVT/manager.hpp>
#include <EVT/utils/log.hpp>
#include <EVT/utils/types/FixedQueue.hpp>

#include "PVC/PVC.hpp"
#include <PVC/GFDB.hpp>

namespace IO = EVT::core::IO;
namespace DEV = EVT::core::DEV;
namespace time = EVT::core::time;

///////////////////////////////////////////////////////////////////////////////
// EVT-core CAN callback and CAN setup. This will include logic to set
// aside CANopen messages into a specific queue
///////////////////////////////////////////////////////////////////////////////

struct CANInterruptParams {
    EVT::core::types::FixedQueue<CANOPEN_QUEUE_SIZE, IO::CANMessage>* queue;
    IO::CANf3xx* can;
};

/**
* Interrupt handler to get CAN messages. A function pointer to this function
* will be passed to the EVT-core CAN interface which will in turn call this
* function each time a new CAN message comes in.
*
* NOTE: For this sample, every non-extended (so 11 bit CAN IDs) will be
* assummed to be intended to be passed as a CANopen message.
*
* @param message[in] The passed in CAN message that was read.
*/
void canInterruptHandler(IO::CANMessage& message, void* priv) {
    auto* canStruct = (struct CANInterruptParams*) priv;
    if (message.isCANExtended()) {
        if (canStruct != nullptr && canStruct->can != nullptr) {
            canStruct->can->addCANMessage(message);
        }
    } else {
        if (canStruct != nullptr && canStruct->queue != nullptr) {
            canStruct->queue->append(message);
        }
    }
}

int main() {
    // Initialize system
    EVT::core::platform::init();

    // Queue that will store CANopen messages
    EVT::core::types::FixedQueue<CANOPEN_QUEUE_SIZE, IO::CANMessage> canOpenQueue;

    // Initialize CAN, add an IRQ that will populate the above queue
    IO::CAN& can = IO::getCAN<PVC::PVC::CAN_TX_PIN, PVC::PVC::CAN_RX_PIN>();
    struct CANInterruptParams canParams = {
        .queue = &canOpenQueue,
        .can = static_cast<IO::CANf3xx*>(&can),
    };
    can.addIRQHandler(canInterruptHandler, reinterpret_cast<void*>(&canParams));

    // Initialize MAX22530 and SPI
    IO::GPIO* CSPins[1];
    CSPins[0] = &IO::getGPIO<PVC::PVC::SPI_CS>(IO::GPIO::Direction::OUTPUT);
    CSPins[0]->writePin(IO::GPIO::State::HIGH);
    IO::SPI& spi = IO::getSPI<PVC::PVC::SPI_SCK, PVC::PVC::SPI_MOSI, PVC::PVC::SPI_MISO>(CSPins, 1);
    spi.configureSPI(SPI_SPEED_125KHZ, SPI_MODE0, SPI_MSB_FIRST);
    PVC::MAX22530 MAX(spi);

    // Initialize the timer
    DEV::Timer& timer = DEV::getTimer<DEV::MCUTimer::Timer2>(100);

    // Set up Logger
    IO::UART& uart = IO::getUART<PVC::PVC::UART_TX_PIN, PVC::PVC::UART_RX_PIN>(9600, true);
    EVT::core::log::LOGGER.setUART(&uart);
    EVT::core::log::LOGGER.setLogLevel(EVT::core::log::Logger::LogLevel::DEBUG);
    EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "Logger initialized.");
    timer.stopTimer();

    // Reserved memory for CANopen stack usage
    uint8_t sdoBuffer[CO_SSDO_N * CO_SDO_BUF_BYTE];
    CO_TMR_MEM appTmrMem[16];

    // Attempt to join the CAN network
    IO::CAN::CANStatus result = can.connect();

    if (result != IO::CAN::CANStatus::OK) {
        EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Failed to connect to CAN network");
        return 1;
    }

    // Set up pre_charge
    IO::GPIO& key = IO::getGPIO<PVC::PVC::KEY_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryOne = IO::getGPIO<PVC::PVC::BAT_OK_1_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryTwo = IO::getGPIO<PVC::PVC::BAT_OK_2_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& eStop = IO::getGPIO<PVC::PVC::ESTOP_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& pc = IO::getGPIO<PVC::PVC::PC_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& dc = IO::getGPIO<PVC::PVC::DC_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    PVC::Contactor cont(
        IO::getGPIO<PVC::PVC::CONT1_PIN>(IO::GPIO::Direction::OUTPUT),
        IO::getGPIO<PVC::PVC::CONT2_PIN>(IO::GPIO::Direction::OUTPUT));
    IO::GPIO& apm = IO::getGPIO<PVC::PVC::APM_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    //    IO::GPIO& forward = IO::getGPIO<IO::Pin::PA_3>(IO::GPIO::Direction::OUTPUT);
    GFDB::GFDB gfdb(can);
    PVC::PVC precharge(key, batteryOne, batteryTwo, eStop, pc, dc, cont, apm, gfdb, can, MAX);

    ///////////////////////////////////////////////////////////////////////////
    // Setup CAN configuration, this handles making drivers, applying settings.
    // And generally creating the CANopen stack node which is the interface
    // between the application (the code we write) and the physical CAN network
    ///////////////////////////////////////////////////////////////////////////
    // Make drivers

    // Reserve driver variables
    CO_IF_DRV canStackDriver;

    CO_IF_CAN_DRV canDriver;
    CO_IF_TIMER_DRV timerDriver;
    CO_IF_NVM_DRV nvmDriver;

    CO_NODE canNode;

    // Initialize all the CANOpen drivers.
    IO::initializeCANopenDriver(&canOpenQueue, &can, &timer, &canStackDriver, &nvmDriver, &timerDriver, &canDriver);

    // Initialize the CANOpen node we are using.
    IO::initializeCANopenNode(&canNode, &precharge, &canStackDriver, sdoBuffer, appTmrMem);

    // Set the node to operational mode
    CONmtSetMode(&canNode.Nmt, CO_OPERATIONAL);

    time::wait(500);

    // Start with everything at 0
    apm.writePin(IO::GPIO::State::LOW);
    //    forward.writePin(IO::GPIO::State::LOW);
    pc.writePin(IO::GPIO::State::LOW);
    dc.writePin(IO::GPIO::State::LOW);

    while (1) {
        PVC::PVC::PVCStatus current_status = precharge.handle();// Update state machine

        // Process incoming CAN messages
        CONodeProcess(&canNode);
        // Update the state of timer based events
        COTmrService(&canNode.Tmr);
        // Handle executing timer events that have elapsed
        COTmrProcess(&canNode.Tmr);

        time::wait(100);// May not be necessary
    }
}