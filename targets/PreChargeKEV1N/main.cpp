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

#include <PreCharge/GFDB.hpp>
#include <PreCharge/PreChargeKEV1N.hpp>

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

///////////////////////////////////////////////////////////////////////////////
// CANopen specific Callbacks. Need to be defined in some location
///////////////////////////////////////////////////////////////////////////////
extern "C" void CONodeFatalError(void) {
    EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Fatal CANopen error");
}

int main() {
    // Initialize system
    EVT::core::platform::init();

    // Queue that will store CANopen messages
    EVT::core::types::FixedQueue<CANOPEN_QUEUE_SIZE, IO::CANMessage> canOpenQueue;

    // Initialize CAN, add an IRQ that will populate the above queue
    IO::CAN& can = IO::getCAN<PreCharge::PreChargeKEV1N::CAN_TX_PIN, PreCharge::PreChargeKEV1N::CAN_RX_PIN>();
    struct CANInterruptParams canParams = {
        .queue = &canOpenQueue,
        .can = static_cast<IO::CANf3xx*>(&can),
    };
    can.addIRQHandler(canInterruptHandler, reinterpret_cast<void*>(&canParams));

    // Initialize MAX22530 and SPI
    IO::GPIO* CSPins[1];
    CSPins[0] = &IO::getGPIO<PreCharge::PreChargeKEV1N::SPI_CS>(IO::GPIO::Direction::OUTPUT);
    CSPins[0]->writePin(IO::GPIO::State::HIGH);
    IO::SPI& spi = IO::getSPI<PreCharge::PreChargeKEV1N::SPI_SCK, PreCharge::PreChargeKEV1N::SPI_MOSI, PreCharge::PreChargeKEV1N::SPI_MISO>(CSPins, 1);
    spi.configureSPI(SPI_SPEED_125KHZ, SPI_MODE0, SPI_MSB_FIRST);
    PreCharge::MAX22530 MAX(spi);

    // Initialize the timer
    DEV::Timer& timer = DEV::getTimer<DEV::MCUTimer::Timer2>(100);

    // Set up Logger
    IO::UART& uart = IO::getUART<PreCharge::PreChargeKEV1N::UART_TX_PIN, PreCharge::PreChargeKEV1N::UART_RX_PIN>(9600, true);
    EVT::core::log::LOGGER.setUART(&uart);
    EVT::core::log::LOGGER.setLogLevel(EVT::core::log::Logger::LogLevel::DEBUG);
    EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "Logger initialized.");
    timer.stopTimer();

    // Reserved memory for CANopen stack usage
    uint8_t sdoBuffer[1][CO_SDO_BUF_BYTE];
    CO_TMR_MEM appTmrMem[4];

    // Attempt to join the CAN network
    IO::CAN::CANStatus result = can.connect();

    if (result != IO::CAN::CANStatus::OK) {
        EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Failed to connect to CAN network");
        return 1;
    }

    // Set up pre_charge
    IO::GPIO& key = IO::getGPIO<PreCharge::PreChargeKEV1N::KEY_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryOne = IO::getGPIO<PreCharge::PreChargeKEV1N::BAT_OK_1_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryTwo = IO::getGPIO<PreCharge::PreChargeKEV1N::BAT_OK_2_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& eStop = IO::getGPIO<PreCharge::PreChargeKEV1N::ESTOP_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& pc = IO::getGPIO<PreCharge::PreChargeKEV1N::PC_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& dc = IO::getGPIO<PreCharge::PreChargeKEV1N::DC_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    PreCharge::Contactor cont(
        IO::getGPIO<PreCharge::PreChargeKEV1N::CONT1_PIN>(IO::GPIO::Direction::OUTPUT),
        IO::getGPIO<PreCharge::PreChargeKEV1N::CONT2_PIN>(IO::GPIO::Direction::OUTPUT));
    IO::GPIO& apm = IO::getGPIO<PreCharge::PreChargeKEV1N::APM_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    //    IO::GPIO& forward = IO::getGPIO<IO::Pin::PA_3>(IO::GPIO::Direction::OUTPUT);
    GFDB::GFDB gfdb(can);
    PreCharge::PreChargeKEV1N precharge(key, batteryOne, batteryTwo, eStop, pc, dc, cont, apm, gfdb, can, MAX);

    ///////////////////////////////////////////////////////////////////////////
    // Setup CAN configuration, this handles making drivers, applying settings.
    // And generally creating the CANopen stack node which is the interface
    // between the application (the code we write) and the physical CAN network
    ///////////////////////////////////////////////////////////////////////////
    // Make drivers
    CO_IF_DRV canStackDriver;

    CO_IF_CAN_DRV canDriver;
    CO_IF_TIMER_DRV timerDriver;
    CO_IF_NVM_DRV nvmDriver;

    IO::getCANopenCANDriver(&can, &canOpenQueue, &canDriver);
    IO::getCANopenTimerDriver(&timer, &timerDriver);
    IO::getCANopenNVMDriver(&nvmDriver);

    canStackDriver.Can = &canDriver;
    canStackDriver.Timer = &timerDriver;
    canStackDriver.Nvm = &nvmDriver;

    CO_NODE_SPEC canSpec = {
        .NodeId = PreCharge::PreChargeKEV1N::NODE_ID,
        .Baudrate = IO::CAN::DEFAULT_BAUD,
        .Dict = precharge.getObjectDictionary(),
        .DictLen = precharge.getNumElements(),
        .EmcyCode = nullptr,
        .TmrMem = appTmrMem,
        .TmrNum = 16,
        .TmrFreq = 100,
        .Drv = &canStackDriver,
        .SdoBuf = reinterpret_cast<uint8_t*>(&sdoBuffer[0]),
    };

    CO_NODE canNode;

    CONodeInit(&canNode, &canSpec);
    CONodeStart(&canNode);
    CONmtSetMode(&canNode.Nmt, CO_OPERATIONAL);

    time::wait(500);

    // Start with everything at 0
    apm.writePin(IO::GPIO::State::LOW);
    //    forward.writePin(IO::GPIO::State::LOW);
    pc.writePin(IO::GPIO::State::LOW);
    dc.writePin(IO::GPIO::State::LOW);

    while (1) {
        PreCharge::PreChargeKEV1N::PVCStatus current_status = precharge.handle(uart);// Update state machine

        if (current_status == PreCharge::PreChargeKEV1N::PVCStatus::PVC_PRE_OP) {
            CONmtSetMode(&canNode.Nmt, CO_PREOP);
        } else if (current_status == PreCharge::PreChargeKEV1N::PVCStatus::PVC_OP) {
            CONmtSetMode(&canNode.Nmt, CO_OPERATIONAL);
        }

        // Process incoming CAN messages
        CONodeProcess(&canNode);
        // Update the state of timer based events
        COTmrService(&canNode.Tmr);
        // Handle executing timer events that have elapsed
        COTmrProcess(&canNode.Tmr);

        time::wait(100);// May not be necessary
    }
}