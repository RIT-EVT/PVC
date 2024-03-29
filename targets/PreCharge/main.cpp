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
#include <PreCharge/PreCharge.hpp>

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

extern "C" void COIfCanReceive(CO_IF_FRM* frm) {}

extern "C" int16_t COLssStore(uint32_t baudrate, uint8_t nodeId) { return 0; }

extern "C" int16_t COLssLoad(uint32_t* baudrate, uint8_t* nodeId) { return 0; }

extern "C" void CONmtModeChange(CO_NMT* nmt, CO_MODE mode) {}

extern "C" void CONmtHbConsEvent(CO_NMT* nmt, uint8_t nodeId) {}

extern "C" void CONmtHbConsChange(CO_NMT* nmt, uint8_t nodeId, CO_MODE mode) {}

extern "C" int16_t COParaDefault(CO_PARA* pg) { return 0; }

extern "C" void COPdoTransmit(CO_IF_FRM* frm) {}

extern "C" int16_t COPdoReceive(CO_IF_FRM* frm) { return 0; }

extern "C" void COPdoSyncUpdate(CO_RPDO* pdo) {}

extern "C" void COTmrLock(void) {}

extern "C" void COTmrUnlock(void) {}

int main() {
    // Initialize system
    EVT::core::platform::init();

    // Queue that will store CANopen messages
    EVT::core::types::FixedQueue<CANOPEN_QUEUE_SIZE, IO::CANMessage> canOpenQueue;

    // Initialize CAN, add an IRQ that will populate the above queue
    IO::CAN& can = IO::getCAN<PreCharge::PreCharge::CAN_TX_PIN, PreCharge::PreCharge::CAN_RX_PIN>();
    struct CANInterruptParams canParams = {
        .queue = &canOpenQueue,
        .can = static_cast<IO::CANf3xx*>(&can),
    };
    can.addIRQHandler(canInterruptHandler, reinterpret_cast<void*>(&canParams));

    // Initialize MAX22530 and SPI
    IO::GPIO* CSPins[1];
    CSPins[0] = &IO::getGPIO<PreCharge::PreCharge::SPI_CS>(IO::GPIO::Direction::OUTPUT);
    CSPins[0]->writePin(IO::GPIO::State::HIGH);
    IO::SPI& spi = IO::getSPI<PreCharge::PreCharge::SPI_SCK, PreCharge::PreCharge::SPI_MOSI, PreCharge::PreCharge::SPI_MISO>(CSPins, 1);
    spi.configureSPI(SPI_SPEED_125KHZ, SPI_MODE0, SPI_MSB_FIRST);
    PreCharge::MAX22530 MAX(spi);

    // Initialize the timer
    DEV::Timer& timer = DEV::getTimer<DEV::MCUTimer::Timer2>(100);

    // Set up Logger
    IO::UART& uart = IO::getUART<PreCharge::PreCharge::UART_TX_PIN, PreCharge::PreCharge::UART_RX_PIN>(9600, true);
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
    IO::GPIO& key = IO::getGPIO<PreCharge::PreCharge::KEY_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryOne = IO::getGPIO<PreCharge::PreCharge::BAT_OK_1_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryTwo = IO::getGPIO<PreCharge::PreCharge::BAT_OK_2_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& eStop = IO::getGPIO<PreCharge::PreCharge::ESTOP_IN_PIN>(IO::GPIO::Direction::INPUT);
    IO::GPIO& pc = IO::getGPIO<PreCharge::PreCharge::PC_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& dc = IO::getGPIO<PreCharge::PreCharge::DC_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    PreCharge::Contactor cont(
        IO::getGPIO<PreCharge::PreCharge::CONT1_PIN>(IO::GPIO::Direction::OUTPUT),
        IO::getGPIO<PreCharge::PreCharge::CONT2_PIN>(IO::GPIO::Direction::OUTPUT));
    IO::GPIO& apm = IO::getGPIO<PreCharge::PreCharge::APM_CTL_PIN>(IO::GPIO::Direction::OUTPUT);
    //    IO::GPIO& forward = IO::getGPIO<IO::Pin::PA_3>(IO::GPIO::Direction::OUTPUT);
    GFDB::GFDB gfdb(can);
    PreCharge::PreCharge precharge(key, batteryOne, batteryTwo, eStop, pc, dc, cont, apm, gfdb, can, MAX);

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
        .NodeId = PreCharge::PreCharge::NODE_ID,
        .Baudrate = IO::CAN::DEFAULT_BAUD,
        .Dict = precharge.getObjectDictionary(),
        .DictLen = precharge.getObjectDictionarySize(),
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

    PreCharge::PreCharge::PVCStatus status = PreCharge::PreCharge::PVCStatus::PVC_OK;

    while (1) {
        PreCharge::PreCharge::PVCStatus current_status = precharge.handle(uart);// Update state machine

        if (current_status == PreCharge::PreCharge::PVCStatus::PVC_ERROR) {
            status = PreCharge::PreCharge::PVCStatus::PVC_ERROR;
        } else if (status == PreCharge::PreCharge::PVCStatus::PVC_ERROR && current_status == PreCharge::PreCharge::PVCStatus::PVC_OK) {
            status = PreCharge::PreCharge::PVCStatus::PVC_OK;
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