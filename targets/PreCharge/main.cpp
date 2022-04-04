/**
 * This is the primary State machine handler for the pre-charge board
 */
#include <EVT/io/CANopen.hpp>
#include <EVT/io/UART.hpp>
#include <EVT/io/manager.hpp>
#include <EVT/io/pin.hpp>
#include <EVT/io/types/CANMessage.hpp>
#include <EVT/utils/log.hpp>
#include <EVT/utils/types/FixedQueue.hpp>

#include <PreCharge/PreCharge.hpp>
#include <EVT/dev/platform/f3xx/f302x8/Timerf302x8.hpp>

namespace IO = EVT::core::IO;
namespace DEV = EVT::core::DEV;
namespace time = EVT::core::time;
namespace log = EVT::core::log;

///////////////////////////////////////////////////////////////////////////////
// EVT-core CAN callback and CAN setup. This will include logic to set
// aside CANopen messages into a specific queue
///////////////////////////////////////////////////////////////////////////////

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
    EVT::core::types::FixedQueue<CANOPEN_QUEUE_SIZE, IO::CANMessage>* queue =
        (EVT::core::types::FixedQueue<CANOPEN_QUEUE_SIZE, IO::CANMessage>*) priv;
    if (queue != nullptr)
        queue->append(message);
}

///////////////////////////////////////////////////////////////////////////////
// CANopen specific Callbacks. Need to be defined in some location
///////////////////////////////////////////////////////////////////////////////
extern "C" void CONodeFatalError(void) {
    log::LOGGER.log(log::Logger::LogLevel::ERROR, "Fatal CANopen error");
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
    IO::init();

    // Set up pre_charge
    IO::GPIO& key = IO::getGPIO<IO::Pin::PA_1>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryOne = IO::getGPIO<IO::Pin::PA_10>(IO::GPIO::Direction::INPUT);
    IO::GPIO& batteryTwo = IO::getGPIO<IO::Pin::PA_9>(IO::GPIO::Direction::INPUT);
    IO::GPIO& eStop = IO::getGPIO<IO::Pin::PA_0>(IO::GPIO::Direction::INPUT);
    IO::GPIO& pc = IO::getGPIO<IO::Pin::PA_4>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& dc = IO::getGPIO<IO::Pin::PA_5>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& cont = IO::getGPIO<IO::Pin::PA_8>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& apm = IO::getGPIO<IO::Pin::PA_2>(IO::GPIO::Direction::OUTPUT);
    IO::GPIO& forward = IO::getGPIO<IO::Pin::PA_3>(IO::GPIO::Direction::OUTPUT);
    PreCharge::PreCharge precharge(key, batteryOne, batteryTwo, eStop, pc, dc, cont, apm, forward);

    // Queue that will store CANopen messages
    EVT::core::types::FixedQueue<CANOPEN_QUEUE_SIZE, IO::CANMessage> canOpenQueue;

    // Initialize CAN, add an IRQ that will populate the above queue
    IO::CAN& can = IO::getCAN<IO::Pin::PA_12, IO::Pin::PA_11>();
    can.addIRQHandler(canInterruptHandler, reinterpret_cast<void*>(&canOpenQueue));

    // Initialize the timer
    DEV::Timerf302x8 timer(TIM2, 100);

    // Set up Logger
    IO::UART& uart = IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600);
    log::LOGGER.setUART(&uart);
    log::LOGGER.setLogLevel(log::Logger::LogLevel::DEBUG);
    log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Logger initialized.");
    timer.stopTimer();

    // Reserved memory for CANopen stack usage
    uint8_t sdoBuffer[1][CO_SDO_BUF_BYTE];
    CO_TMR_MEM appTmrMem[4];

    // Attempt to join the CAN network
    IO::CAN::CANStatus result = can.connect();

    if (result != IO::CAN::CANStatus::OK) {
        EVT::core::log::LOGGER.log(log::Logger::LogLevel::ERROR, "Failed to connect to CAN network");
        return 1;
    }

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
        .NodeId = pre_charge::pre_charge::NODE_ID,
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

    EVT::core::log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Error: %d", CONodeGetErr(&canNode));

    while (1) {
        precharge.handle();//update state machine

        // Process incoming CAN messages
        CONodeProcess(&canNode);
        // Update the state of timer based events
        COTmrService(&canNode.Tmr);
        // Handle executing timer events that have elapsed
        COTmrProcess(&canNode.Tmr);

        time::wait(10);// May not be necessary
    }
}
