#include <PreCharge/PreChargeKEV1N.hpp>

#include <EVT/utils/log.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreChargeKEV1N::PreChargeKEV1N(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
                               IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, Contactor cont,
                               IO::GPIO& apm, GFDB::GFDB& gfdb, IO::CAN& can, MAX22530 MAX) : PreChargeBase(key,
                                                                                                            batteryOne,
                                                                                                            batteryTwo,
                                                                                                            eStop,
                                                                                                            pc,
                                                                                                            dc,
                                                                                                            cont,
                                                                                                            apm,
                                                                                                            gfdb,
                                                                                                            can,
                                                                                                            MAX) {
    pre_charged = 2;
}

PreChargeKEV1N::PVCStatus PreChargeKEV1N::handle(IO::UART& uart) {
    getSTO();     //update value of STO
    getMCKey();   //update value of MC_KEY_IN
    getIOStatus();//update value of IOStatus

    Statusword = static_cast<uint8_t>(state);
    InputVoltage = 0; //Does not exist yet
    OutputVoltage = 0;//Does not exist yet
    BasePTemp = 0;    //Does not exist yet

    cyclicPDO = (InputVoltage << 32) | (OutputVoltage << 16) | BasePTemp;

    switch (state) {
    case PreChargeKEV1N::State::MC_OFF:
        mcOffState();
        break;
    case PreChargeKEV1N::State::ESTOPWAIT:
        eStopState();
        break;
    case PreChargeKEV1N::State::PRECHARGE:
        prechargeState();
        break;
    case PreChargeKEV1N::State::CONT_CLOSE:
        contCloseState();
        break;
    case PreChargeKEV1N::State::MC_ON:
        mcOnState();
        break;
    case PreChargeKEV1N::State::CONT_OPEN:
        contOpenState();
        break;
    default:
        break;
    }

    if (pre_charged == 1) {
        return PVCStatus::PVC_OP;
    } else if (pre_charged == 0) {
        return PVCStatus::PVC_PRE_OP;
    } else if (pre_charged == 2) {
        return PVCStatus::PVC_NONE;
    }
}

void PreChargeKEV1N::getSTO() {
    if (in_precharge == 2 && time::millis() - lastPrechargeTime > 5000) {
        uint8_t gfdBuffer;
        IO::CAN::CANStatus gfdbConn = gfdb.requestIsolationState(&gfdBuffer);
        //Error connecting to GFDB
        if (gfdbConn == IO::CAN::CANStatus::OK && (gfdBuffer == 0b00 || gfdBuffer == 0b10)) {
            gfdStatus = 1;
        } else if (gfdBuffer == 0b11) {
            EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Bad GFDB");
            gfdStatus = 0;
        }
    }

    batteryOneOkStatus = batteryOne.readPin();
    batteryTwoOkStatus = batteryTwo.readPin();
    eStopActiveStatus = eStop.readPin();
    voltStatus = MAX.readVoltage(0x02) > MIN_PACK_VOLTAGE;

    if (in_precharge == 2) {
        if (batteryOneOkStatus == IO::GPIO::State::HIGH
            && batteryTwoOkStatus == IO::GPIO::State::HIGH
            && eStopActiveStatus == IO::GPIO::State::HIGH
            && gfdStatus == 1
            && voltStatus == 1) {
            stoStatus = IO::GPIO::State::HIGH;
            numAttemptsMade = 0;
        } else {
            if (numAttemptsMade > MAX_STO_ATTEMPTS) {
                EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Too many fails, error out");
                stoStatus = IO::GPIO::State::LOW;
                numAttemptsMade = 0;
                return;
            }
            EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "1: %d, 2: %d, e: %d, g: %d", batteryOneOkStatus, batteryTwoOkStatus, eStopActiveStatus, gfdStatus);

            numAttemptsMade++;
        }
    } else {
        if (batteryOneOkStatus == IO::GPIO::State::HIGH
            && batteryTwoOkStatus == IO::GPIO::State::HIGH
            && eStopActiveStatus == IO::GPIO::State::HIGH
            && voltStatus == 1) {
            stoStatus = IO::GPIO::State::HIGH;
            numAttemptsMade = 0;
        } else {
            if (numAttemptsMade > MAX_STO_ATTEMPTS) {
                EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Too many fails, error out");
                stoStatus = IO::GPIO::State::LOW;
                numAttemptsMade = 0;
                return;
            }
            EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "1: %d, 2: %d, e: %d", batteryOneOkStatus, batteryTwoOkStatus, eStopActiveStatus);

            numAttemptsMade++;
        }
    }
}

void PreChargeKEV1N::getMCKey() {
    keyInStatus = key.readPin();
}

void PreChargeKEV1N::mcOnState() {
    pre_charged = 2;
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
        if (prevState != state) {
            sendChangePDO();
        }
        prevState = state;
    }
    //else stay on MC_ON
}

void PreChargeKEV1N::prechargeState() {
    //Set apm relay
    setAPM(PreChargeKEV1N::PinStatus::ENABLE);
    //Send Pre-op
    pre_charged = 0;
    //Wait 2 seconds
    while ((time::millis() - state_start_time) < PRECHARGE_DELAY) {
        state = State::MC_ON;
        sendChangePDO();
    }
    //Send Op
    pre_charged = 1;

    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreChargeKEV1N::contCloseState() {
    cont.setOpen(false);
    contStatus = 1;
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyInStatus == IO::GPIO::State::HIGH) {
        setAPM(PreChargeKEV1N::PinStatus::ENABLE);

        //CAN message to wake TMS
        uint8_t payload[2] = {0x01, 0x08};
        IO::CANMessage TMSOpMessage(0, 2, payload, false);
        can.transmit(TMSOpMessage);

        state = State::MC_ON;
    }
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

}// namespace PreCharge
