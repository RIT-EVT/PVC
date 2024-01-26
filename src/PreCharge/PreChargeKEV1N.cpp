#include <PreCharge/PreChargeKEV1N.hpp>

#include <EVT/utils/log.hpp>
#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreChargeKEV1N::PreChargeKEV1N(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
                               IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, Contactor cont,
                               IO::GPIO& apm, GFDB::GFDB& gfdb, IO::CAN& can, MAX22530 MAX) : key(key),
                                                                                              batteryOne(batteryOne),
                                                                                              batteryTwo(batteryTwo),
                                                                                              eStop(eStop),
                                                                                              pc(pc),
                                                                                              dc(dc),
                                                                                              cont(cont),
                                                                                              apm(apm),
                                                                                              gfdb(gfdb),
                                                                                              can(can),
                                                                                              MAX(MAX) {
    state = State::MC_OFF;
    prevState = State::MC_OFF;

    keyInStatus = IO::GPIO::State::LOW;
    stoStatus = IO::GPIO::State::LOW;
    batteryOneOkStatus = IO::GPIO::State::LOW;
    batteryTwoOkStatus = IO::GPIO::State::LOW;
    eStopActiveStatus = IO::GPIO::State::HIGH;
    pcStatus = IO::GPIO::State::LOW;
    dcStatus = IO::GPIO::State::LOW;
    contStatus = 0;
    voltStatus = 0;
    apmStatus = IO::GPIO::State::LOW;

    gfdStatus = 1;
    initVolt = 0;

    pre_charged = 2;
    sendChangePDO();
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

void PreChargeKEV1N::getIOStatus() {
    pcStatus = pc.readPin();
    dcStatus = dc.readPin();
    apmStatus = apm.readPin();
}

void PreChargeKEV1N::setPrecharge(PreChargeKEV1N::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        pc.writePin(IO::GPIO::State::HIGH);
    } else {
        pc.writePin(IO::GPIO::State::LOW);
    }
}

void PreChargeKEV1N::setDischarge(PreChargeKEV1N::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        dc.writePin(IO::GPIO::State::HIGH);
    } else {
        dc.writePin(IO::GPIO::State::LOW);
    }
}

void PreChargeKEV1N::setAPM(PreChargeKEV1N::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        apm.writePin(IO::GPIO::State::HIGH);
    } else {
        apm.writePin(IO::GPIO::State::LOW);
    }
}

void PreChargeKEV1N::mcOffState() {
    in_precharge = 0;
    if (stoStatus == IO::GPIO::State::LOW) {
        state = State::ESTOPWAIT;
        if (prevState != state) {
            sendChangePDO();
        }
        prevState = state;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyInStatus == IO::GPIO::State::HIGH) {
        state = State::PRECHARGE;
        state_start_time = time::millis();
        if (prevState != state) {
            sendChangePDO();
        }
        prevState = state;
    }
    //else stay on MC_OFF
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

void PreChargeKEV1N::eStopState() {
    if (stoStatus == IO::GPIO::State::HIGH) {
        state = State::MC_OFF;
        if (prevState != state) {
            sendChangePDO();
        }
        prevState = state;
    }
    //else stay on E-Stop
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

void PreChargeKEV1N::contOpenState() {
    cont.setOpen(true);
    state = State::DISCHARGE;
    state_start_time = time::millis();
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreChargeKEV1N::contCloseState() {
    cont.setOpen(false);
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyInStatus == IO::GPIO::State::HIGH) {
        setAPM(PreChargeKEV1N::PinStatus::ENABLE);

        //CAN message to wake TMS
        uint8_t payload[1] = {0x01};
        IO::CANMessage TMSOpMessage(0, 1, payload, false);
        can.transmit(TMSOpMessage);

        state = State::MC_ON;
    }
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreChargeKEV1N::forwardDisableState() {
    setPrecharge(PreChargeKEV1N::PinStatus::DISABLE);
    if ((time::millis() - state_start_time) > FORWARD_DISABLE_DELAY) {
        setAPM(PreChargeKEV1N::PinStatus::DISABLE);

        //CAN message to send TMS into pre-op state
        uint8_t payload[1] = {0x80};
        IO::CANMessage TMSOpMessage(0, 1, payload, false);
        can.transmit(TMSOpMessage);

        state = State::CONT_OPEN;
    }
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

CO_OBJ_T* PreChargeKEV1N::getObjectDictionary() {
    return &objectDictionary[0];
}

uint16_t PreChargeKEV1N::getObjectDictionarySize() {
    return OBJECT_DICTIONARY_SIZE;
}

void PreChargeKEV1N::sendChangePDO() {
    uint8_t payload[] = {
        Statusword,
        0x00,
        static_cast<unsigned short>(keyInStatus) << 4 | static_cast<unsigned short>(stoStatus),
        static_cast<unsigned short>(batteryOneOkStatus) << 4 | static_cast<unsigned short>(batteryTwoOkStatus),
        static_cast<unsigned short>(eStopActiveStatus) << 4 | static_cast<unsigned short>(apmStatus),
        static_cast<unsigned short>(pcStatus) << 4 | static_cast<unsigned short>(dcStatus),
        static_cast<unsigned short>(contStatus) << 4};
    IO::CANMessage changePDOMessage(0x48A, 7, &payload[0], false);
    can.transmit(changePDOMessage);

    EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "s: %d, k: %d; sto: %d; b1: %d; b2: %d; e: %d; f: %d, pc: %d, dc: %d, c: %d, apm: %d",
                               state, keyInStatus, stoStatus, batteryOneOkStatus, batteryTwoOkStatus, eStopActiveStatus, apmStatus, pcStatus, dcStatus, contStatus);
}

}// namespace PreCharge