#include <PreCharge/PreCharge.hpp>

#include <EVT/utils/log.hpp>
#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreCharge::PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
                     IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, IO::GPIO& cont1,
                     IO::GPIO& cont2, IO::GPIO& apm,  GFDB::GFDB& gfdb, IO::CAN& can, IO::SPI& spi) : key(key),
                                                                                         batteryOne(batteryOne),
                                                                                         batteryTwo(batteryTwo),
                                                                                         eStop(eStop),
                                                                                         pc(pc),
                                                                                         dc(dc),
                                                                                         cont1(cont1),
                                                                                         cont2(cont2),
                                                                                         apm(apm),
                                                                                         gfdb(gfdb),
                                                                                         can(can),
                                                                                         spi(spi) {
    state = State::MC_OFF;
    prevState = State::MC_OFF;

    keyInStatus = IO::GPIO::State::LOW;
    stoStatus = IO::GPIO::State::LOW;
    batteryOneOkStatus = IO::GPIO::State::LOW;
    batteryTwoOkStatus = IO::GPIO::State::LOW;
    eStopActiveStatus = IO::GPIO::State::HIGH;
    // forwardStatus = IO::GPIO::State::LOW;
    pcStatus = IO::GPIO::State::LOW;
    dcStatus = IO::GPIO::State::LOW;
    cont1Status = IO::GPIO::State::LOW;
    cont2Status = IO::GPIO::State::LOW;
    apmStatus = IO::GPIO::State::LOW;

    gfdStatus = 0;
}

void PreCharge::handle() {
    getSTO();     //update value of STO
    getMCKey();   //update value of MC_KEY_IN
    getIOStatus();//update value of IOStatus

    Statusword = static_cast<uint8_t>(state);
    InputVoltage = 0; //Does not exist yet
    OutputVoltage = 0;//Does not exist yet
    BasePTemp = 0;    //Does not exist yet

    cyclicPDO = (InputVoltage << 32) | (OutputVoltage << 16) | BasePTemp;

    switch (state) {
    case PreCharge::State::MC_OFF:
        mcOffState();
        break;
    case PreCharge::State::ESTOPWAIT:
        eStopState();
        break;
    case PreCharge::State::PRECHARGE:
        prechargeState();
        break;
    case PreCharge::State::CONT_CLOSE:
        contCloseState();
        break;
    case PreCharge::State::MC_ON:
        mcOnState();
        break;
    case PreCharge::State::CONT_OPEN:
        contOpenState();
        break;
    case PreCharge::State::DISCHARGE:
        dischargeState();
        break;
    case PreCharge::State::FORWARD_DISABLE:
        forwardDisableState();
        break;
    default:
        break;
    }
}

void PreCharge::getSTO() {
    uint8_t gfdBuffer;
    IO::CAN::CANStatus gfdbConn = gfdb.requestIsolationState(&gfdBuffer);
    //GFDB messages gotten from datasheet. 00 = no error, 10 = warning, 11 = error
    if (gfdbConn == IO::CAN::CANStatus::OK && (gfdBuffer == 0b00 || gfdBuffer == 0b10)) {
        gfdStatus = 0;
    } else if (gfdBuffer == 0b11) {
        gfdStatus = 1;
    }

    batteryOneOkStatus = batteryOne.readPin();
    batteryTwoOkStatus = batteryTwo.readPin();
    eStopActiveStatus = eStop.readPin();

    if (batteryOneOkStatus == IO::GPIO::State::HIGH && batteryTwoOkStatus == IO::GPIO::State::HIGH && eStopActiveStatus == IO::GPIO::State::HIGH) {
        stoStatus = IO::GPIO::State::HIGH;
    } else {
        if (numAttemptsMade > MAX_STO_ATTEMPTS) {
            stoStatus = IO::GPIO::State::LOW;
            numAttemptsMade = 0;
            return;
        }

        numAttemptsMade++;
    }
}

int PreCharge::getPrechargeStatus() {
    PrechargeStatus status = PrechargeStatus::ERROR;
    float expected_voltage = solveForVoltage();
    uint8_t measured_voltage;
    spi.read(&measured_voltage, 8);

    if (measured_voltage >= (expected_voltage + 1.0) && measured_voltage <= (expected_voltage - 1.0)) {
        if (measured_voltage >= (PACK_VOLTAGE + 1.0) && measured_voltage <= (PACK_VOLTAGE - 1.0)) {
            status = PrechargeStatus::DONE;
        } else {
            status = PrechargeStatus::OK;
        }
    }

    return static_cast<int>(status);
}

float PreCharge::solveForVoltage() {
    uint32_t t = time::millis() - state_start_time;
    return (PACK_VOLTAGE * (1 - CONST_E * (-(t / (CONST_R * CONST_C)))));
}

void PreCharge::getMCKey() {
    keyInStatus = key.readPin();
}

void PreCharge::getIOStatus() {
    pcStatus = pc.readPin();
    dcStatus = dc.readPin();
    cont1Status = cont1.readPin();
    cont2Status = cont2.readPin();
    apmStatus = apm.readPin();
//    forwardStatus = forward.readPin();
}

void PreCharge::setPrecharge(PreCharge::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        pc.writePin(IO::GPIO::State::HIGH);
    } else {
        pc.writePin(IO::GPIO::State::LOW);
    }
}

void PreCharge::setDischarge(PreCharge::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        dc.writePin(IO::GPIO::State::HIGH);
    } else {
        dc.writePin(IO::GPIO::State::LOW);
    }
}

void PreCharge::setContactor(PreCharge::PinStatus state) {
    if (state == PinStatus::ENABLE) {
        cont1.writePin(IO::GPIO::State::HIGH);
        cont2.writePin(IO::GPIO::State::LOW);
    } else if (state == PinStatus::DISABLE) {
        cont1.writePin(IO::GPIO::State::LOW);
        cont2.writePin(IO::GPIO::State::HIGH);
    } else {
        cont1.writePin(IO::GPIO::State::LOW);
        cont2.writePin(IO::GPIO::State::LOW);
    }
}

void PreCharge::setAPM(PreCharge::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        apm.writePin(IO::GPIO::State::HIGH);
    } else {
        apm.writePin(IO::GPIO::State::LOW);
    }
}

// void PreCharge::setForward(PreCharge::PinStatus state) {
//    if (static_cast<uint8_t>(state)) {
//        forward.writePin(IO::GPIO::State::HIGH);
//    } else {
//        forward.writePin(IO::GPIO::State::LOW);
//    }
// }

void PreCharge::mcOffState() {
    setContactor(PreCharge::PinStatus::HOLD);
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

void PreCharge::mcOnState() {
    setContactor(PreCharge::PinStatus::HOLD);
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
        state_start_time = time::millis();
        if (prevState != state) {
            sendChangePDO();
        }
        prevState = state;
    }
    //else stay on MC_ON
}

void PreCharge::eStopState() {
    if (stoStatus == IO::GPIO::State::HIGH) {
        state = State::MC_OFF;
        if (prevState != state) {
            sendChangePDO();
        }
        prevState = state;
    }
    //else stay on E-Stop
}

void PreCharge::prechargeState() {
    setPrecharge(PreCharge::PinStatus::ENABLE);
    if (prevState != state) {
        sendChangePDO();
    }
    int precharging = getPrechargeStatus();

    // Stay in prechargeState until DONE unless ERROR
    if (precharging == static_cast<int>(PrechargeStatus::ERROR) || stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::CONT_OPEN;
    } else if (precharging == static_cast<int>(PrechargeStatus::DONE)) {
        state = State::CONT_CLOSE;
    }
    prevState = state;
}

void PreCharge::dischargeState() {
    setDischarge(PreCharge::PinStatus::ENABLE);
    if (prevState != state) {
        sendChangePDO();
    }
    if ((time::millis() - state_start_time) > DISCHARGE_DELAY) {
        setDischarge(PreCharge::PinStatus::DISABLE);
        state = State::MC_OFF;
        sendChangePDO();
    }
    prevState = state;
}

void PreCharge::contOpenState() {
    setContactor(PreCharge::PinStatus::DISABLE);
    state = State::DISCHARGE;
    state_start_time = time::millis();
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreCharge::contCloseState() {
    setContactor(PreCharge::PinStatus::ENABLE);
    setPrecharge(PreCharge::PinStatus::DISABLE);
    // setForward(PreCharge::PinStatus::ENABLE);
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyInStatus == IO::GPIO::State::HIGH) {
        setAPM(PreCharge::PinStatus::ENABLE);

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

void PreCharge::forwardDisableState() {
    // setForward(PreCharge::PinStatus::DISABLE);
    if ((time::millis() - state_start_time) > FORWARD_DISABLE_DELAY) {
        setAPM(PreCharge::PinStatus::DISABLE);

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

CO_OBJ_T* PreCharge::getObjectDictionary() {
    return &objectDictionary[0];
}

uint16_t PreCharge::getObjectDictionarySize() {
    return OBJECT_DICTIONARY_SIZE;
}

void PreCharge::sendChangePDO() {
    uint8_t payload[] = {
        Statusword,
        0x00,
        static_cast<unsigned short>(keyInStatus) << 4 | static_cast<unsigned short>(stoStatus),
        static_cast<unsigned short>(batteryOneOkStatus) << 4 | static_cast<unsigned short>(batteryTwoOkStatus),
        static_cast<unsigned short>(eStopActiveStatus) << 4 | static_cast<unsigned short>(apmStatus),
        static_cast<unsigned short>(pcStatus) << 4 | static_cast<unsigned short>(dcStatus),
        static_cast<unsigned short>(cont1Status) << 4 | static_cast<unsigned short>(cont2Status),
    };
    IO::CANMessage changePDOMessage(0x48A, 7, &payload[0], false);
    can.transmit(changePDOMessage);

    EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "s: %d, k: %d; sto: %d; b1: %d; b2: %d; e: %d; f: %d, pc: %d, dc: %d, c: %d, apm: %d",
                               state, keyInStatus, stoStatus, batteryOneOkStatus, batteryTwoOkStatus, eStopActiveStatus, apmStatus, pcStatus, dcStatus, cont1Status, cont2Status);
}

}// namespace PreCharge
