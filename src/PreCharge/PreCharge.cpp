#include <PreCharge/PreCharge.hpp>

#include <EVT/utils/log.hpp>
#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreCharge::PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
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
    apmStatus = IO::GPIO::State::LOW;

    gfdStatus = 1;
    initVolt = 0;

    cycle_key = 0;
    sendChangePDO();
}

PreCharge::PVCStatus PreCharge::handle(IO::UART& uart) {
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

    if (cycle_key) {
        return PVCStatus::PVC_ERROR;
    } else {
        return PVCStatus::PVC_OK;
    }
}

void PreCharge::getSTO() {
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

    if (in_precharge == 2) {
        if (batteryOneOkStatus == IO::GPIO::State::HIGH
            && batteryTwoOkStatus == IO::GPIO::State::HIGH
            && eStopActiveStatus == IO::GPIO::State::HIGH
            && gfdStatus == 1) {
            stoStatus = IO::GPIO::State::HIGH;
            numAttemptsMade = 0;
        } else {
            if (numAttemptsMade > MAX_STO_ATTEMPTS) {
                EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Too many fails, error out");
                cycle_key = 1;
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
            && eStopActiveStatus == IO::GPIO::State::HIGH) {
            stoStatus = IO::GPIO::State::HIGH;
            numAttemptsMade = 0;
        } else {
            if (numAttemptsMade > MAX_STO_ATTEMPTS) {
                EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Too many fails, error out");
                cycle_key = 1;
                stoStatus = IO::GPIO::State::LOW;
                numAttemptsMade = 0;
                return;
            }
            EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "1: %d, 2: %d, e: %d", batteryOneOkStatus, batteryTwoOkStatus, eStopActiveStatus);

            numAttemptsMade++;
        }
    }
}

int PreCharge::getPrechargeStatus() {
    PrechargeStatus status;
    static uint64_t delta_time;
    uint16_t pack_voltage;
    uint16_t measured_voltage;
    uint16_t expected_voltage;

    if (in_precharge == 0) {
        in_precharge = 1;
        state_start_time = time::millis();
        initVolt = MAX.readVoltage(0x01);
    }

    delta_time = time::millis() - state_start_time;
    measured_voltage = MAX.readVoltage(0x01);
    pack_voltage = MAX.readVoltage(0x02);
    expected_voltage = solveForVoltage(pack_voltage, delta_time);

    if (measured_voltage >= (expected_voltage - 5) && measured_voltage <= (expected_voltage + 5) && pack_voltage > MIN_PACK_VOLTAGE) {
        if (measured_voltage >= (pack_voltage - 1) && measured_voltage <= (pack_voltage + 1)) {
            lastPrechargeTime = time::millis();
            status = PrechargeStatus::DONE;
            in_precharge = 2;
        } else {
            status = PrechargeStatus::OK;
        }
    } else {
        EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::ERROR, "Meas: %d, Exp: %d, Pack: %d", measured_voltage, expected_voltage, pack_voltage);
        status = PrechargeStatus::ERROR;
        cycle_key = 1;
        in_precharge = 0;
    }

    return static_cast<int>(status);
}

uint16_t PreCharge::solveForVoltage(uint16_t pack_voltage, uint64_t delta_time) {
    return initVolt + ((pack_voltage - initVolt) * (1 - exp(-(delta_time / (1000 * 30 * 0.014)))));
}

void PreCharge::getMCKey() {
    if (cycle_key) {
        if (key.readPin() == IO::GPIO::State::LOW) {
            cycle_key = 0;
        } else {
            keyInStatus = IO::GPIO::State::LOW;
        }
    } else {
        keyInStatus = key.readPin();
    }
}

void PreCharge::getIOStatus() {
    pcStatus = pc.readPin();
    dcStatus = dc.readPin();
    apmStatus = apm.readPin();
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

void PreCharge::setAPM(PreCharge::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        apm.writePin(IO::GPIO::State::HIGH);
    } else {
        apm.writePin(IO::GPIO::State::LOW);
    }
}

void PreCharge::mcOffState() {
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

void PreCharge::mcOnState() {
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
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
    int precharging = getPrechargeStatus();

    // Stay in prechargeState until DONE unless ERROR
    if (precharging == static_cast<int>(PrechargeStatus::ERROR) || stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "Precharge error");
        state = State::FORWARD_DISABLE;
    } else if (precharging == static_cast<int>(PrechargeStatus::DONE)) {
        EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "Precharge done");
        state = State::CONT_CLOSE;
    }
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreCharge::dischargeState() {
    setDischarge(PreCharge::PinStatus::ENABLE);
    if ((time::millis() - state_start_time) > DISCHARGE_DELAY) {
        setDischarge(PreCharge::PinStatus::DISABLE);
        state = State::MC_OFF;
        sendChangePDO();
    }
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreCharge::contOpenState() {
    cont.setOpen(true);
    contStatus = 0;
    state = State::DISCHARGE;
    state_start_time = time::millis();
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreCharge::contCloseState() {
    cont.setOpen(false);
    contStatus = 1;
    setPrecharge(PreCharge::PinStatus::DISABLE);
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
    setPrecharge(PreCharge::PinStatus::DISABLE);
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
        static_cast<uint8_t>(state),
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
