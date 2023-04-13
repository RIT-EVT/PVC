#include <PreCharge/PreCharge.hpp>

#include <EVT/utils/log.hpp>
#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreCharge::PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
                     IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, Contactor cont,
                     IO::GPIO& apm,  GFDB::GFDB& gfdb, IO::CAN& can, MAX22530 MAX) : key(key),
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

    gfdStatus = 0;

    cycle_key = 0;
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
        prechargeState(uart);
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

    if (batteryOneOkStatus == IO::GPIO::State::HIGH
        && batteryTwoOkStatus == IO::GPIO::State::HIGH
        && eStopActiveStatus == IO::GPIO::State::HIGH
        && !gfdStatus) {
        stoStatus = IO::GPIO::State::HIGH;
    } else {
        if (numAttemptsMade > MAX_STO_ATTEMPTS) {
            cycle_key = 1;
            stoStatus = IO::GPIO::State::LOW;
            numAttemptsMade = 0;
            return;
        }

        numAttemptsMade++;
    }
}

int PreCharge::getPrechargeStatus(IO::UART& uart) {
    PrechargeStatus status;
    static uint64_t delta_time;
    uint16_t pack_voltage;
    uint16_t measured_voltage;
    uint16_t expected_voltage;

    if (in_precharge == 0) {
        state_start_time = time::millis();
        in_precharge = 1;
        delta_time = time::millis() - state_start_time;
        status = PrechargeStatus::OK;
        pack_voltage = MAX.readVoltage(0x02);
        uart.printf("Pack: %d\r\n", pack_voltage);
        measured_voltage = MAX.readVoltage(0x01);
        expected_voltage = solveForVoltage((pack_voltage - measured_voltage), delta_time);
        uart.printf("Expected: %d\r\n", static_cast<int>(expected_voltage));
        uart.printf("Measured: %d\r\n", measured_voltage);
    } else {
        delta_time = time::millis() - state_start_time;
        status = PrechargeStatus::OK;
        pack_voltage = MAX.readVoltage(0x02);
        uart.printf("Pack: %d\r\n", pack_voltage);
        expected_voltage = solveForVoltage(pack_voltage, delta_time);
        uart.printf("Expected: %d\r\n", static_cast<int>(expected_voltage));
        measured_voltage = MAX.readVoltage(0x01);
        uart.printf("Measured: %d\r\n", measured_voltage);
    }

    if (measured_voltage >= (expected_voltage - 5) && measured_voltage <= (expected_voltage + 5)) {
        if (measured_voltage >= (pack_voltage - 1) && measured_voltage <= (pack_voltage + 1)) {
            status = PrechargeStatus::DONE;
            in_precharge = 0;
        } else {
            status = PrechargeStatus::OK;
        }
    } else {
        status = PrechargeStatus::ERROR;
        cycle_key = 1;
        in_precharge = 0;
    }

    return static_cast<int>(status);
}

uint16_t PreCharge::solveForVoltage(uint16_t pack_voltage, uint64_t delta_time) {
    return (pack_voltage * (1 - exp(-(delta_time / (1000 * 30 * 0.014)))));
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
    contStatus = cont.openState();
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

void PreCharge::prechargeState(IO::UART& uart) {
    setPrecharge(PreCharge::PinStatus::ENABLE);
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
    int precharging = getPrechargeStatus(uart);

    uart.printf("Precharging Status: %d\r\n", precharging);
    // Stay in prechargeState until DONE unless ERROR
    if (precharging == static_cast<int>(PrechargeStatus::ERROR) || stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
    } else if (precharging == static_cast<int>(PrechargeStatus::DONE)) {
        state = State::CONT_CLOSE;
    }
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
    cont.setOpen(true);
    state = State::DISCHARGE;
    state_start_time = time::millis();
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreCharge::contCloseState() {
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;

    cont.setOpen(false);
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
        Statusword,
        0x00,
        static_cast<unsigned short>(keyInStatus) << 4 | static_cast<unsigned short>(stoStatus),
        static_cast<unsigned short>(batteryOneOkStatus) << 4 | static_cast<unsigned short>(batteryTwoOkStatus),
        static_cast<unsigned short>(eStopActiveStatus) << 4 | static_cast<unsigned short>(apmStatus),
        static_cast<unsigned short>(pcStatus) << 4 | static_cast<unsigned short>(dcStatus),
        static_cast<unsigned short>(contStatus) << 4
    };
    IO::CANMessage changePDOMessage(0x48A, 7, &payload[0], false);
    can.transmit(changePDOMessage);

    EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "s: %d, k: %d; sto: %d; b1: %d; b2: %d; e: %d; f: %d, pc: %d, dc: %d, c: %d, apm: %d",
                               state, keyInStatus, stoStatus, batteryOneOkStatus, batteryTwoOkStatus, eStopActiveStatus, apmStatus, pcStatus, dcStatus, contStatus);
}

}// namespace PreCharge
