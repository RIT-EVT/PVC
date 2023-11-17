#include <PreCharge/PreCharge.hpp>

#include <EVT/utils/log.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreCharge::PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
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
            && eStopActiveStatus == IO::GPIO::State::HIGH
            && voltStatus == 1) {
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
    }
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
