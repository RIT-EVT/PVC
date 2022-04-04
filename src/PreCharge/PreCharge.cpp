#include <PreCharge/PreCharge.hpp>

#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreCharge::PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
                     IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, IO::GPIO& cont,
                     IO::GPIO& apm, IO::GPIO& forward) : key(key),
                                                         batteryOne(batteryOne),
                                                         batteryTwo(batteryTwo),
                                                         eStop(eStop),
                                                         pc(pc),
                                                         dc(dc),
                                                         cont(cont),
                                                         apm(apm),
                                                         forward(forward) {
    state = State::MC_OFF;

    keyInStatus = IO::GPIO::State::LOW;
    stoStatus = IO::GPIO::State::LOW;
    batteryOneOkStatus = IO::GPIO::State::LOW;
    batteryTwoOkStatus = IO::GPIO::State::LOW;
    eStopActiveStatus = IO::GPIO::State::LOW;
    //TODO: Add GFD
}

void PreCharge::handle() {
    getSTO();  //update value of STO
    getMCKey();//update value of MC_KEY_IN

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
    batteryOneOkStatus = batteryOne.readPin();
    batteryTwoOkStatus = batteryTwo.readPin();
    eStopActiveStatus = eStop.readPin();
    //TODO: Add GFD

    if (batteryOneOkStatus == IO::GPIO::State::HIGH && batteryTwoOkStatus == IO::GPIO::State::HIGH && eStopActiveStatus == IO::GPIO::State::LOW) {
        stoStatus = IO::GPIO::State::HIGH;
    } else {
        stoStatus = IO::GPIO::State::LOW;
    }
}

void PreCharge::getMCKey() {
    keyInStatus = key.readPin();
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
    if (static_cast<uint8_t>(state)) {
        cont.writePin(IO::GPIO::State::HIGH);
    } else {
        cont.writePin(IO::GPIO::State::LOW);
    }
}

void PreCharge::setAPM(PreCharge::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        apm.writePin(IO::GPIO::State::HIGH);
    } else {
        apm.writePin(IO::GPIO::State::LOW);
    }
}

void PreCharge::setForward(PreCharge::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        forward.writePin(IO::GPIO::State::HIGH);
    } else {
        forward.writePin(IO::GPIO::State::LOW);
    }
}

void PreCharge::mcOffState() {
    if (stoStatus == IO::GPIO::State::LOW) {
        state = State::ESTOPWAIT;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyInStatus == IO::GPIO::State::HIGH) {
        state = State::PRECHARGE;
        state_start_time = time::millis();
    }
    //else stay on MC_OFF
}

void PreCharge::mcOnState() {
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
        state_start_time = time::millis();
    }
    //else stay on MC_ON
}

void PreCharge::eStopState() {
    if (stoStatus == IO::GPIO::State::HIGH) {
        state = State::MC_OFF;
    }
    //else stay on E-Stop
}

void PreCharge::prechargeState() {
    setPrecharge(PreCharge::PinStatus::ENABLE);
    if ((time::millis() - state_start_time) > PRECHARGE_DELAY) {
        setPrecharge(PreCharge::PinStatus::DISABLE);
        if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
            state = State::CONT_OPEN;
        } else {
            state = State::CONT_CLOSE;
        }
    }
}

void PreCharge::dischargeState() {
    setDischarge(PreCharge::PinStatus::ENABLE);
    if ((time::millis() - state_start_time) > DISCHARGE_DELAY) {
        setDischarge(PreCharge::PinStatus::DISABLE);
        state = State::MC_OFF;
    }
}

void PreCharge::contOpenState() {
    setContactor(PreCharge::PinStatus::DISABLE);
    state = State::DISCHARGE;
    state_start_time = time::millis();
}

void PreCharge::contCloseState() {
    setContactor(PreCharge::PinStatus::ENABLE);
    setForward(PreCharge::PinStatus::ENABLE);
    setAPM(PreCharge::PinStatus::ENABLE);
    if (stoStatus == IO::GPIO::State::LOW || keyInStatus == IO::GPIO::State::LOW) {
        state = State::CONT_OPEN;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyInStatus == IO::GPIO::State::HIGH) {
        state = State::MC_ON;
    }
}

void PreCharge::forwardDisableState() {
    setForward(PreCharge::PinStatus::DISABLE);
    if ((time::millis() - state_start_time) > FORWARD_DISABLE_DELAY) {
        setAPM(PreCharge::PinStatus::DISABLE);
        state = State::CONT_OPEN;
    }
}

}// namespace PreCharge