#include <pre-charge/pre_charge.hpp>

#include <libs/EVT-core/include/EVT/dev/platform/f3xx/f302x8/Timerf302x8.hpp>

namespace pre_charge {

pre_charge::pre_charge(IO::GPIO& key, IO::GPIO& pc, IO::GPIO& dc, IO::GPIO& cont) : gpio(key),
                                                                        gpio(pc),
                                                                        gpio(dc),
                                                                        gpio(cont),
                                                                        gpio(batteryOne), 
                                                                        gpio(batteryTwo), 
                                                                        gpio(eStop) {
    keyStatus = IO::GPIO::State::LOW;
    stoStatus = IO::GPIO::State::LOW;
    batteryOneStatus = IO::GPIO::State::LOW;
    batteryTwoStatus = IO::GPIO::State::LOW;
    eStopStatus = IO::GPIO::State::LOW;
}

void pre_charge::handle() {
    stoStatus = getSTO();   //update value of STO
    keyStatus = getMCKey(); //update value of MC_KEY_IN

    switch (state) {
        case State::MC_OFF:
            mcOffState();
            break;
        case State::ESTOPWAIT:
            eStopStatus();
            break;
        case State::PRECHARGE:
            prechargeStatus();
            break;
        case State::CONT_CLOSE:
            contCloseState();
            break;
        case State::MC_ON:
            mcOnState();
            break;
        case State::CONT_OPEN:
            contOpenState();
            break;
        case State::DISCHARGE:
            dischargeState();
            break;
        default:
            break;
        }
}

int pre_charge::getSTO() {
    batteryOneStatus = batteryOne.readPin();
    batteryTwoStatus = batteryTwo.readPin();
    eStopStatus = eStop.readPin();

    if(batteryOneStatus == IO::GPIO::State::HIGH && 
    batteryTwoStatus == IO::GPIO::State::HIGH && 
    eStopStatus == IO::GPIO::State::HIGH) {
        return 1;
    } else {
        return 0;
    }
}

int pre_charge::getMCKey() {
    keyStatus = key.readPin();

    if(keyStatus == IO::GPIO::State::HIGH) {
        return 1;
    } else {
        return 0;
    }
}

void pre_charge::setPrecharge(int state) {
    if(state) {
        pc.writePin(IO::GPIO::State::HIGH);
    } else {
        pc.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::setDischarge(int state) {
    if(state) {
        dc.writePin(IO::GPIO::State::HIGH);
    } else {
        dc.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::setContactor(int state) {
    if(state) {
        cont.writePin(IO::GPIO::State::HIGH);
    } else {
        cont.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::mcOffState() {
    if(stoStatus == 0) {
        state = State::ESTOPWAIT;
    } else if(stoStatus == 1 && keyStatus == 1) {
        state = State::PRECHARGE;
    }
    //else stay on MC_OFF
}

void pre_charge::mcOnState() {
    if(stoStatus == 0 || keyStatus == 0) {
        state = State::CONT_OPEN;
    }
    //else stay on MC_ON
}

void pre_charge::eStopState() {
    if(stoStatus == 1) {
        state = State::MC_OFF;
    }
    //else stay on E-Stop
}

void pre_charge::prechargeState() {
    setPrecharge(1);
    time::wait(5);  //wait 5 tau
    setPrecharge(0);
    if(stoStatus == 0 || keyStatus == 0) {
        state = State::CONT_OPEN;
    } else if(stoStatus == 1) {
        state = State::CONT_CLOSE;
    }
}

void pre_charge::dischargeState() {
    setDischarge(1);
    time::wait(10); //wait 10 tau
    setDischarge(0);
    state = State::MC_OFF;
}

void pre_charge::contOpenState() {
    setContactor(0);
    state = State::DISCHARGE;
}

void pre_charge::contCloseState() {
    setContactor(1);
    if(stoStatus == 0 || keyStatus == 0) {
        state = State::CONT_OPEN;
    } else if(stoStatus == 1 && keyStatus == 1) {
        state = State::MC_ON;
    }
}

}// namespace pre_charge