#include <pre-charge/pre_charge.hpp>

#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace pre_charge {

pre_charge::pre_charge(IO::GPIO& key, IO::GPIO& pc, IO::GPIO& dc, IO::GPIO& cont,
                        IO::GPIO& batteryOne, IO::GPIO& batteryTwo, IO::GPIO& eStop) :  key(key),
                                                                                        pc(pc),
                                                                                        dc(dc),
                                                                                        cont(cont),
                                                                                        batteryOne(batteryOne), 
                                                                                        batteryTwo(batteryTwo), 
                                                                                        eStop(eStop) {
    keyStatus = IO::GPIO::State::LOW;
    stoStatus = IO::GPIO::State::LOW;
    batteryOneStatus = IO::GPIO::State::LOW;
    batteryTwoStatus = IO::GPIO::State::LOW;
    eStopStatus = IO::GPIO::State::LOW;
}

void pre_charge::handle() {
    getSTO();   //update value of STO
    getMCKey(); //update value of MC_KEY_IN

    switch (state) {
        case pre_charge::State::MC_OFF:
            mcOffState();
            break;
        case pre_charge::State::ESTOPWAIT:
            eStopState();
            break;
        case pre_charge::State::PRECHARGE:
            prechargeState();
            break;
        case pre_charge::State::CONT_CLOSE:
            contCloseState();
            break;
        case pre_charge::State::MC_ON:
            mcOnState();
            break;
        case pre_charge::State::CONT_OPEN:
            contOpenState();
            break;
        case pre_charge::State::DISCHARGE:
            dischargeState();
            break;
        default:
            break;
        }
}

void pre_charge::getSTO() {
    batteryOneStatus = batteryOne.readPin();
    batteryTwoStatus = batteryTwo.readPin();
    eStopStatus = eStop.readPin();

    if(batteryOneStatus == IO::GPIO::State::HIGH && 
    batteryTwoStatus == IO::GPIO::State::HIGH && 
    eStopStatus == IO::GPIO::State::HIGH) {
        stoStatus = IO::GPIO::State::HIGH;
    } else {
        stoStatus = IO::GPIO::State::LOW;
    }
}

void pre_charge::getMCKey() {
    keyStatus = key.readPin();
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
    if(stoStatus == IO::GPIO::State::LOW) {
        state = State::ESTOPWAIT;
    } else if(stoStatus == IO::GPIO::State::HIGH && keyStatus == IO::GPIO::State::HIGH) {
        state = State::PRECHARGE;
    }
    //else stay on MC_OFF
}

void pre_charge::mcOnState() {
    if(stoStatus == IO::GPIO::State::LOW || keyStatus == IO::GPIO::State::LOW) {
        state = State::CONT_OPEN;
    }
    //else stay on MC_ON
}

void pre_charge::eStopState() {
    if(stoStatus == IO::GPIO::State::HIGH) {
        state = State::MC_OFF;
    }
    //else stay on E-Stop
}

void pre_charge::prechargeState() {
    setPrecharge(1);
    time::wait(5);  //wait 5 tau
    setPrecharge(0);
    if(stoStatus == IO::GPIO::State::LOW || keyStatus == IO::GPIO::State::LOW) {
        state = State::CONT_OPEN;
    } else if(stoStatus == IO::GPIO::State::HIGH) {
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
    if(stoStatus == IO::GPIO::State::LOW || keyStatus == IO::GPIO::State::LOW) {
        state = State::CONT_OPEN;
    } else if(stoStatus == IO::GPIO::State::HIGH && keyStatus == IO::GPIO::State::HIGH) {
        state = State::MC_ON;
    }
}

std::string pre_charge::printState() {
    std::string str = "";
    str.append("State Machine Report:\n\r");
    
    str.append("MC_KEY_IN: ");
    str.append(std::to_string(static_cast<int>(keyStatus)));
    str.append("\n\r");

    str.append("STO: ");
    str.append(std::to_string(static_cast<int>(stoStatus)));
    str.append("\n\n\r");

    return str;
}

}// namespace pre_charge