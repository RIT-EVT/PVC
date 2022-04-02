#include <pre-charge/pre_charge.hpp>

#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace pre_charge {

pre_charge::pre_charge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
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

    keyStatus = IO::GPIO::State::LOW;
    stoStatus = IO::GPIO::State::LOW;
    batteryOneStatus = IO::GPIO::State::LOW;
    batteryTwoStatus = IO::GPIO::State::LOW;
    eStopStatus = IO::GPIO::State::LOW;
    //TODO: Add GFD
}

void pre_charge::handle() {
    getSTO();  //update value of STO
    getMCKey();//update value of MC_KEY_IN

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
    case pre_charge::State::FORWARD_DISABLE:
        forwardDisableState();
    default:
        break;
    }
}

void pre_charge::getSTO() {
    batteryOneStatus = batteryOne.readPin();
    batteryTwoStatus = batteryTwo.readPin();
    eStopStatus = eStop.readPin();
    //TODO: Add GFD

    if (batteryOneStatus == IO::GPIO::State::HIGH && batteryTwoStatus == IO::GPIO::State::HIGH && eStopStatus == IO::GPIO::State::LOW) {
        stoStatus = IO::GPIO::State::HIGH;
    } else {
        stoStatus = IO::GPIO::State::LOW;
    }
}

void pre_charge::getMCKey() {
    keyStatus = key.readPin();
}

void pre_charge::setPrecharge(int state) {
    if (state) {
        pc.writePin(IO::GPIO::State::HIGH);
    } else {
        pc.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::setDischarge(int state) {
    if (state) {
        dc.writePin(IO::GPIO::State::HIGH);
    } else {
        dc.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::setContactor(int state) {
    if (state) {
        cont.writePin(IO::GPIO::State::HIGH);
    } else {
        cont.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::setAPM(int state) {
    if (state) {
        apm.writePin(IO::GPIO::State::HIGH);
    } else {
        apm.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::setForward(int state) {
    if (state) {
        forward.writePin(IO::GPIO::State::HIGH);
    } else {
        forward.writePin(IO::GPIO::State::LOW);
    }
}

void pre_charge::mcOffState() {
    if (stoStatus == IO::GPIO::State::LOW) {
        state = State::ESTOPWAIT;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyStatus == IO::GPIO::State::HIGH) {
        state = State::PRECHARGE;
    }
    //else stay on MC_OFF
}

void pre_charge::mcOnState() {
    if (stoStatus == IO::GPIO::State::LOW || keyStatus == IO::GPIO::State::LOW) {
        state = State::FORWARD_DISABLE;
    }
    //else stay on MC_ON
}

void pre_charge::eStopState() {
    if (stoStatus == IO::GPIO::State::HIGH) {
        state = State::MC_OFF;
    }
    //else stay on E-Stop
}

void pre_charge::prechargeState() {
    setPrecharge(1);
    time::wait(5);//wait 5 tau
    setPrecharge(0);
    if (stoStatus == IO::GPIO::State::LOW || keyStatus == IO::GPIO::State::LOW) {
        state = State::CONT_OPEN;
    } else if (stoStatus == IO::GPIO::State::HIGH) {
        state = State::CONT_CLOSE;
    }
}

void pre_charge::dischargeState() {
    setDischarge(1);
    time::wait(10);//wait 10 tau
    setDischarge(0);
    state = State::MC_OFF;
}

void pre_charge::contOpenState() {
    setContactor(0);
    state = State::DISCHARGE;
}

void pre_charge::contCloseState() {
    setContactor(1);
    setForward(1);
    setAPM(1);
    if (stoStatus == IO::GPIO::State::LOW || keyStatus == IO::GPIO::State::LOW) {
        state = State::CONT_OPEN;
    } else if (stoStatus == IO::GPIO::State::HIGH && keyStatus == IO::GPIO::State::HIGH) {
        state = State::MC_ON;
    }
}

void pre_charge::forwardDisableState() {
    setForward(0);
    time::wait(5000);
    setAPM(0);
    state = State::CONT_OPEN;
}

std::string pre_charge::printState() {
    std::string str = "";

    str.append("MC_KEY_IN: ");
    str.append(std::to_string(static_cast<int>(keyStatus)));
    str.append("\t");

    str.append("[");
    str.append(std::to_string(static_cast<int>(batteryOneStatus)));
    str.append(" : ");
    str.append(std::to_string(static_cast<int>(batteryTwoStatus)));
    str.append(" : ");
    str.append(std::to_string(static_cast<int>(eStopStatus)));
    str.append("] -> ");

    str.append("STO: ");
    str.append(std::to_string(static_cast<int>(stoStatus)));
    str.append("\t");

    str.append("Cont: ");
    str.append(std::to_string(static_cast<int>(cont.readPin())));
    str.append("\t");

    str.append("APM: ");
    str.append(std::to_string(static_cast<int>(apm.readPin())));
    str.append("\t");

    str.append("Forward: ");
    str.append(std::to_string(static_cast<int>(forward.readPin())));
    str.append("\t");

    str.append("Current State: ");
    str.append(stateString(state));
    str.append("\n\r");

    return str;
}

std::string pre_charge::stateString(State currentState) {
    std::string s("unknown");

    switch (currentState) {
    case State::MC_OFF:
        s = "MC_OFF";
        break;
    case State::ESTOPWAIT:
        s = "ESTOPWAIT";
        break;
    case State::PRECHARGE:
        s = "PRECHARGE";
        break;
    case State::CONT_CLOSE:
        s = "CONT_CLOSE";
        break;
    case State::MC_ON:
        s = "MC_ON";
        break;
    case State::CONT_OPEN:
        s = "CONT_OPEN";
        break;
    case State::DISCHARGE:
        s = "DISCHARGE";
        break;
    case State::FORWARD_DISABLE:
        s = "FORWARD_DISABLE";
        break;
    default:
        break;
    }
    return s;
}

CO_OBJ_T* pre_charge::getObjectDictionary() {
    return objectDictionary;
}

uint16_t pre_charge::getObjectDictionarySize() {
    return OBJECT_DICTIONARY_SIZE;
}

}// namespace pre_charge