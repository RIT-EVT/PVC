#include <PreCharge/PreChargeBase.hpp>

#include <EVT/utils/log.hpp>
#include <EVT/utils/time.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace PreCharge {

PreChargeBase::PreChargeBase(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
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

    sendChangePDO();
}

void PreChargeBase::getIOStatus() {
    pcStatus = pc.readPin();
    dcStatus = dc.readPin();
    apmStatus = apm.readPin();
}

void PreChargeBase::setPrecharge(PreChargeBase::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        pc.writePin(IO::GPIO::State::HIGH);
    } else {
        pc.writePin(IO::GPIO::State::LOW);
    }
}

void PreChargeBase::setDischarge(PreChargeBase::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        dc.writePin(IO::GPIO::State::HIGH);
    } else {
        dc.writePin(IO::GPIO::State::LOW);
    }
}

void PreChargeBase::setAPM(PreChargeBase::PinStatus state) {
    if (static_cast<uint8_t>(state)) {
        apm.writePin(IO::GPIO::State::HIGH);
    } else {
        apm.writePin(IO::GPIO::State::LOW);
    }
}

void PreChargeBase::mcOffState() {
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

void PreChargeBase::eStopState() {
    if (stoStatus == IO::GPIO::State::HIGH) {
        state = State::MC_OFF;
        if (prevState != state) {
            sendChangePDO();
        }
        prevState = state;
    }
    //else stay on E-Stop
}

void PreChargeBase::contOpenState() {
    cont.setOpen(true);
    contStatus = 0;
    state = State::DISCHARGE;
    state_start_time = time::millis();
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

void PreChargeBase::forwardDisableState() {
    setPrecharge(PreChargeBase::PinStatus::DISABLE);
    if ((time::millis() - state_start_time) > FORWARD_DISABLE_DELAY) {
        setAPM(PreChargeBase::PinStatus::DISABLE);

        //CAN message to send TMS into pre-op state
        uint8_t payload[2] = {0x80, 0x08};
        IO::CANMessage TMSOpMessage(0, 2, payload, false);
        can.transmit(TMSOpMessage);

        state = State::CONT_OPEN;
    }
    if (prevState != state) {
        sendChangePDO();
    }
    prevState = state;
}

CO_OBJ_T* PreChargeBase::getObjectDictionary() {
    return &objectDictionary[0];
}

uint16_t PreChargeBase::getObjectDictionarySize() {
    return OBJECT_DICTIONARY_SIZE;
}

void PreChargeBase::sendChangePDO() {
    uint8_t payload[] = {
        static_cast<uint8_t>(state),
        0x00,
        static_cast<unsigned short>(keyInStatus) << 4 | static_cast<unsigned short>(stoStatus),
        static_cast<unsigned short>(batteryOneOkStatus) << 4 | static_cast<unsigned short>(batteryTwoOkStatus),
        static_cast<unsigned short>(eStopActiveStatus) << 4 | static_cast<unsigned short>(apmStatus),
        static_cast<unsigned short>(pcStatus) << 4 | static_cast<unsigned short>(dcStatus),
        static_cast<unsigned short>(contStatus) << 4 | static_cast<unsigned short>(voltStatus)};
    IO::CANMessage changePDOMessage(0x48A, 7, payload, false);
    can.transmit(changePDOMessage);

    EVT::core::log::LOGGER.log(EVT::core::log::Logger::LogLevel::DEBUG, "s: %d, k: %d; sto: %d; b1: %d; b2: %d; e: %d; a: %d, pc: %d, dc: %d, c: %d, v: %d",
                               state, keyInStatus, stoStatus, batteryOneOkStatus, batteryTwoOkStatus, eStopActiveStatus, apmStatus, pcStatus, dcStatus, contStatus, voltStatus);
}

}// namespace PreCharge
