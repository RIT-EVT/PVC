#include <PreCharge/GFDB.hpp>

namespace IO = EVT::core::IO;
namespace time = EVT::core::time;

namespace GFDB {

GFDB::GFDB(IO::CAN& can) : can(can){};

IO::CAN::CANStatus GFDB::requestVoltagePositiveHighRes(int32_t* highRes) {
    uint8_t rxBuffer[8] = {};
    IO::CAN::CANStatus result = requestData(VN_HIGH_RES_CMD, rxBuffer, 8);
    if (result == IO::CAN::CANStatus::ERROR)
        return result;

    *highRes = (rxBuffer[0] << 24) | (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
    return IO::CAN::CANStatus::OK;
}

IO::CAN::CANStatus GFDB::requestVoltageNegativeHighRes(int32_t* highRes) {
    uint8_t rxBuffer[8] = {};
    IO::CAN::CANStatus result = requestData(VP_HIGH_RES_CMD, rxBuffer, 8);
    if (result == IO::CAN::CANStatus::ERROR)
        return result;

    *highRes = (rxBuffer[0] << 24) | (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
    return IO::CAN::CANStatus::OK;
}

IO::CAN::CANStatus GFDB::requestTemp(int32_t* temperature) {
    uint8_t tempBuffer[5] = {};

    IO::CAN::CANStatus result = requestData(TEMP_REQ_CMD, tempBuffer, 5);
    if (result == IO::CAN::CANStatus::ERROR) {
        return result;
    }

    *temperature = (tempBuffer[1] << 24) | (tempBuffer[2] << 16) | (tempBuffer[3] << 8) | tempBuffer[4];
    return result;
}

IO::CAN::CANStatus GFDB::requestIsolationState(uint8_t* isoState) {
    uint8_t buf[8];

    IO::CAN::CANStatus result = requestData(ISO_STATE_REQ_CMD, buf, 8);
    if (result != IO::CAN::CANStatus::OK) {
        return result;
    }

    *isoState = buf[1] & 0x03;
    return result;
}

IO::CAN::CANStatus GFDB::requestIsolationResistances(uint8_t* resistances) {
    return requestData(ISO_RESISTANCES_REQ_CMD, resistances, 8);
}

IO::CAN::CANStatus GFDB::requestIsolationCapacitances(uint8_t* capacitances) {
    return requestData(ISO_CAPACITANCES_REQ_CMD, capacitances, 8);
}

IO::CAN::CANStatus GFDB::requestVoltagesPositiveNegative(uint16_t* voltageP, uint16_t* voltageN) {
    uint8_t rxBuffer[8] = {};

    IO::CAN::CANStatus result = requestData(VP_VN_REQ_CMD, rxBuffer, 8);
    if (result == IO::CAN::CANStatus::ERROR)
        return result;

    *voltageP = rxBuffer[2] << 8 | rxBuffer[3];
    *voltageN = rxBuffer[5] << 8 | rxBuffer[6];

    return IO::CAN::CANStatus::OK;
}

IO::CAN::CANStatus GFDB::requestBatteryVoltage(uint16_t* batteryVoltage) {
    uint8_t rxBuffer[8] = {};
    IO::CAN::CANStatus result = requestData(BATTERY_VOLTAGE_REQ_CMD, rxBuffer, 8);
    if (result == IO::CAN::CANStatus::ERROR) {
        return result;
    }

    *batteryVoltage = rxBuffer[2] << 8 | rxBuffer[3];

    return IO::CAN::CANStatus::OK;
}

IO::CAN::CANStatus GFDB::requestErrorFlags(uint8_t* errorFlags) {
    return requestData(ERROR_FLAGS_REQ_CMD, errorFlags, 1);
}

IO::CAN::CANStatus GFDB::restartGFDB() {
    uint8_t payload[4] = {0x01, 0x23, 0x45, 0x67};

    return sendCommand(RESTART_CMD, payload, 4);
}

IO::CAN::CANStatus GFDB::turnExcitationPulseOff() {
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0x1F};

    return sendCommand(EXCITATION_PULSE_OFF_CMD, payload, 4);
}

IO::CAN::CANStatus GFDB::setMaxBatteryVoltage(uint16_t maxVoltage) {
    uint8_t payload[2] = {static_cast<uint8_t>(maxVoltage >> 8), static_cast<uint8_t>(maxVoltage & 0xFF)};

    return sendCommand(SET_MAX_VOLTAGE_CMD, payload, 4);
}

IO::CAN::CANStatus GFDB::requestData(uint8_t command, uint8_t* receiveBuff, uint8_t receiveSize) {
    IO::CANMessage txMessage(GFDB_ID, 1, &command, true);

    IO::CAN::CANStatus result;
    IO::CANMessage rxMessage;

    for (uint8_t i = 0; i < 10; i++) {
        result = can.transmit(txMessage);
        if (result == IO::CAN::CANStatus::ERROR)
            return result;

        result = can.receive(&rxMessage, false);

        if (result != IO::CAN::CANStatus::OK) {
            return result;
        }

        if (rxMessage.getPayload()[0] == command) {
            break;
        }
    }

    if (rxMessage.getPayload()[0] != command) {
        return IO::CAN::CANStatus::ERROR;
    }

    for (uint8_t i = 0; i < receiveSize; i++) {
        receiveBuff[i] = rxMessage.getPayload()[i];
    }

    return result;
}

IO::CAN::CANStatus GFDB::sendCommand(uint8_t command, uint8_t* payload, size_t payloadSize) {
    // TODO: This line does not look like it functions as intended
    IO::CANMessage txMessage(GFDB_ID, payloadSize, &command, true);
    return can.transmit(txMessage);
}

};// namespace GFDB
