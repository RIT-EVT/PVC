#include <PreCharge/GFDB.hpp>

namespace IO = EVT::core::IO;

namespace GFDB {

    GFDB::GFDB(IO::CAN& can) : can(can){};

    IO::CAN::CANStatus GFDB::init() {
        return can.connect();
    }

    IO::CAN::CANStatus GFDB::requestVnHighRes(int32_t* highRes) {
        uint8_t rxBuffer[8] = {};
        IO::CAN::CANStatus result = requestData(0x60, rxBuffer, 8);
        if (result != IO::CAN::CANStatus::OK) return result;

        *highRes = (rxBuffer[0] << 24) | (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
        return IO::CAN::CANStatus::OK;
    }

    IO::CAN::CANStatus GFDB::requestVpHighRes(int32_t* highRes) {
        uint8_t rxBuffer[8] = {};
        IO::CAN::CANStatus result = requestData(0x61, rxBuffer, 8);
        if (result != IO::CAN::CANStatus::OK) return result;

        *highRes = (rxBuffer[0] << 24) | (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
        return IO::CAN::CANStatus::OK;
    }

    IO::CAN::CANStatus GFDB::requestTemp(int32_t* temperature) {
        uint8_t tempBuffer[4] = {};

        IO::CAN::CANStatus result = requestData(0x80, tempBuffer, 4);
        if (result != IO::CAN::CANStatus::OK) return result;

        *temperature = (tempBuffer[0] << 24) | (tempBuffer[1] << 16) | (tempBuffer[2] << 8) | tempBuffer[3];
        return IO::CAN::CANStatus::OK;
    }

    IO::CAN::CANStatus GFDB::requestIsolationState(uint8_t* isoState) {
        return requestData(0xE0, isoState, 8);
    }

    IO::CAN::CANStatus GFDB::requestIsolationResistances(uint8_t* resistances) {
        return requestData(0xE1, resistances, 8);
    }

    IO::CAN::CANStatus GFDB::requestIsolationCapacitances(uint8_t* capacitances) {
        return requestData(0xE2, capacitances, 8);
    }

    IO::CAN::CANStatus GFDB::requestVpVn(uint16_t* voltageP, uint16_t* voltageN) {
        uint8_t rxBuffer[8] = {};

        IO::CAN::CANStatus result = requestData(0xE3, rxBuffer, 8);
        if (result != IO::CAN::CANStatus::OK) return result;

        *voltageP = rxBuffer[2] << 8 | rxBuffer[3];
        *voltageN = rxBuffer[5] << 8 | rxBuffer[6];

        return IO::CAN::CANStatus::OK;
    }

    IO::CAN::CANStatus GFDB::requestBatteryVoltage(uint16_t* batteryVoltage) {
        uint8_t rxBuffer[8] = {};
        IO::CAN::CANStatus result = requestData(0xE4, rxBuffer, 8);
        if (result != IO::CAN::CANStatus::OK)
            return result;

        *batteryVoltage = rxBuffer[2] << 8 | rxBuffer[3];

        return IO::CAN::CANStatus::OK;
    }

    IO::CAN::CANStatus GFDB::requestErrorFlags(uint8_t* errorFlags) {
        return requestData(0xE5, errorFlags, 1);
    }

    IO::CAN::CANStatus GFDB::restartGFDB() {
        uint8_t payload[4] = {0x01, 0x23, 0x45, 0x67};

        return sendCommand(0xC1, payload, 4);
    }

    IO::CAN::CANStatus GFDB::turnExcitationPulseOff(uint16_t* maxVoltage) {
        uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0x1F};

        return sendCommand(0x62, payload, 4);
    }

    IO::CAN::CANStatus GFDB::setMaxBatteryVoltage(uint16_t* maxVoltage){
        uint8_t payload[2] = {*maxVoltage >> 8, *maxVoltage & 0xFF};

        return sendCommand(0x62, payload, 4);
    }

    IO::CAN::CANStatus GFDB::requestData(uint8_t command, uint8_t *receiveBuff, size_t receiveSize) {
        IO::CANMessage txMessage(GFDB_ID, 1, &command, false);
        IO::CANMessage rxMessage(GFDB_ID, receiveSize, receiveBuff, false);

        IO::CAN::CANStatus result = can.transmit(txMessage);
        if (result != IO::CAN::CANStatus::OK)
            return result;

        result = can.receive(&rxMessage, false);
        return result;
    }

    IO::CAN::CANStatus GFDB::sendCommand(uint8_t command, uint8_t *payload, size_t payloadSize) {
        IO::CANMessage txMessage(GFDB_ID, payloadSize, &command, false);
        return can.transmit(txMessage);
    }
}; // namespace GFDB
