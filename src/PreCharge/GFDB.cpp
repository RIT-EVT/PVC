#include <PreCharge/GFDB.cpp>
namespace IO = EVT::core::IO;

namespace GFDB {

class GFDB {
public:
    const GFDB_ID = 0xA100101;

    GFDB(IO::CAN& can) : can(can) {};

    IO::CAN::CANStatus init() {
        return can->connect;
    }

    IO::CAN::CANStatus requestVnHighRes(uint8_t *highRes) {
        return requestData(0x60, 1, highRes, 1);
    }

    IO::CAN::CANStatus requestVpHighRes(uint8_t *highRes) {
        return requestData(0x61, 1, highRes, 1);
    }

     IO::CAN::CANStatus requestTemp(int32_t *temperature) {
        uint8_t tempBuffer[4] = {};

        IO::CAN::CANStatus result = requestData(0x80, 1, tempBuffer, 4);
        if (result != IO::CAN::CANStatus::OK) return result;

        *temperature = (tempBuffer[0] << 24) | (tempBuffer[1] << 16) | (tempBuffer[2] << 8) | tempBuffer[3];
        return IO::CAN::CANStatus::OK;
    }

    IO::CAN::CANStatus requestIsolationState(uint8_t *isoState) {
        return requestData(0xE0, 1, isoState, 8);
    }

    IO::CAN::CANStatus requestIsolationResistances(uint8_t *resistances) {
        return requestData(0xE1, 1, resistances, 8);
    }

    IO::CAN::CANStatus requestIsolationCapacitances(uint8_t *capacitances) {
        return requestData(0xE2, 1, capacitances, 8);
    }

    IO::CAN::CANStatus requestVpVn(uint16_t *voltageP, uint16_t *voltageN) {
        uint8_t rxBuffer[8] = {};

        IO::CAN::CANStatus result = requestData(0xE3, 1, rxBuffer, 8);
        if (result != IO::CAN::CANStatus::OK) return result;

        *voltageP = rxBuffer[2] << 8 | rxBuffer[3];
        *voltageN = rxBuffer[5] << 8 | rxBuffer[6];

        return result;
    }

    IO::CAN::CANStatus requestBatteryVoltage(uint16_t *batteryVoltage) {
        uint8_t rxBuffer[8] = {};
        IO::CAN::CANStatus result = requestData(0xE4, 1, rxBuffer, 8);
        if (result != IO::CAN::CANStatus::OK) return result;

        *batteryVoltage = rxBuffer[2] << 8 | rxBuffer[3];

        return result;
    }

    IO::CAN::CANStatus requestErrorFlags(uint8_t *errorFlags) {
        return requestData(0xE5, 1, errorFlags, 1);
    }

    IO::CAN::CANStatus restartGFDB() {
        // TODO
        return IO::CAN::CANStatus::OK;
    }

    IO::CAN::CANStatus setMaxDesignVoltage(uint16_t *maxVoltage) {
        // TODO
        return IO::CAN::CANStatus::OK;
    }



private:
    IO::CAN& can;

    IO::CAN::CANStatus requestData(uint8_t command, size_t payloadSize, uint8_t receiveBuff, size_t receiveSize) {
        IO::CANMessage txMessage(GFDB_ID, payloadSize, &command, false);
        IO::CANMessage rxMessage(GFDB_ID, receiveSize, receiveBuff, false);

        IO::CAN::CANStatus result = can.transmit(txMessage);
        if (result != IO::CAN::CANStatus::OK) return result;

        IO::CAN::CANStatus result = can.receive(&rxMessage, false);
        return result;

    }
};

}

