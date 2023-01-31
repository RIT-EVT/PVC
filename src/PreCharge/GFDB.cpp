#ifndef PVC_GFDB_H
#define PVC_GFDB_H

namespace IO = EVT::core::IO;

namespace GFDB {

class GFDB {
public:
    const GFDB_ID = 0xA100101;

    GFDB(IO::CAN& can) : can(can) {};

    IO::CAN::CANStatus init() {
        return can->connect;
    }

    IO::CAN::CANStatus requestVnHighRes(int8_t *highRes) {
        return requestData(0x60, 1, highRes, 1);
    }

    IO::CAN::CANStatus requestVnHighRes(int8_t *highRes) {
        return requestData(0x61, 1, highRes, 1);
    }

     IO::CAN::CANStatus requestTemp(int8_t *temperature) {
        return requestData(0x80, 1, temperature, 1);
    }

    IO::CAN::CANStatus requestIsolationState(int8_t *isoState) {
        return requestData(0xE0, 1, isoState, 1);
    }

    IO::CAN::CANStatus requestIsolationResistances(uint8_t *resistances) {
        return requestData(0xE1, 1, resistances, 1);
    }

    IO::CAN::CANStatus requestIsolationCapacitances(uint8_t *capacitances) {
        return requestData(0xE2, 1, capacitances, 1);
    }

    IO::CAN::CANStatus requestVpVn(uint8_t *voltageP, uint8_t *voltageN) {
        uint8_t rxBuffer[2] = {};

        IO::CAN::CANStatus result = requestData(0xE3, 1, rxBuffer, 2);
        if (result != IO::CAN::CANStatus::OK) return result;

        *voltageP = rxBuffer[0];
        *voltageN = rxBuffer[1];

        return result;
    }

    IO::CAN::CANStatus requestBatteryVoltage(uint8_t *batteryVoltage) {
        return requestData(0xE4, 1, batteryVoltage, 1);
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

#endif//PVC_GFDB_H
