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

    IO::CAN::CANStatus requestTemp(int8_t *temperature) {
    }

    IO::CAN::CANStatus requestVnVp(uint16_t *voltageN, uint16_t *voltageP) {
        uint8_t payload = 0xE3;
        uint8_t rxBuffer[2] = {};
        IO::CANMessage txMessage(GFDB_ID, 1, &payload, false);
        IO::CANMessage rxMessage(GFDB_ID, 2, rxBuffer, false);

        IO::CAN::CANStatus result = can.transmit(txMessage);
        if (result != IO::CAN::CANStatus::OK) return result;

        IO::CAN::CANStatus result = can.receive(&rxMessage, false);
        if (result != IO::CAN::CANStatus::OK) return result;

        *voltageN = rxBuffer[0];
        *voltageP = rxBuffer[1];

        return result;
    }

    IO::CAN::CANStatus requestBatteryVoltage(uint8_t *batteryVoltage) {
        return requestData(&payload, 1, batteryVoltage, 1);
    }

    IO::CAN::CANStatus restartGFDB() {
    }

    IO::CAN::CANStatus setMaxDesignVoltage(uint16_t *maxVoltage) {
    }



private:
    IO::CAN& can;

    IO::CAN::CANStatus requestData(uint8_t *payload, size_t payloadSize, uint8_t receiveBuff, size_t receiveSize) {
        IO::CANMessage txMessage(GFDB_ID, payloadSize, payload, false);
        IO::CANMessage rxMessage(GFDB_ID, receiveSize, receiveBuff, false);

        IO::CAN::CANStatus result = can.transmit(txMessage);
        if (result != IO::CAN::CANStatus::OK) return result;

        IO::CAN::CANStatus result = can.receive(&rxMessage, false);
        return result;

    }
};

}

#endif//PVC_GFDB_H
