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
    }

    IO::CAN::CANStatus requestBatteryVoltage(uint8_t *batteryVoltage) {
        uint8_t payload = 0xE4;
        IO::CANMessage txMessage(GFDB_ID, 1, &payload, false);
        IO::CANMessage rxMessage(GFDB_ID, 1, batteryVoltage, false);

        IO::CAN::CANStatus result = can.transmit(txMessage);
        if (result != IO::CAN::CANStatus::OK) return result;

        IO::CAN::CANStatus result = can.receive(&rxMessage, false);
        return result;
    }

    IO::CAN::CANStatus restartGFDB() {
        return IO::CANMessage transmit_message(GFDB_ID, len, payload, false);
    }

    IO::CAN::CANStatus setMaxDesignVoltage(uint16_t *maxVoltage) {
        return IO::CANMessage transmit_message(GFDB_ID, len, payload, false);
    }



private:
    IO::CAN& can;
};

}

#endif//PVC_GFDB_H
