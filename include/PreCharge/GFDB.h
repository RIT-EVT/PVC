/**
 * SIM100 Driver for ground fault detection
 */

#ifndef PVC_GFDB_H
#define PVC_GFDB_H

namespace IO = EVT::core::IO;

namespace GFDB {

class GFDB {
public:
    const GFDB_ID = 0xA100101;

    GFDB(IO::CAN& can);

    IO::CAN::CANStatus init();

    IO::CAN::CANStatus requestTemp(int8_t *temperature);

    IO::CAN::CANStatus requestVnVp(uint8_t *voltageN, uint8_t *voltageP);

    IO::CAN::CANStatus requestBatteryVoltage(uint8_t *batteryVoltage);

    IO::CAN::CANStatus restartGFDB();

    IO::CAN::CANStatus setMaxDesignVoltage(uint16_t *maxVoltage);



private:
    IO::CAN& can;

    IO::CAN::CANStatus transmitMessage(uint8_t *payload, size_t len);

    IO::CAN::CANStatus receiveMessage(uint8_t *payload, size_t len);
};

}

#endif//PVC_GFDB_H
