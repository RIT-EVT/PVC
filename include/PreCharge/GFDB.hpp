/**
 * SIM100 Driver for ground fault detection
 */

#ifndef PVC_GFDB_H
#define PVC_GFDB_H

namespace IO = EVT::core::IO;

namespace GFDB {

class GFDB {
public:
    const uint32_t GFDB_ID = 0xA100101;

    GFDB(IO::CAN& can);

    IO::CAN::CANStatus requestVnHighRes(uint8_t *highRes);

    IO::CAN::CANStatus requestVpHighRes(uint8_t *highRes);

     IO::CAN::CANStatus requestTemp(int8_t *temperature);

    IO::CAN::CANStatus requestIsolationState(uint8_t *isoState);

    IO::CAN::CANStatus requestIsolationResistances(uint8_t *resistances);

    IO::CAN::CANStatus requestIsolationCapacitances(uint8_t *capacitances);

    IO::CAN::CANStatus requestVpVn(uint8_t *voltageP, uint8_t *voltageN);

    IO::CAN::CANStatus requestBatteryVoltage(uint8_t *batteryVoltage);

    IO::CAN::CANStatus requestErrorFlags(uint8_t *errorFlags);

    IO::CAN::CANStatus restartGFDB();

    IO::CAN::CANStatus setMaxDesignVoltage(uint16_t *maxVoltage);

private:
    IO::CAN& can;

    IO::CAN::CANStatus requestData(uint8_t command, size_t payloadSize, uint8_t receiveBuff, size_t receiveSize);

};

}

#endif//PVC_GFDB_H