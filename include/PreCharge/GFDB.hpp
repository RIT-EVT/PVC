

#ifndef _EVT_GFDB_H
#define _EVT_GFDB_H

#include <EVT/io/CAN.hpp>
#include <EVT/utils/time.hpp>
#include <stddef.h>

namespace IO = EVT::core::IO;

namespace GFDB {

/**
 * SIM100 Driver for ground fault detection
 * https://sendyne.com/Datasheets/Sendyne%20SIM100MOD%20Datasheet%20V1.5.pdf
 * https://sendyne.com/Datasheets/SIM100_CAN_protocol_v0.8a.pdf
 */
class GFDB {
public:
    GFDB(IO::CAN& can);

    IO::CAN::CANStatus init();

    IO::CAN::CANStatus requestVnHighRes(int32_t *highRes);

    IO::CAN::CANStatus requestVpHighRes(int32_t *highRes);

     IO::CAN::CANStatus requestTemp(int32_t *temperature);

    IO::CAN::CANStatus requestIsolationState(uint8_t *isoState);

    IO::CAN::CANStatus requestIsolationResistances(uint8_t *resistances);

    IO::CAN::CANStatus requestIsolationCapacitances(uint8_t *capacitances);

    IO::CAN::CANStatus requestVpVn(uint16_t *voltageP, uint16_t *voltageN);

    IO::CAN::CANStatus requestBatteryVoltage(uint16_t *batteryVoltage);

    IO::CAN::CANStatus requestErrorFlags(uint8_t *errorFlags);

    IO::CAN::CANStatus restartGFDB();

    IO::CAN::CANStatus turnExcitationPulseOff(uint16_t *maxVoltage);

    IO::CAN::CANStatus setMaxBatteryVoltage(uint16_t *maxVoltage);

private:
    IO::CAN& can;
    const uint32_t GFDB_ID = 0xA100101;


    IO::CAN::CANStatus requestData(uint8_t command, uint8_t *receiveBuff, size_t receiveSize);

    IO::CAN::CANStatus sendCommand(uint8_t command, uint8_t *payload, size_t payloadSize);
};

}

#endif//_EVT_GFDB_H
