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
    /**
     * Constructor for the GFDB Class
     *
     * @param can CAN driver the GFDB will use
     */
    GFDB(IO::CAN& can);

    /**
     * Connects the GFDB's CAN to the network
     *
     * @return CAN Connection status
     */
    IO::CAN::CANStatus init();

    /**
     * Requests the high resolution negative voltage from the GFDB
     *
     * @param highRes[out] The high resolution negative voltage
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestVnHighRes(int32_t* highRes);

    /**
     * Requests the high resolution positive voltage from the GFDB
     *
     * @param highRes[out] The high resolution positive voltage
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestVpHighRes(int32_t* highRes);

    /**
     * Requests the temperature from the GFDB
     *
     * @param temperature[out] The temperature
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestTemp(int32_t* temperature);

    /**
     * Requests the isolation state from the GFDB
     *
     * @param isoState[out] The isolation state
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestIsolationState(uint8_t* isoState);

    /**
     * Requests the isolation resistances from the GFDB
     *
     * @param isoState[out] The isolation resistances
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestIsolationResistances(uint8_t* resistances);

    /**
     * Requests the isolation capacitances from the GFDB
     *
     * @param isoState[out] The isolation capacitances
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestIsolationCapacitances(uint8_t* capacitances);

    /**
     * Requests both the positive and negative voltages from the GFDB
     *
     * @param voltageP[out] The positive voltage
     * @param voltageN[out] The negative voltage
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestVpVn(uint16_t* voltageP, uint16_t* voltageN);

    /**
     * Requests the battery voltage from the GFDB
     * @param isoState[out] The isolation state
     *
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestBatteryVoltage(uint16_t* batteryVoltage);

    /**
     * Requests any error flags from the GFDB
     * @param isoState[out] The isolation state
     *
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestErrorFlags(uint8_t* errorFlags);

    /**
     * Restarts the GFDB
     * @param isoState[out] The isolation state
     *
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus restartGFDB();

    /**
     * Turns of excitation pulse
     * @param isoState[out] The isolation state
     *
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus turnExcitationPulseOff(uint16_t* maxVoltage);

    /**
     * Sets the max battery voltage of the GFDB
     * @param isoState[in] The isolation state
     *
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus setMaxBatteryVoltage(uint16_t maxVoltage);

private:
    IO::CAN& can;
    const uint32_t GFDB_ID = 0xA100101;

    /**
     * Helper method for requesting data from the GFDB through CAN
     *
     * @param command[in] Command to send to the GFDB
     * @param receiveBuff[out] Buffer to store data in
     * @param receiveSize[in] Size of receive buffer
     * @return The status of the CAN call
     */
    IO::CAN::CANStatus requestData(uint8_t command, uint8_t* receiveBuff, size_t receiveSize);

    /**
     * Helper method for sending a command to the GFDB through CAN
     *
     * @param command[in] Command to send to the GFDB
     * @param payload[in] Payload of the data for the GFDB
     * @param payloadSize[in] Size of the payload
     * @return The status of the CAN call
     */
    IO::CAN::CANStatus sendCommand(uint8_t command, uint8_t* payload, size_t payloadSize);
};

}// namespace GFDB

#endif//_EVT_GFDB_H
