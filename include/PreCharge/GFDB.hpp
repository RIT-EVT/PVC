#ifndef _EVT_GFDB_H
#define _EVT_GFDB_H

#include <EVT/io/CAN.hpp>
#include <EVT/utils/time.hpp>
#include <stddef.h>

namespace IO = EVT::core::IO;

namespace GFDB {

/**
 * All possible commands for the GFDB
 */
enum GFDB_COMMAND {
    VN_HIGH_RES_CMD = 0x60,
    VP_HIGH_RES_CMD = 0x61,
    TEMP_REQ_CMD = 0x80,
    ISO_STATE_REQ_CMD = 0xE0,
    ISO_RESISTANCES_REQ_CMD = 0xE1,
    ISO_CAPACITANCES_REQ_CMD = 0xE2,
    VP_VN_REQ_CMD = 0xE3,
    BATTERY_VOLTAGE_REQ_CMD = 0xE4,
    ERROR_FLAGS_REQ_CMD = 0xE5,

    RESTART_CMD = 0xC1,
    EXCITATION_PULSE_OFF_CMD = 0x62,
    SET_MAX_VOLTAGE_CMD = 0xF0
};


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
     * @param[in] can CAN driver the GFDB will use
     */
    GFDB(IO::CAN& can);

    /**
     * Requests the high resolution negative voltage from the GFDB
     *
     * @param[out]  highRes The high resolution negative voltage
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestVoltagePositiveHighRes(int32_t* highRes);

    /**
     * Requests the high resolution positive voltage from the GFDB
     *
     * @param[out] highRes The high resolution positive voltage
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestVoltageNegativeHighRes(int32_t* highRes);

    /**
     * Requests the temperature from the GFDB
     *
     * @param[out] temperature The temperature
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestTemp(int32_t* temperature);

    /**
     * Requests the isolation state from the GFDB
     *
     * @param[out] isoState The isolation state
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestIsolationState(uint8_t* isoState);

    /**
     * Requests the isolation resistances from the GFDB
     *
     * @param[out] isoState The isolation resistances
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestIsolationResistances(uint8_t* resistances);

    /**
     * Requests the isolation capacitances from the GFDB
     *
     * @param[out] isoState The isolation capacitances
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestIsolationCapacitances(uint8_t* capacitances);

    /**
     * Requests both the positive and negative voltages from the GFDB
     *
     * @param[out] voltageP The positive voltage
     * @param[out] voltageN The negative voltage
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestVoltagesPositiveNegative(uint16_t* voltageP, uint16_t* voltageN);

    /**
     * Requests the battery voltage from the GFDB
     *
     * @param[out] isoState The isolation state
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestBatteryVoltage(uint16_t* batteryVoltage);

    /**
     * Requests any error flags from the GFDB
     *
     * @param[out] isoState The isolation state
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus requestErrorFlags(uint8_t* errorFlags);

    /**
     * Restarts the GFDB
     *
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus restartGFDB();

    /**
     * Turns off excitation pulse
     *
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus turnExcitationPulseOff();

    /**
     * Sets the max battery voltage of the GFDB
     *
     * @param[in] isoState The isolation state
     * @return CAN Status of the request
     */
    IO::CAN::CANStatus setMaxBatteryVoltage(uint16_t maxVoltage);

private:
    IO::CAN& can;
    const uint32_t GFDB_ID = 0xA100101;

    /**
     * Helper method for requesting data from the GFDB through CAN
     *
     * @param[in] command Command to send to the GFDB
     * @param[out] receiveBuff Buffer to store data in
     * @param[in] receiveSize Size of receive buffer
     * @return The status of the CAN call
     */
    IO::CAN::CANStatus requestData(uint8_t command, uint8_t* receiveBuff, size_t receiveSize);

    /**
     * Helper method for sending a command to the GFDB through CAN
     *
     * @param[in] command Command to send to the GFDB
     * @param[in] payload Payload of the data for the GFDB
     * @param[in] payloadSize Size of the payload
     * @return The status of the CAN call
     */
    IO::CAN::CANStatus sendCommand(uint8_t command, uint8_t* payload, size_t payloadSize);
};

}// namespace GFDB

#endif//_EVT_GFDB_H
