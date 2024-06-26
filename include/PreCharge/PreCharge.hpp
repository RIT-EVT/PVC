#pragma once

#include <EVT/io/CAN.hpp>
#include <EVT/io/CANDevice.hpp>
#include <EVT/io/CANOpenMacros.hpp>
#include <EVT/io/GPIO.hpp>
#include <EVT/io/SPI.hpp>
#include <EVT/io/UART.hpp>
#include <EVT/io/pin.hpp>
#include <PreCharge/GFDB.hpp>
#include <PreCharge/dev/Contactor.hpp>
#include <PreCharge/dev/MAX22530.hpp>
#include <co_core.h>

#include <math.h>

namespace IO = EVT::core::IO;

namespace PreCharge {

/**
 * Represents the pre-charge controller used for DEV1
 */
class PreCharge : public CANDevice {
public:
    /**
     * Binary representation of the states that the pre-charge controller can be in
     */
    enum class State {
        // When the MC is powered off
        MC_OFF = 0u,
        // When the MC is powered on
        MC_ON = 1u,
        // When the MC is in emergency state
        ESTOPWAIT = 2u,
        // When the MC is precharging
        PRECHARGE = 3u,
        // When the MC is discharging
        DISCHARGE = 4u,
        // When the contactor is open
        CONT_OPEN = 5u,
        // When the contactor is closed
        CONT_CLOSE = 6u,
        // When the forward enable is disabled
        FORWARD_DISABLE = 7u
    };

    /** PVC Pinout */
    static constexpr IO::Pin UART_TX_PIN = IO::Pin::PB_6;
    static constexpr IO::Pin UART_RX_PIN = IO::Pin::PB_7;
    static constexpr IO::Pin CAN_TX_PIN = IO::Pin::PA_12;
    static constexpr IO::Pin CAN_RX_PIN = IO::Pin::PA_11;
    static constexpr IO::Pin KEY_IN_PIN = IO::Pin::PF_1;
    static constexpr IO::Pin BAT_OK_1_PIN = IO::Pin::PB_5;
    static constexpr IO::Pin BAT_OK_2_PIN = IO::Pin::PB_4;
    static constexpr IO::Pin ESTOP_IN_PIN = IO::Pin::PF_0;
    static constexpr IO::Pin PC_CTL_PIN = IO::Pin::PA_3;
    static constexpr IO::Pin DC_CTL_PIN = IO::Pin::PA_4;
    static constexpr IO::Pin APM_CTL_PIN = IO::Pin::PA_2;
    static constexpr IO::Pin CONT1_PIN = IO::Pin::PA_10;
    static constexpr IO::Pin CONT2_PIN = IO::Pin::PA_9;
    static constexpr IO::Pin SPI_CS = IO::Pin::PB_0;
    static constexpr IO::Pin SPI_MOSI = IO::Pin::PA_7;
    static constexpr IO::Pin SPI_MISO = IO::Pin::PA_6;
    static constexpr IO::Pin SPI_SCK = IO::Pin::PA_5;
    static constexpr IO::Pin SPI_INT = IO::Pin::PA_8;

    enum class PinStatus {
        DISABLE = 0u,
        ENABLE = 1u,
        HOLD = 2u
    };

    enum class PrechargeStatus {
        OK = 0u,
        DONE = 1u,
        ERROR = 2u
    };

    enum class PVCStatus {
        PVC_OK = 0u,
        PVC_ERROR = 1u
    };

    uint8_t Statusword;//8

    uint64_t InputVoltage; //16
    uint16_t OutputVoltage;//16
    uint16_t BasePTemp;    //16

    uint64_t changePDO;
    uint64_t cyclicPDO;

    static constexpr uint16_t DISCHARGE_DELAY = 5250;      // 5.25 seconds
    static constexpr uint16_t FORWARD_DISABLE_DELAY = 5000;// 5 seconds

    static constexpr uint8_t MIN_PACK_VOLTAGE = 70;

    static constexpr uint8_t CONST_R = 30;
    static constexpr float CONST_C = 0.014;

    /**
     * Number of attempts that will be made to check the STO status
     * before failing
     */
    static constexpr uint16_t MAX_STO_ATTEMPTS = 25;

    /**
     * Utility variable which can be used to count the number of attempts that
     * was made to complete a certain actions.
     *
     * For example, this is used for checking the STO N
     * number of times before failing
     */
    uint16_t numAttemptsMade = 0;

    /**
     * Constructor for pre-charge state machine
     *
     * @param[in] key GPIO for motorcycle key
     * @param[in] batteryOne GPIO for battery ok signal
     * @param[in] batteryTwo GPIO for battery ok signal
     * @param[in] eStop GPIO for motorcycle e-stop
     * @param[in] pc GPIO for precharge contactor
     * @param[in] dc GPIO for discharge contactor
     * @param[in] cont Driver for main contactor
     * @param[in] apm GPIO for apm control
     * @param[in] forward GPIO for forward enable
     * @param[in] gfdb GPIO for gfdb fault signal
     * @param[in] can can instance for CANopen
     */
    PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
              IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, Contactor cont,
              IO::GPIO& apm, GFDB::GFDB& gfdb, IO::CAN& can, MAX22530 MAX);

    /**
     * The node ID used to identify the device on the CAN network.
     */
    static constexpr uint8_t NODE_ID = 10;

    /**
     * Handler running the pre-charge state switching
     *
     * @return the current status of the PVC, either PVC_OK or PVC_Error
     */
    PVCStatus handle();

    /**
     * Get the value of STO (Safe to Operate)
     *
     * TODO: Update documentation
     * STO=1 when BATTERY_1_OK=1 AND BATTERY_2_OK=1 AND EMERGENCY_STOP=1
     *
     * @return value of STO, 0 or 1
     */
    void getSTO();

    /**
     * Get the value of the Precharge compared against the ideal precharge voltage curve
     *
     * DONE signifies that the precharge system has reached the target voltage without any errors
     * OK signifies that the precharge system is not at target voltage and there are no errors
     * ERROR signifies that the precharge system has either lagged behind or overshot the ideal curve
     *
     * @return int value of PrechargeStatus enum
    */
    int getPrechargeStatus();

    /**
     * Requests and handles the isolation state from the SIM100
    */
    void checkGFDB();

    /**
     * Get the current expected voltage based on the current precharge time
     *
     * @return float value of expected voltage in Volts
    */
    uint16_t solveForVoltage(uint16_t pack_voltage, uint64_t delta_time);

    /**
     * Get the state of MC_KEY_IN
     *
     */
    void getMCKey();

    /**
     * Get the state of all IO instances and update IOStatus
     */
    void getIOStatus();

    /**
     * Set the Precharge state
     *
     * @param state 0 = precharge disabled, 1 = precharge enabled
     */
    void setPrecharge(PinStatus state);

    /**
     * Set the Discharge state
     *
     * @param state 0 = discharge disabled, 1 = discharge enabled
     */
    void setDischarge(PinStatus state);

    /**
     * Set the APM state
     *
     * @param state 0 = APM disabled, 1 = APM enabled
     */
    void setAPM(PinStatus state);

    /**
     * Handles when the MC is powered off.
     *
     * State: State::MC_OFF
     */
    void mcOffState();

    /**
     * Handles when the MC is powered on.
     *
     * State: State::MC_ON
     */
    void mcOnState();

    /**
     * Handles when an unsafe condition is detected while the MC is off.
     *
     * State: State::ESTOPWAIT
     */
    void eStopState();

    /**
     * Handles when precharge is to occur.
     *
     * State: State::PRECHARGE
     */
    void prechargeState();

    /**
     * Handles when discharge is to occur.
     *
     * State: State::DISCHARGE
     */
    void dischargeState();

    /**
     * Handles when the contactor is to be open.
     *
     * State: State::CONT_OPEN
     */
    void contOpenState();

    /**
     * Handles when the contactor is to be closed.
     *
     * State: State::CONT_CLOSE
     */
    void contCloseState();

    /**
     * Handles when the Forward Enable is to be disabled.
     *
     * State: State::FORWARD_DISABLE
     */
    void forwardDisableState();

    CO_OBJ_T* getObjectDictionary() override;

    uint8_t getNumElements() override;

    uint8_t getNodeID() override;

private:
    /** GPIO instance to monitor KEY_IN */
    IO::GPIO& key;
    /** GPIO instance to monitor BATTERY_1_OK */
    IO::GPIO& batteryOne;
    /** GPIO instance to monitor BATTERY_2_OK */
    IO::GPIO& batteryTwo;
    /** GPIO instance to monitor ESTOP_STATUS */
    IO::GPIO& eStop;
    /** GPIO instance to toggle PC_CTL */
    IO::GPIO& pc;
    /** GPIO instance to toggle DC_CTL */
    IO::GPIO& dc;
    /** Contactor instance to control the main contactor */
    Contactor cont;
    /** GPIO instance to toggle APM_CTL */
    IO::GPIO& apm;
    /** GPIO instance to toggle FW_EN_CTL */
    //    IO::GPIO& forward;
    /** GFDB instance to handle isolation status*/
    GFDB::GFDB& gfdb;
    /** CAN instance to handle CANOpen processes*/
    IO::CAN& can;

    MAX22530 MAX;

    IO::GPIO::State keyInStatus;
    IO::GPIO::State stoStatus;
    IO::GPIO::State batteryOneOkStatus;
    IO::GPIO::State batteryTwoOkStatus;
    IO::GPIO::State eStopActiveStatus;
    IO::GPIO::State pcStatus;
    IO::GPIO::State dcStatus;
    uint8_t contStatus;
    uint8_t voltStatus;
    IO::GPIO::State apmStatus;

    uint8_t gfdStatus;
    uint32_t lastPrechargeTime;

    // Status bit to indicate a precharge error
    // Key must be cycled (on->off->on) to resume state machine
    uint8_t cycle_key;

    State state;
    State prevState;
    uint64_t state_start_time;
    int in_precharge;
    uint8_t initVolt;

    /**
     * Handles the sending of a CAN message upon each state change.
    */
    void sendChangePDO();

    /**
     * Have to know the size of the object dictionary for initialization
     * process.
     */
    static constexpr uint8_t OBJECT_DICTIONARY_SIZE = 22;

    /**
     * The object dictionary itself. Will be populated by this object during
     * construction.
     *
     * The plus one is for the special "end of dictionary" marker.
     */
    CO_OBJ_T objectDictionary[OBJECT_DICTIONARY_SIZE + 1] = {

        MANDATORY_IDENTIFICATION_ENTRIES_1000_1014,
        HEARTBEAT_PRODUCER_1017(2000),
        IDENTITY_OBJECT_1018,
        SDO_CONFIGURATION_1200,

        // TPDO0 settings
        // 0: The TPDO number, default 0
        // 1: The COB-ID used by TPDO0, provided as a function of the TPDO number
        // 2: How the TPO is triggered, default to manual triggering
        // 3: Inhibit time, defaults to 0
        // 5: Timer trigger time in 1ms units, 0 will disable the timer based triggering

        TRANSMIT_PDO_SETTINGS_OBJECT_18XX(0x00, TRANSMIT_PDO_TRIGGER_TIMER, TRANSMIT_PDO_INHIBIT_TIME_DISABLE, 2000),

        // TPDO0 mapping, determines the PDO messages to send when TPDO1 is triggered
        // 0: The number of PDO message associated with the TPDO
        // 1: Link to the first PDO message
        // n: Link to the nth PDO message

        TRANSMIT_PDO_MAPPING_START_KEY_1AXX(0x00, 0x01),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(0x00, 0x01, PDO_MAPPING_UNSIGNED8),

        // User defined data, this will be where we put elements that can be
        // accessed via SDO and depeneding on configuration PDO
        DATA_LINK_START_KEY_21XX(0, 0x01),
        DATA_LINK_21XX(0x00, 0x01, CO_TUNSIGNED8, &state),

        // End of dictionary marker
        CO_OBJ_DICT_ENDMARK,
    };
};

}// namespace PreCharge