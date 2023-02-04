#pragma once

#include <Canopen/co_core.h>
#include <EVT/io/CAN.hpp>
#include <EVT/io/GPIO.hpp>

namespace IO = EVT::core::IO;

namespace PreCharge {

/**
 * Represents the pre-charge controller used for DEV1
 */
class PreCharge {
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

    enum class PinStatus {
        DISABLE = 0u,
        ENABLE = 1u
    };

    uint16_t IOStatus;     //16 //keyStatus:stoStatus:batteryOneOkStatus:batteryTwoOkStatus:eStopStatus:
                           //pcStatus:dcStatus:contStatus:apmStatus:forwardStatus
    uint8_t Statusword;    //8
    uint64_t InputVoltage; //16
    uint16_t OutputVoltage;//16
    uint16_t BasePTemp;    //16

    uint64_t changePDO;
    uint64_t cyclicPDO;

    static constexpr uint16_t PRECHARGE_DELAY = 5250;      // 5.25 seconds
    static constexpr uint16_t DISCHARGE_DELAY = 5250;      // 5.25 seconds
    static constexpr uint16_t FORWARD_DISABLE_DELAY = 5000;// 5 seconds

    /**
     * Number of attempts that will be made to check the STO status
     * before failing
     */
    static constexpr uint16_t MAX_STO_ATTEMPTS = 50;

    /**
     * Utility variable which can be used to count the number of attempts that
     * was made to complete a certain actions.
     *
     * For example, this is used for checking the STO N
     * number of times before failing
     */
    uint16_t numAttemptsMade;

    /**
     * Constructor for pre-charge state machine
     * 
     */
    PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
              IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, IO::GPIO& cont,
              IO::GPIO& apm, IO::GPIO& forward, IO::CAN& can);

    /**
     * The node ID used to identify the device on the CAN network.
     */
    static constexpr uint8_t NODE_ID = 0x02;

    /**
     * Handler running the pre-charge state switching
     * 
     */
    void handle();

    /**
    * Get the value of STO (Safe to Operate)
    * 
    * STO=1 when BATTERY_1_OK=1 AND BATTERY_2_OK=1 AND EMERGENCY_STOP=1
    * 
    * @return value of STO, 0 or 1
    */
    void getSTO();

    /**
     * Get the state of MC_KEY_IN
     * 
     */
    void getMCKey();

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
     * Toggle the state of the Main Contactor
     * 
     * @param state 0 = contactor open, 1 = contactor closed
     */
    void setContactor(PinStatus state);

    /**
     * Set the Forward state
     * 
     * @param state 0 = forward disabled, 1 = forward enabled
     */
    void setForward(PinStatus state);

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

    /**
     * Get a pointer to the start of the CANopen object dictionary.
     *
     * @return Pointer to the start of the CANopen object dictionary.
     */
    CO_OBJ_T* getObjectDictionary();

    /**
     * Get the number of elements in the object dictionary.
     *
     * @return The number of elements in the object dictionary
     */
    uint16_t getObjectDictionarySize();

    void sendChangePDO();

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
    /** GPIO instance to toggle CONT_CTL */
    IO::GPIO& cont;
    /** GPIO instance to toggle APM_CTL */
    IO::GPIO& apm;
    /** GPIO instance to toggle FW_EN_CTL */
    IO::GPIO& forward;

    IO::CAN& can;

    IO::GPIO::State keyInStatus;
    IO::GPIO::State stoStatus;
    IO::GPIO::State batteryOneOkStatus;
    IO::GPIO::State batteryTwoOkStatus;
    IO::GPIO::State eStopActiveStatus;
    IO::GPIO::State pcStatus;
    IO::GPIO::State dcStatus;
    IO::GPIO::State contStatus;
    IO::GPIO::State apmStatus;
    IO::GPIO::State forwardStatus;

    State state;
    uint64_t state_start_time;

    /**
     * Have to know the size of the object dictionary for initialization
     * process.
     */
    static constexpr uint8_t OBJECT_DICTIONARY_SIZE = 19;

    /**
     * The object dictionary itself. Will be populated by this object during
     * construction.
     *
     * The plus one is for the special "end of dictionary" marker.
     */
    CO_OBJ_T objectDictionary[OBJECT_DICTIONARY_SIZE + 1] = {
        // Sync ID, defaults to 0x80
        {CO_KEY(0x1005, 0, CO_UNSIGNED32 | CO_OBJ_D__R_), nullptr, (uintptr_t) 0x80},

        // Information about the hardware, hard coded sample values for now
        // 1: Vendor ID
        // 2: Product Code
        // 3: Revision Number
        // 4: Serial Number
        {
            .Key = CO_KEY(0x1018, 1, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0x10,
        },
        {
            .Key = CO_KEY(0x1018, 2, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0x11,
        },
        {
            .Key = CO_KEY(0x1018, 3, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0x12,
        },
        {
            .Key = CO_KEY(0x1018, 4, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0x13,
        },

        // SDO CAN message IDS.
        // 1: Client -> Server ID, default is 0x600 + NODE_ID
        // 2: Server -> Client ID, default is 0x580 + NODE_ID
        {
            .Key = CO_KEY(0x1200, 1, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0x600 + NODE_ID,
        },
        {
            .Key = CO_KEY(0x1200, 2, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0x580 + NODE_ID,
        },

        // TPDO0 settings
        // 0: The TPDO number, default 0
        // 1: The COB-ID used by TPDO0, provided as a function of the TPDO number
        // 2: How the TPO is triggered, default to manual triggering
        // 3: Inhibit time, defaults to 0
        // 5: Timer trigger time in 1ms units, 0 will disable the timer based triggering
        {
            .Key = CO_KEY(0x1800, 0, CO_UNSIGNED8 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0,
        },
        {
            .Key = CO_KEY(0x1800, 1, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) CO_COBID_TPDO_DEFAULT(0),
        },
        {
            .Key = CO_KEY(0x1800, 2, CO_UNSIGNED8 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0xFE,
        },
        {
            .Key = CO_KEY(0x1800, 3, CO_UNSIGNED16 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 0,
        },
        {
            .Key = CO_KEY(0x1800, 5, CO_UNSIGNED16 | CO_OBJ_D__R_),
            .Type = CO_TEVENT,
            .Data = (uintptr_t) 2000,
        },

        // TPDO0 mapping, determines the PDO messages to send when TPDO1 is triggered
        // 0: The number of PDO message associated with the TPDO
        // 1: Link to the first PDO message
        // n: Link to the nth PDO message
        {
            .Key = CO_KEY(0x1A00, 0, CO_UNSIGNED8 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = (uintptr_t) 1},
        {
            .Key = CO_KEY(0x1A00, 1, CO_UNSIGNED32 | CO_OBJ_D__R_),
            .Type = nullptr,
            .Data = CO_LINK(0x2100, 0, 8)// Link to sample data position in dictionary
        },

        // User defined data, this will be where we put elements that can be
        // accessed via SDO and depeneding on configuration PDO
        {
            .Key = CO_KEY(0x2100, 0, CO_UNSIGNED8 | CO_OBJ___PRW),
            .Type = nullptr,
            .Data = (uintptr_t) &state,
        },

        // End of dictionary marker
        CO_OBJ_DIR_ENDMARK,
    };
};

}// namespace PreCharge