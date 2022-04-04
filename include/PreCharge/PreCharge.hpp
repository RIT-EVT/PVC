#pragma once

#include <string>

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

    static constexpr uint16_t PRECHARGE_DELAY = 5; //TODO: make actually 5 tau
    static constexpr uint16_t DISCHARGE_DELAY = 10; //TODO: make actually 10 tau
    static constexpr uint16_t FORWARD_DISABLE_DELAY = 5000; // 5 seconds

    /**
     * Constructor for pre-charge state machine
     * 
     */
    PreCharge(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
              IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, IO::GPIO& cont,
              IO::GPIO& apm, IO::GPIO& forward);

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

    IO::GPIO::State keyInStatus;
    IO::GPIO::State stoStatus;
    IO::GPIO::State batteryOneOkStatus;
    IO::GPIO::State batteryTwoOkStatus;
    IO::GPIO::State eStopActiveStatus;

    State state;
    uint64_t state_start_time;
};

}// namespace PreCharge