#pragma once

#include <EVT/io/GPIO.hpp>

namespace IO = EVT::core::IO;

namespace pre_charge {

/**
 * Represents the pre-charge controller used for DEV1
 */
class pre_charge {
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
        CONT_CLOSE = 6u
    };

    /**
     * Constructor for pre-charge state machine
     * 
     */
    pre_charge(IO::GPIO& key, IO::GPIO& pc, IO::GPIO& dc, IO::GPIO& cont,
                IO::GPIO& batteryOne, IO::GPIO& batteryTwo, IO::GPIO& eStop);

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
    int getSTO();

    /**
     * Get the state of MC_KEY_IN
     * 
     */
    int getMCKey();

    /**
     * Set the Precharge state
     * 
     * @param state 0 = precharge disabled, 1 = precharge enabled
     */
    void setPrecharge(int state);

    /**
     * Set the Discharge state
     * 
     * @param state 0 = discharge disabled, 1 = discharge enabled
     */
    void setDischarge(int state);

    /**
     * Toggle the state of the Main Contactor
     * 
     * @param state 0 = contactor open, 1 = contactor closed
     */
    void setContactor(int state);

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

private:
    /** GPIO instance to monitor KEY_IN */
    IO::GPIO& key;
    /** GPIO instance to monitor PC_CTL */
    IO::GPIO& pc;
    /** GPIO instance to monitor DC_CTL */
    IO::GPIO& dc;
    /** GPIO instance to monitor CONT_CTL */
    IO::GPIO& cont;
    /** GPIO instance to monitor BATTERY_1_OK */
    IO::GPIO& batteryOne
    /** GPIO instance to monitor BATTERY_2_OK */
    IO::GPIO& batteryTwo
    /** GPIO instance to monitor ESTOP_STATUS */
    IO::GPIO& eStop

    IO::GPIO::State keyStatus;
    IO::GPIO::State stoStatus;
    IO::GPIO::State batteryOneStatus;
    IO::GPIO::State batteryTwoStatus;
    IO::GPIO::State eStopStatus;

    State state;
};

}// namespace pre_charge