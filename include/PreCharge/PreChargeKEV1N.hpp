#pragma once

#include "PreCharge/dev/Contactor.hpp"
#include "PreCharge/PreChargeBase.hpp"
#include <Canopen/co_core.h>
#include <EVT/io/CAN.hpp>
#include <EVT/io/GPIO.hpp>
#include <EVT/io/SPI.hpp>
#include <EVT/io/UART.hpp>
#include <EVT/io/pin.hpp>
#include <PreCharge/GFDB.hpp>
#include <PreCharge/dev/MAX22530.hpp>


#include <math.h>

namespace IO = EVT::core::IO;

namespace PreCharge {

/**
 * Represents the pre-charge controller used for KEV1N
 */
class PreChargeKEV1N : public PreChargeBase {
public:
    enum class PVCStatus {
        PVC_OP = 0u,
        PVC_PRE_OP = 1u,
        PVC_NONE = 2u
    };

    static PVCStatus pvcStatus;

    static constexpr uint16_t PRECHARGE_DELAY = 2000;      // 2 seconds

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
    PreChargeKEV1N(IO::GPIO& key, IO::GPIO& batteryOne, IO::GPIO& batteryTwo,
              IO::GPIO& eStop, IO::GPIO& pc, IO::GPIO& dc, Contactor cont,
              IO::GPIO& apm, GFDB::GFDB& gfdb, IO::CAN& can, MAX22530 MAX);

    /**
     * Handler running the pre-charge state switching
     */
    PVCStatus handle(IO::UART& uart);

    /**
     * Get the value of STO (Safe to Operate)
     *
     * TODO: Update documentation
     * STO=1 when BATTERY_1_OK=1 AND BATTERY_2_OK=1 AND EMERGENCY_STOP=1
     *
     * @return value of STO, 0 or 1
     */
    void getSTO() override;

    /**
     * Get the state of MC_KEY_IN
     * 
     */
    void getMCKey() override;

    /**
     * Handles when the MC is powered on.
     * 
     * State: State::MC_ON
     */
    void mcOnState() override;

    /**
     * Handles when precharge is to occur.
     * 
     * State: State::PRECHARGE
     */
    void prechargeState() override;

    /**
     * Handles when the contactor is to be closed.
     * 
     * State: State::CONT_CLOSE
     */
    void contCloseState() override;

private:
    uint8_t pre_charged;
};

}// namespace PreCharge
