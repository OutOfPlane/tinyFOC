#ifndef FOCDRIVER_H
#define FOCDRIVER_H

#include "stdbool.h"

enum PhaseState {
  PHASE_OFF = 0, // both sides of the phase are off
  PHASE_ON = 1,  // both sides of the phase are driven with PWM, dead time is applied in 6-PWM mode
  PHASE_HI = 2,  // only the high side of the phase is driven with PWM (6-PWM mode only)
  PHASE_LO = 3,  // only the low side of the phase is driven with PWM (6-PWM mode only)
};


enum DriverType{
    DriverType_UnknownDriver=0,
    DriverType_BLDC=1,
    DriverType_Stepper=2,
    DriverType_Hybrid=3
};

/**
 * FOC driver class
 */
typedef struct s_FOCDriver{
        /** Initialise hardware */
        int (*init)(struct s_FOCDriver *fd);
        /** Enable hardware */
        void (*enable)(struct s_FOCDriver *fd);
        /** Disable hardware */
        void (*disable)(struct s_FOCDriver *fd);

        /** get the driver type*/
        enum DriverType (*type)(struct s_FOCDriver *fd);

        long pwm_frequency; //!< pwm frequency value in hertz
        float voltage_power_supply; //!< power supply voltage
        float voltage_limit; //!< limiting voltage set to the motor

        bool initialized; //!< true if driver was successfully initialized
        void* params; //!< pointer to hardware specific parameters of driver

        bool enable_active_high; //!< enable pin should be set to high to enable the driver (default is HIGH)

        /**
         * Set phase voltages to the harware
         *
         * @param Ua - phase A voltage
         * @param Ub - phase B voltage
         * @param Uc - phase C voltage
         */
        void (*setPwm)(struct s_FOCDriver *fd, float Ua, float Ub, float Uc);

        /**
         * Set phase voltages to the harware
         *
         * @param sc - phase A state : active / disabled ( high impedance )
         * @param sb - phase B state : active / disabled ( high impedance )
         * @param sa - phase C state : active / disabled ( high impedance )
         */
        void (*setPhaseState)(struct s_FOCDriver *fd, enum PhaseState sa, enum PhaseState sb, enum PhaseState sc);

} FOCDriver;

void FOCDriver_load_default(FOCDriver *driver);

#endif
