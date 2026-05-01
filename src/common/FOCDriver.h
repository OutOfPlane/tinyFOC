#ifndef FOCDRIVER_H
#define FOCDRIVER_H

#include "common/foc_utils.h"

enum PhaseState
{
  PHASE_OFF = 0, // both sides of the phase are off
  PHASE_ON = 1,  // both sides of the phase are driven with PWM, dead time is applied in 6-PWM mode
  PHASE_HI = 2,  // only the high side of the phase is driven with PWM (6-PWM mode only)
  PHASE_LO = 3,  // only the low side of the phase is driven with PWM (6-PWM mode only)
};

enum DriverType
{
  DriverType_UnknownDriver = 0,
  DriverType_BLDC = 1,
  DriverType_Stepper = 2,
  DriverType_Hybrid = 3
};

bool FOCDriver_ll_init(void *params);
void FOCDriver_ll_setPwm(void *params, FIXP dcA, FIXP dcB, FIXP dcC);
void FOCDriver_ll_enable(void *params);
void FOCDriver_ll_disable(void *params);

/**
 * FOC driver class
 */
typedef struct s_FOCDriver
{
  // pointers to hardware specific parameters
  void *params;

  FIXP voltage_power_supply; //!< power supply voltage
  FIXP voltage_limit;        //!< limiting voltage set to the motor

  bool initialized; //!< true if driver was successfully initialized

  bool enable_active_high; //!< enable pin should be set to high to enable the driver (default is HIGH)

} FOCDriver;

/** Initialise hardware */
int FOCDriver_init(struct s_FOCDriver *fd);
/** get the driver type*/
enum DriverType FOCDriver_type(struct s_FOCDriver *fd);
/** Enable hardware */
void FOCDriver_enable(struct s_FOCDriver *fd);
/** Disable hardware */
void FOCDriver_disable(struct s_FOCDriver *fd);

/**
 * Set phase voltages to the harware
 *
 * @param Ua - phase A voltage
 * @param Ub - phase B voltage
 * @param Uc - phase C voltage
 */
void FOCDriver_setPwm(struct s_FOCDriver *fd, FIXP Ua, FIXP Ub, FIXP Uc);

/**
 * Set phase voltages to the harware
 *
 * @param sc - phase A state : active / disabled ( high impedance )
 * @param sb - phase B state : active / disabled ( high impedance )
 * @param sa - phase C state : active / disabled ( high impedance )
 */
void FOCDriver_setPhaseState(struct s_FOCDriver *fd, enum PhaseState sa, enum PhaseState sb, enum PhaseState sc);

void FOCDriver_load_default(FOCDriver *driver);

#endif
