#ifndef BLDCDriver6PWM_h
#define BLDCDriver6PWM_h

#include "../common/base/FOCDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
 6 pwm bldc driver class
*/
typedef struct s_BLDCDriver6PWM
{
  FOCDriver focDriver;
  // hardware variables
  int pwmA_h, pwmA_l; //!< phase A pwm pin number
  int pwmB_h, pwmB_l; //!< phase B pwm pin number
  int pwmC_h, pwmC_l; //!< phase C pwm pin number
  int enable_pin;     //!< enable pin number

  //duty cycle
  float dc_a, dc_b, dc_c;

  float dead_zone; //!< a percentage of dead-time(zone) (both high and low side in low) for each pwm cycle [0,1]

  enum PhaseState phase_state[3]; //!< phase state (active / disabled)

  void *llparam;
} BLDCDriver6PWM;

void BLDCDriver6PWM_load_default(BLDCDriver6PWM *driver);
/**
  BLDCDriver class constructor
  @param phA_h A phase pwm pin
  @param phA_l A phase pwm pin
  @param phB_h B phase pwm pin
  @param phB_l A phase pwm pin
  @param phC_h C phase pwm pin
  @param phC_l A phase pwm pin
  @param en enable pin (optional input)
*/
void BLDCDriver6PWM_set_pins(FOCDriver *driver, int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en);


#endif
