#include "BLDCDriver6PWM.h"
#include "stdint.h"

void BLDCDriver6PWM_set_pins(FOCDriver *driver, int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
{
  BLDCDriver6PWM *param = driver->params;
  // Pin initialization
  param->pwmA_h = phA_h;
  param->pwmB_h = phB_h;
  param->pwmC_h = phC_h;
  param->pwmA_l = phA_l;
  param->pwmB_l = phB_l;
  param->pwmC_l = phC_l;

  // enable_pin pin
  param->enable_pin = en;

  // dead zone initial - 2%
  param->dead_zone = 0.02f;

  // default power-supply value
  driver->voltage_power_supply = DEF_POWER_SUPPLY;
  driver->voltage_limit = NOT_SET;
  driver->pwm_frequency = NOT_SET;
}

// enable motor driver
void BLDCDriver6PWM_enable(FOCDriver *driver)
{
  BLDCDriver6PWM *param = driver->params;
  // enable_pin the driver - if enable_pin pin available
  if (_isset(param->enable_pin))
    digitalWrite(param->enable_pin, driver->enable_active_high);
  // set phase state enabled
  driver->setPhaseState(driver, PHASE_ON, PHASE_ON, PHASE_ON);
  // set zero to PWM
  driver->setPwm(driver, 0, 0, 0);
}

// disable motor driver
void BLDCDriver6PWM_disable(FOCDriver *driver)
{
  BLDCDriver6PWM *param = driver->params;
  // set phase state to disabled
  driver->setPhaseState(driver, PHASE_OFF, PHASE_OFF, PHASE_OFF);
  // set zero to PWM
  driver->setPwm(driver, 0, 0, 0);
  // disable the driver - if enable_pin pin available
  if (_isset(param->enable_pin))
    digitalWrite(param->enable_pin, !driver->enable_active_high);
}

// init hardware pins
int BLDCDriver6PWM_init(FOCDriver *driver)
{
  BLDCDriver6PWM *param = driver->params;
  // PWM pins
  _pinMode(param->pwmA_h, OUTPUT);
  _pinMode(param->pwmB_h, OUTPUT);
  _pinMode(param->pwmC_h, OUTPUT);
  _pinMode(param->pwmA_l, OUTPUT);
  _pinMode(param->pwmB_l, OUTPUT);
  _pinMode(param->pwmC_l, OUTPUT);
  if (_isset(param->enable_pin))
    _pinMode(param->enable_pin, OUTPUT);

  // sanity check for the voltage limit configuration
  if (!_isset(driver->voltage_limit) || driver->voltage_limit > driver->voltage_power_supply)
    driver->voltage_limit = driver->voltage_power_supply;

  // set phase state to disabled
  param->phase_state[0] = PHASE_OFF;
  param->phase_state[1] = PHASE_OFF;
  param->phase_state[2] = PHASE_OFF;

  // set zero to PWM
  param->dc_a = param->dc_b = param->dc_c = 0;

  // configure 6pwm
  // hardware specific function - depending on driver and mcu
  param->llparam = _configure6PWM(driver->pwm_frequency, param->dead_zone, param->pwmA_h, param->pwmA_l, param->pwmB_h, param->pwmB_l, param->pwmC_h, param->pwmC_l);
  driver->initialized = (param->llparam != SIMPLEFOC_DRIVER_INIT_FAILED);
  return driver->initialized;
}

// Set voltage to the pwm pin
void BLDCDriver6PWM_setPwm(FOCDriver *driver, float Ua, float Ub, float Uc)
{
  BLDCDriver6PWM *param = driver->params;
  // limit the voltage in driver
  Ua = _constrain(Ua, 0, driver->voltage_limit);
  Ub = _constrain(Ub, 0, driver->voltage_limit);
  Uc = _constrain(Uc, 0, driver->voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  param->dc_a = _constrain(Ua / driver->voltage_power_supply, 0.0f, 1.0f);
  param->dc_b = _constrain(Ub / driver->voltage_power_supply, 0.0f, 1.0f);
  param->dc_c = _constrain(Uc / driver->voltage_power_supply, 0.0f, 1.0f);
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle6PWM(param->dc_a, param->dc_b, param->dc_c, param->phase_state, param->llparam);
}

// Set the phase state
// actually changing the state is only done on the next call to setPwm, and depends
// on the hardware capabilities of the driver and MCU.
void BLDCDriver6PWM_setPhaseState(FOCDriver *driver, enum PhaseState sa, enum PhaseState sb, enum PhaseState sc)
{
  BLDCDriver6PWM *param = driver->params;
  param->phase_state[0] = sa;
  param->phase_state[1] = sb;
  param->phase_state[2] = sc;
}

void BLDCDriver6PWM_load_default(BLDCDriver6PWM *driver)
{
  FOCDriver_load_default(&(driver->focDriver));
  driver->llparam = NULL;
  driver->focDriver.params = &driver; //point back to us
  driver->focDriver.init = BLDCDriver6PWM_init;
  driver->focDriver.setPwm = BLDCDriver6PWM_setPwm;
  driver->focDriver.setPhaseState = BLDCDriver6PWM_setPhaseState;
}
