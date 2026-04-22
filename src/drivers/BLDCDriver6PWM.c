#include "BLDCDriver6PWM.h"

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
  dc_a = dc_b = dc_c = 0;

  // configure 6pwm
  // hardware specific function - depending on driver and mcu
  params = _configure6PWM(pwm_frequency, dead_zone, pwmA_h, pwmA_l, pwmB_h, pwmB_l, pwmC_h, pwmC_l);
  initialized = (params != SIMPLEFOC_DRIVER_INIT_FAILED);
  return params != SIMPLEFOC_DRIVER_INIT_FAILED;
}

// Set voltage to the pwm pin
void BLDCDriver6PWM_setPwm(FOCDriver *driver, float Ua, float Ub, float Uc)
{
  // limit the voltage in driver
  Ua = _constrain(Ua, 0, voltage_limit);
  Ub = _constrain(Ub, 0, voltage_limit);
  Uc = _constrain(Uc, 0, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle6PWM(dc_a, dc_b, dc_c, phase_state, params);
}

// Set the phase state
// actually changing the state is only done on the next call to setPwm, and depends
// on the hardware capabilities of the driver and MCU.
void BLDCDriver6PWM_setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc)
{
  phase_state[0] = sa;
  phase_state[1] = sb;
  phase_state[2] = sc;
}

void BLDCDriver6PWM_load_default(FOCDriver *driver)
{
}
