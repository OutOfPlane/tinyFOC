#include "FOCDriver.h"
#include "stdint.h"
#include "common/defaults.h"
#include "common/foc_utils.h"


// enable motor driver
void default_enable(FOCDriver *driver)
{
    // enable_pin the driver - if enable_pin pin available
    if (driver->ll->enable) driver->ll->enable(driver->ll);
    // set phase state enabled
    driver->setPhaseState(driver, PHASE_ON, PHASE_ON, PHASE_ON);
    // set zero to PWM
    driver->setPwm(driver, 0, 0, 0);
}

// disable motor driver
void default_disable(FOCDriver *driver)
{
    // set phase state to disabled
    driver->setPhaseState(driver, PHASE_OFF, PHASE_OFF, PHASE_OFF);
    // set zero to PWM
    driver->setPwm(driver, 0, 0, 0);
    // disable the driver - if enable_pin pin available
    if (driver->ll->disable) driver->ll->disable(driver->ll);
}

// init hardware pins
int default_init(FOCDriver *driver)
{
    // ll hw
    if(driver->ll->init) driver->ll->init(driver->ll);
    

    // sanity check for the voltage limit configuration
    if (!_isset(driver->voltage_limit) || driver->voltage_limit > driver->voltage_power_supply)
        driver->voltage_limit = driver->voltage_power_supply;

    // set zero to PWM
    driver->setPwm(driver, 0, 0, 0);

    driver->initialized = driver->ll->initOK;
    return driver->initialized;
}

// Set voltage to the pwm pin
void default_setPwm(FOCDriver *driver, float Ua, float Ub, float Uc)
{
    FOCDriver_ll *param = driver->ll;
    // limit the voltage in driver
    Ua = _constrain(Ua, 0, driver->voltage_limit);
    Ub = _constrain(Ub, 0, driver->voltage_limit);
    Uc = _constrain(Uc, 0, driver->voltage_limit);
    // calculate duty cycle
    // limited in [0,1]
    param->dcA = _constrain(Ua / driver->voltage_power_supply, 0.0f, 1.0f);
    param->dcB = _constrain(Ub / driver->voltage_power_supply, 0.0f, 1.0f);
    param->dcC = _constrain(Uc / driver->voltage_power_supply, 0.0f, 1.0f);
    // hardware specific writing
    // hardware specific function - depending on driver and mcu
    param->setpwm(param, param->dcA, param->dcB, param->dcC);
}

// Set the phase state
// actually changing the state is only done on the next call to setPwm, and depends
// on the hardware capabilities of the driver and MCU.
void default_setPhaseState(FOCDriver *driver, enum PhaseState sa, enum PhaseState sb, enum PhaseState sc)
{
    //TODO: add ll implementation
}

void FOCDriver_load_default(FOCDriver *driver)
{
    driver->init = default_init;
    driver->setPwm = default_setPwm;
    driver->setPhaseState = default_setPhaseState;
    
    //low level driver
    driver->ll = NULL;

    // default power-supply value
    driver->voltage_power_supply = DEF_POWER_SUPPLY;
    driver->voltage_limit = NOT_SET;
}