#include "FOCDriver.h"
#include "stdint.h"
#include "defaults.h"
#include "foc_utils.h"


// enable motor driver
void FOCDriver_enable(FOCDriver *driver)
{
    // enable_pin the driver - if enable_pin pin available
    FOCDriver_ll_enable(driver->params);
    // set phase state enabled
    FOCDriver_setPhaseState(driver, PHASE_ON, PHASE_ON, PHASE_ON);
    // set zero to PWM
    FOCDriver_setPwm(driver, 0, 0, 0);
}

// disable motor driver
void FOCDriver_disable(FOCDriver *driver)
{
    // set phase state to disabled
    FOCDriver_setPhaseState(driver, PHASE_OFF, PHASE_OFF, PHASE_OFF);
    // set zero to PWM
    FOCDriver_setPwm(driver, 0, 0, 0);
    // disable the driver - if enable_pin pin available
    FOCDriver_ll_disable(driver->params);
}

// init hardware pins
int FOCDriver_init(FOCDriver *driver)
{
    // ll hw
    driver->initialized = FOCDriver_ll_init(driver->params);    

    // sanity check for the voltage limit configuration
    if (!_isset(driver->voltage_limit) || driver->voltage_limit > driver->voltage_power_supply)
        driver->voltage_limit = driver->voltage_power_supply;

    // set zero to PWM
    FOCDriver_setPwm(driver, 0, 0, 0);
    return driver->initialized;
}

// Set voltage to the pwm pin
void FOCDriver_setPwm(FOCDriver *driver, FIXP Ua, FIXP Ub, FIXP Uc)
{
    // limit the voltage in driver
    Ua = _constrain(Ua, 0, driver->voltage_limit);
    Ub = _constrain(Ub, 0, driver->voltage_limit);
    Uc = _constrain(Uc, 0, driver->voltage_limit);
    // calculate duty cycle
    // limited in [0,1]
    FIXP dcA = _constrain(FIX_DIV(Ua, driver->voltage_power_supply), 0, FIX_ONE);
    FIXP dcB = _constrain(FIX_DIV(Ub, driver->voltage_power_supply), 0, FIX_ONE);
    FIXP dcC = _constrain(FIX_DIV(Uc, driver->voltage_power_supply), 0, FIX_ONE);
    // hardware specific writing
    // hardware specific function - depending on driver and mcu
    FOCDriver_ll_setPwm(driver->params, dcA, dcB, dcC);
}

// Set the phase state
// actually changing the state is only done on the next call to setPwm, and depends
// on the hardware capabilities of the driver and MCU.
void FOCDriver_setPhaseState(FOCDriver *driver, enum PhaseState sa, enum PhaseState sb, enum PhaseState sc)
{
    //TODO: add ll implementation
}

enum DriverType FOCDriver_type(FOCDriver *driver)
{
    return DriverType_BLDC;
}

void FOCDriver_load_default(FOCDriver *driver)
{    
    //low level driver
    driver->params = NULL;

    // default power-supply value
    driver->voltage_power_supply = DEF_POWER_SUPPLY_FIXP;
    driver->voltage_limit = NOT_SET;
}