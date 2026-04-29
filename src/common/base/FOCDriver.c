#include "FOCDriver.h"
#include "stdint.h"
#include "common/defaults.h"
#include "common/foc_utils.h"


// enable motor driver
static void default_enable(FOCDriver *driver)
{
    // enable_pin the driver - if enable_pin pin available
    if (driver->ll->enable) driver->ll->enable(driver->ll);
    // set phase state enabled
    driver->setPhaseState(driver, PHASE_ON, PHASE_ON, PHASE_ON);
    // set zero to PWM
    driver->setPwm(driver, 0, 0, 0);
}

// disable motor driver
static void default_disable(FOCDriver *driver)
{
    // set phase state to disabled
    driver->setPhaseState(driver, PHASE_OFF, PHASE_OFF, PHASE_OFF);
    // set zero to PWM
    driver->setPwm(driver, 0, 0, 0);
    // disable the driver - if enable_pin pin available
    if (driver->ll->disable) driver->ll->disable(driver->ll);
}

// init hardware pins
static int default_init(FOCDriver *driver)
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
static void default_setPwm(FOCDriver *driver, FIXP Ua, FIXP Ub, FIXP Uc)
{
    FOCDriver_ll *param = driver->ll;
    // limit the voltage in driver
    Ua = _constrain(Ua, 0, driver->voltage_limit);
    Ub = _constrain(Ub, 0, driver->voltage_limit);
    Uc = _constrain(Uc, 0, driver->voltage_limit);
    // calculate duty cycle
    // limited in [0,1]
    param->dcA = _constrain(FIX_DIV(Ua, driver->voltage_power_supply), 0, FIX_ONE);
    param->dcB = _constrain(FIX_DIV(Ub, driver->voltage_power_supply), 0, FIX_ONE);
    param->dcC = _constrain(FIX_DIV(Uc, driver->voltage_power_supply), 0, FIX_ONE);
    // hardware specific writing
    // hardware specific function - depending on driver and mcu
    param->setpwm(param, param->dcA, param->dcB, param->dcC);
}

// Set the phase state
// actually changing the state is only done on the next call to setPwm, and depends
// on the hardware capabilities of the driver and MCU.
static void default_setPhaseState(FOCDriver *driver, enum PhaseState sa, enum PhaseState sb, enum PhaseState sc)
{
    //TODO: add ll implementation
}

static enum DriverType default_drivertype(FOCDriver *driver)
{
    return DriverType_BLDC;
}

void FOCDriver_load_default(FOCDriver *driver)
{
    driver->init = default_init;
    driver->setPwm = default_setPwm;
    driver->setPhaseState = default_setPhaseState;
    driver->enable = default_enable;
    driver->disable = default_disable;
    driver->type = default_drivertype;
    
    //low level driver
    driver->ll = NULL;

    // default power-supply value
    driver->voltage_power_supply = DEF_POWER_SUPPLY_FIXP;
    driver->voltage_limit = NOT_SET;
}