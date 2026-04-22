#include "FOCDriver.h"
#include "stdint.h"

void FOCDriver_load_default(FOCDriver *driver)
{
    driver->init = NULL;
}