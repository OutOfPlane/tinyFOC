#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "common/base/FOCMotor.h"
#include "common/base/Sensor.h"
#include "common/base/FOCDriver.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "common/defaults.h"



/**
 BLDCMotor class constructor
  @param pp  pole pairs number
  @param R   motor phase resistance - [Ohm]
  @param KV  motor KV rating (1/K_bemf) - rpm/V
  @param Lq  motor q-axis inductance - [H]
  @param Ld  motor d-axis inductance - [H]
*/
void BLDCMotor_load_default(FOCMotor *motor);

#endif
