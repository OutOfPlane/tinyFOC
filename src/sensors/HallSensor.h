#ifndef HALL_SENSOR_LIB_H
#define HALL_SENSOR_LIB_H

#include "../common/base/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "stdbool.h"



typedef struct s_HallSensor_ll{
       void (*init)(struct s_HallSensor_ll *ll);
       uint8_t (*read)(struct s_HallSensor_ll *ll);
       bool initOK;
       void *param;
} HallSensor_ll;

typedef struct s_HallSensor
{
  HallSensor_ll *ll;
  Sensor sensor;
  // HallSensor configuration
  int cpr;       //!< HallSensor cpr number
  int pp;       //polepairs

  // whether last step was CW (+1) or CCW (-1).
  enum Direction direction;
  enum Direction old_direction;


  // the current 3bit state of the hall sensors
  volatile uint8_t hall_state;
  // the current sector of the sensor. Each sector is 60deg electrical
  volatile int8_t electric_sector;
  // the number of electric rotations
  volatile long electric_rotations;
  // this is sometimes useful to identify interrupt issues (e.g. weak or no pullup resulting in 1000s of interrupts)
  volatile long total_interrupts;

  // variable used to filter outliers - rad/s
  float velocity_max;
  float angle_cache; // used to store the last angle value for outlier filtering

  volatile unsigned long pulse_timestamp; //!< last impulse timestamp in us
  volatile long pulse_diff;
} HallSensor;

void HallSensor_load_default(HallSensor *hs);
void HallSensor_updateState(HallSensor *hs);

#endif
