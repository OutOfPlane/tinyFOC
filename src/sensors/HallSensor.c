#include "common/Sensor.h"
#include "common/time_utils.h"

#if (SENSOR_TYPE == HallSensor)


// seq 1 > 5 > 4 > 6 > 2 > 3 > 1     000 001 010 011 100 101 110 111
const int8_t ELECTRIC_SECTORS[8] = {-1, 0, 4, 5, 2, 1, 3, -1};

/**
 * Updates the state and sector following an interrupt
 */
void Sensor_updateState(Sensor *hs)
{
  Sensor_ll_read(hs->params);
  // glitch avoidance #1 - sometimes we get an interrupt but pins haven't changed
  if (hs->new_hall_state == hs->hall_state)
    return;

  uint32_t new_pulse_timestamp = _micros();
  hs->hall_state = hs->new_hall_state;

  int8_t new_electric_sector = ELECTRIC_SECTORS[hs->hall_state];
  int8_t electric_sector_dif = new_electric_sector - hs->electric_sector;
  if (electric_sector_dif > 3)
  {
    // underflow
    hs->direction = Direction_CCW;
    hs->electric_rotations += hs->direction;
  }
  else if (electric_sector_dif < (-3))
  {
    // overflow
    hs->direction = Direction_CW;
    hs->electric_rotations += hs->direction;
  }
  else
  {
    hs->direction = (new_electric_sector > hs->electric_sector) ? Direction_CW : Direction_CCW;
  }
  hs->electric_sector = new_electric_sector;

  // glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
  if (hs->direction == hs->old_direction)
  {
    // not oscilating or just changed direction
    hs->pulse_diff = new_pulse_timestamp - hs->pulse_timestamp;
  }
  else
  {
    hs->pulse_diff = 0;
  }

  hs->pulse_timestamp = new_pulse_timestamp;
  hs->total_interrupts++;
  hs->old_direction = hs->direction;
  hs->newpulse = 1;
}

// Sensor update function. Safely copy volatile interrupt variables into Sensor base class state variables.
void Sensor_update(Sensor *sns)
{
  Sensor_updateState(sns);

  sns->angle_prev_ts = sns->pulse_timestamp;
  long last_electric_rotations = sns->electric_rotations;
  int8_t last_electric_sector = sns->electric_sector;
  sns->angle_prev = FIX_MUL_DIV_INT((last_electric_rotations * 6 + last_electric_sector) % sns->cpr, FIX_2PI, sns->cpr); // calculate the angle within the current rotation based on the current sector, and add the full rotations. We use modulo here to avoid numerical precision issues when the electric rotations is large, which would cause the angle to grow to the point where small position changes are no longer captured by the precision of floats. This way the angle is always between 0 and 2PI, and we rely on the electric_rotations variable to track the full rotations.
  sns->full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / sns->cpr);

  sns->angle_cache = sns->angle_prev;
  if(sns->newpulse){
    sns->newpulse = 0;
  }else{
    if(sns->pulse_diff < 50000)
    {
      uint32_t dt = _micros() - sns->pulse_timestamp;
      if(dt < 500000 && sns->pulse_diff > 0)
      {
        //try predicting the angle based on the time since the last pulse and the direction of rotation
        if(dt < sns->pulse_diff){ // only predict if we are within the expected time of the next pulse, otherwise the prediction will be more wrong than just using the last angle 
          sns->angle_cache = sns->angle_prev + sns->direction * FIX_MUL_DIV_INT(FIX_2PI, dt, sns->pulse_diff) / (sns->cpr); // add an offset to the angle based on the time since the last pulse and the direction of rotation
        }else{
          sns->angle_cache = sns->angle_prev + sns->direction * (FIX_2PI / sns->cpr);
        }
      }
    }
  }
}

/*
  Shaft angle calculation
  TODO: numerical precision issue here if the electrical rotation overflows the angle will be lost
*/
FIXP Sensor_getAngle(Sensor *sns)
{
  return sns->full_rotations * FIX_2PI + sns->angle_cache;
}

FIXP Sensor_getMechanicalAngle(Sensor *sns)
{
  return sns->angle_cache;
}

/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
FIXP Sensor_getVelocity(Sensor *sns)
{
  uint32_t last_pulse_timestamp = sns->pulse_timestamp;
  uint32_t last_pulse_diff = sns->pulse_diff;

  if (last_pulse_diff == 0 || ((_micros() - last_pulse_timestamp) > last_pulse_diff * 2))
  { // last velocity isn't accurate if too old
    return 0;
  }
  else
  {
    return sns->direction * FIX_MUL_DIV_INT(FIX_2PI, us_per_s, last_pulse_diff) / sns->cpr;
  }
}

// HallSensor initialisation of the hardware pins
// and calculation variables
void Sensor_init(Sensor *sns)
{
  // hall has 6 segments per electrical revolution
  sns->cpr = sns->pp * 6;

  // initialise the electrical rotations to 0
  sns->electric_rotations = 0;

  Sensor_updateState(sns);

  sns->pulse_timestamp = _micros();
}




#endif