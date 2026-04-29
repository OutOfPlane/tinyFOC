#include "HallSensor.h"

// seq 1 > 5 > 4 > 6 > 2 > 3 > 1     000 001 010 011 100 101 110 111
const int8_t ELECTRIC_SECTORS[8] = {-1, 0, 4, 5, 2, 1, 3, -1};

/**
 * Updates the state and sector following an interrupt
 */
void HallSensor_updateState(HallSensor *hs)
{
  uint8_t new_hall_state = hs->ll->read(hs->ll);
  // glitch avoidance #1 - sometimes we get an interrupt but pins haven't changed
  if (new_hall_state == hs->hall_state)
    return;

  unsigned long new_pulse_timestamp = _micros();
  hs->hall_state = new_hall_state;

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
void HallSensor_update(Sensor *sns)
{
  HallSensor *hs = (HallSensor *)sns->parent;
  HallSensor_updateState(hs);

  hs->sensor.angle_prev_ts = hs->pulse_timestamp;
  long last_electric_rotations = hs->electric_rotations;
  int8_t last_electric_sector = hs->electric_sector;
  hs->sensor.angle_prev = ((float)((last_electric_rotations * 6 + last_electric_sector) % hs->cpr) / (float)hs->cpr) * _2PI;
  hs->sensor.full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / hs->cpr);

  hs->angle_cache = hs->sensor.angle_prev;
  if(hs->newpulse){
    hs->newpulse = 0;
  }else{
    if(hs->pulse_diff < 50000)
    {
      unsigned long dt = _micros() - hs->pulse_timestamp;
      if(dt < 500000 && hs->pulse_diff > 0)
      {
        //try predicting the angle based on the time since the last pulse and the direction of rotation
        if(dt < hs->pulse_diff){ // only predict if we are within the expected time of the next pulse, otherwise the prediction will be more wrong than just using the last angle 
          hs->angle_cache = hs->sensor.angle_prev + hs->direction * (_2PI / (float)hs->cpr) * ((float)(dt) / (float)(hs->pulse_diff)); // add an offset to the angle based on the time since the last pulse and the direction of rotation
        }else{
          hs->angle_cache = hs->sensor.angle_prev + hs->direction * (_2PI / (float)hs->cpr);
        }
      }
    }
  }
}

/*
  Shaft angle calculation
  TODO: numerical precision issue here if the electrical rotation overflows the angle will be lost
*/
float HallSensor_getSensorAngle(Sensor *sns)
{
  HallSensor *hs = (HallSensor *)sns->parent;
  return (float)sns->full_rotations * _2PI + hs->angle_cache;
}

static float HallSensor_getMechanicalAngle(Sensor *sns)
{
  HallSensor *hs = (HallSensor *)sns->parent;
  return hs->angle_cache;
}

/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor_getVelocity(Sensor *sns)
{
  HallSensor *hs = (HallSensor *)sns->parent;
  long last_pulse_timestamp = hs->pulse_timestamp;
  long last_pulse_diff = hs->pulse_diff;

  if (last_pulse_diff == 0 || ((long)(_micros() - last_pulse_timestamp) > last_pulse_diff * 2))
  { // last velocity isn't accurate if too old
    return 0;
  }
  else
  {
    return hs->direction * (_2PI / (float)hs->cpr) / (last_pulse_diff / 1000000.0f);
  }
}

// HallSensor initialisation of the hardware pins
// and calculation variables
void HallSensor_init(Sensor *sns)
{
  HallSensor *hs = (HallSensor *)sns->parent;
  // hall has 6 segments per electrical revolution
  hs->cpr = hs->pp * 6;

  // initialise the electrical rotations to 0
  hs->electric_rotations = 0;

  HallSensor_updateState(hs);

  hs->pulse_timestamp = _micros();

  LowPassFilter_init(&hs->LPF_adaptive, .1f, NOT_SET); // default cutoff frequency of 10ms, will be adapted based on velocity
}

void HallSensor_load_default(HallSensor *hs)
{
  Sensor_load_default(&hs->sensor);
  hs->sensor.init = HallSensor_init;
  hs->sensor.getVelocity = HallSensor_getVelocity;
  hs->sensor.getSensorAngle = HallSensor_getSensorAngle;
  hs->sensor.update = HallSensor_update;
  hs->sensor.getMechanicalAngle = HallSensor_getMechanicalAngle;
  hs->ll = NULL;
  hs->hall_state = 0;
  hs->direction = Direction_UNKNOWN;
  hs->old_direction = Direction_UNKNOWN;
  hs->electric_sector = 0;
  hs->sensor.parent = hs;
  hs->pulse_timestamp = 0;
  hs->total_interrupts = 0;
  hs->velocity_max = 1000; // default max velocity of 1000 rad
  hs->angle_cache = 0;
  hs->newpulse = 0;
}
