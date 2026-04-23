#include "HallSensor.h"

/**
 * Updates the state and sector following an interrupt
 */
void HallSensor_updateState(HallSensor *hs) {
  uint8_t new_hall_state = hs->ll->read(hs->ll);
  // glitch avoidance #1 - sometimes we get an interrupt but pins haven't changed
  if (new_hall_state == hs->hall_state) return;

  long new_pulse_timestamp = _micros();
  hs->hall_state = new_hall_state;

  int8_t new_electric_sector = ELECTRIC_SECTORS[hs->hall_state];
  int8_t electric_sector_dif = new_electric_sector - hs->electric_sector;
  if (electric_sector_dif > 3) {
    //underflow
    hs->direction = Direction_CCW;
    hs->electric_rotations += hs->direction;
  } else if (electric_sector_dif < (-3)) {
    //overflow
    hs->direction = Direction_CW;
    hs->electric_rotations += hs->direction;
  } else {
    hs->direction = (new_electric_sector > hs->electric_sector)? Direction_CW : Direction_CCW;
  }
  hs->electric_sector = new_electric_sector;

  // glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
  if (hs->direction == hs->old_direction) {
    // not oscilating or just changed direction
    hs->pulse_diff = new_pulse_timestamp - hs->pulse_timestamp;
  } else {
    hs->pulse_diff = 0;
  }

  hs->pulse_timestamp = new_pulse_timestamp;
  hs->total_interrupts++;
  hs->old_direction = hs->direction;
}


// Sensor update function. Safely copy volatile interrupt variables into Sensor base class state variables.
void HallSensor_update(HallSensor *hs) {
  HallSensor_updateState(hs);

  hs->sensor.angle_prev_ts = hs->pulse_timestamp;
  long last_electric_rotations = hs->electric_rotations;
  int8_t last_electric_sector = hs->electric_sector;
  hs->sensor.angle_prev = ((float)((last_electric_rotations * 6 + last_electric_sector) % hs->cpr) / (float)hs->cpr) * _2PI ;
  hs->sensor.full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / hs->cpr);
}



/*
	Shaft angle calculation
  TODO: numerical precision issue here if the electrical rotation overflows the angle will be lost
*/
float HallSensor_getSensorAngle(HallSensor *hs) {
  return ((float)(hs->electric_rotations * 6 + hs->electric_sector) / (float)hs->cpr) * _2PI ;
}

/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor_getVelocity(HallSensor *hs){
  long last_pulse_timestamp = hs->pulse_timestamp;
  long last_pulse_diff = hs->pulse_diff;

  if (last_pulse_diff == 0 || ((long)(_micros() - last_pulse_timestamp) > last_pulse_diff*2) ) { // last velocity isn't accurate if too old
    return 0;
  } else {
    return hs->direction * (_2PI / (float)hs->cpr) / (last_pulse_diff / 1000000.0f);
  }

}

// HallSensor initialisation of the hardware pins 
// and calculation variables
void HallSensor_init(HallSensor *hs){
  // hall has 6 segments per electrical revolution
  hs->cpr = hs->pp * 6;

  // initialise the electrical rotations to 0
  hs->electric_rotations = 0;

  updateState();

  hs->pulse_timestamp = _micros();
}

void HallSensor_load_default(HallSensor *hs)
{
  Sensor_load_default(&hs->sensor);
  hs->sensor.init = HallSensor_init;
  hs->sensor.getVelocity = HallSensor_getVelocity;
  hs->sensor.getSensorAngle = HallSensor_getSensorAngle;
  hs->sensor.update = HallSensor_update;

}
