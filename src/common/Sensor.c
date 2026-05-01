#include "Sensor.h"
#include "time_utils.h"


#ifndef SENSOR_TYPE

void Sensor_update(Sensor *sns) {
    FIXP val = Sensor_getSensorAngle(sns);
    if (val<0) // sensor angles are strictly non-negative. Negative values are used to signal errors.
        return; // TODO signal error, e.g. via a flag and counter
    sns->angle_prev_ts = _micros();
    FIXP d_angle = val - sns->angle_prev;
    // if overflow happened track it as full rotation
    if(_abs(d_angle) > FIX_FROM_FLOAT(0.8f*_2PI) ) sns->full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    sns->angle_prev = val;
}


 /** get current angular velocity (rad/s) */
FIXP Sensor_getVelocity(Sensor *sns) {
    // calculate sample time
    // if timestamps were unsigned, we could get rid of this section, unsigned overflow handles it correctly
    uint32_t Ts = (sns->angle_prev_ts - sns->vel_angle_prev_ts);
    if (Ts < sns->min_elapsed_time) return sns->velocity; // don't update velocity if deltaT is too small

    FIXP current_angle = 0;
    FIXP prev_angle = 0;
    // Avoid floating point precision loss for large full_rotations
    // this is likely optional
    if (sns->full_rotations == sns->vel_full_rotations) {
        current_angle = sns->angle_prev;
        prev_angle = sns->vel_angle_prev;
    } else {
        current_angle = sns->full_rotations * FIX_2PI + sns->angle_prev;
        prev_angle = sns->vel_full_rotations * FIX_2PI + sns->vel_angle_prev;
    }
    const FIXP delta_angle = current_angle - prev_angle;

    // floating point equality checks are bad, so instead we check that the angle change is very small
    if (_abs(delta_angle) > FIX_FROM_FLOAT(1e-8f)) {
        sns->velocity = FIX_MUL_DIV_INT(delta_angle, us_per_s, Ts);

        sns->vel_angle_prev = sns->angle_prev;
        sns->vel_full_rotations = sns->full_rotations;
        sns->vel_angle_prev_ts = sns->angle_prev_ts;
    }
    
    return sns->velocity;
}


void Sensor_init(Sensor *sns) {
    // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
    Sensor_getSensorAngle(sns); // call once
    _delay_us(1);
    sns->vel_angle_prev = Sensor_getSensorAngle(sns); // call again
    sns->vel_angle_prev_ts = _micros();
    _delay(1);
    Sensor_getSensorAngle(sns); // call once
    _delay_us(1);
    sns->angle_prev = Sensor_getSensorAngle(sns); // call again
    sns->angle_prev_ts = _micros();
}
#endif


FIXP default_getMechanicalAngle(Sensor *sns) {
    return sns->angle_prev;
}



FIXP default_getAngle(Sensor *sns){
    return sns->full_rotations * FIX_2PI + sns->angle_prev;
}



FIXP Sensor_getPreciseAngle(Sensor *sns) {
    return Sensor_getAngle(sns); // default implementation, can be overridden in subclasses
}



int32_t Sensor_getFullRotations(Sensor *sns) {
    return sns->full_rotations;
}



int Sensor_needsSearch(Sensor *sns) {
    return 0; // default false
}


void Sensor_load_default(Sensor *sns)
{
    sns->min_elapsed_time = 100;
    sns->velocity = 0;
    sns->angle_prev = 0;
    sns->angle_prev_ts = 0;
    sns->vel_angle_prev = 0;
    sns->full_rotations = 0;
    sns->vel_full_rotations = 0;
    sns->params = NULL;

    #if(SENSOR_TYPE == HallSensor)
    sns->hall_state = 0;
    sns->direction = Direction_UNKNOWN;
    sns->old_direction = Direction_UNKNOWN;
    sns->electric_sector = 0;
    sns->pulse_timestamp = 0;
    sns->total_interrupts = 0;
    sns->velocity_max = 1000; // default max velocity of 1000 rad
    sns->angle_cache = 0;
    sns->newpulse = 0;
    #endif
}
