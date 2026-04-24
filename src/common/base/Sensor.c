#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"



static void default_update(Sensor *sns) {
    float val = sns->getSensorAngle(sns);
    if (val<0) // sensor angles are strictly non-negative. Negative values are used to signal errors.
        return; // TODO signal error, e.g. via a flag and counter
    sns->angle_prev_ts = _micros();
    float d_angle = val - sns->angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) sns->full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    sns->angle_prev = val;
}


 /** get current angular velocity (rad/s) */
static float default_getVelocity(Sensor *sns) {
    // calculate sample time
    // if timestamps were unsigned, we could get rid of this section, unsigned overflow handles it correctly
    float Ts = (sns->angle_prev_ts - sns->vel_angle_prev_ts)*1e-6f;
    if (Ts < 0.0f) {    // handle micros() overflow - we need to reset vel_angle_prev_ts
        sns->vel_angle_prev = sns->angle_prev;
        sns->vel_full_rotations = sns->full_rotations;
        sns->vel_angle_prev_ts = sns->angle_prev_ts;
        return sns->velocity;
    }
    if (Ts < sns->min_elapsed_time) return sns->velocity; // don't update velocity if deltaT is too small

    float current_angle = 0.0f;
    float prev_angle = 0.0f;
    // Avoid floating point precision loss for large full_rotations
    // this is likely optional
    if (sns->full_rotations == sns->vel_full_rotations) {
        current_angle = sns->angle_prev;
        prev_angle = sns->vel_angle_prev;
    } else {
        current_angle = (float) sns->full_rotations * _2PI + sns->angle_prev;
        prev_angle = (float) sns->vel_full_rotations * _2PI + sns->vel_angle_prev;
    }
    const float delta_angle = current_angle - prev_angle;

    // floating point equality checks are bad, so instead we check that the angle change is very small
    if (fabsf(delta_angle) > 1e-8f) {
        sns->velocity = delta_angle / Ts;

        sns->vel_angle_prev = sns->angle_prev;
        sns->vel_full_rotations = sns->full_rotations;
        sns->vel_angle_prev_ts = sns->angle_prev_ts;
    }
    
    return sns->velocity;
}



static void default_init(Sensor *sns) {
    // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
    sns->getSensorAngle(sns); // call once
    _delay_us(1);
    sns->vel_angle_prev = sns->getSensorAngle(sns); // call again
    sns->vel_angle_prev_ts = _micros();
    _delay(1);
    sns->getSensorAngle(sns); // call once
    _delay_us(1);
    sns->angle_prev = sns->getSensorAngle(sns); // call again
    sns->angle_prev_ts = _micros();
}


static float default_getMechanicalAngle(Sensor *sns) {
    return sns->angle_prev;
}



static float default_getAngle(Sensor *sns){
    return (float)sns->full_rotations * _2PI + sns->angle_prev;
}



static double default_getPreciseAngle(Sensor *sns) {
    return (double)sns->full_rotations * (double)_2PI + (double)sns->angle_prev;
}



static int32_t default_getFullRotations(Sensor *sns) {
    return sns->full_rotations;
}



static int default_needsSearch(Sensor *sns) {
    return 0; // default false
}


void Sensor_load_default(Sensor *sns)
{
    sns->getMechanicalAngle = default_getMechanicalAngle;
    sns->getAngle = default_getAngle;
    sns->getPreciseAngle = default_getPreciseAngle;
    sns->getVelocity = default_getVelocity;
    sns->getFullRotations = default_getFullRotations;
    sns->update = default_update;
    sns->needsSearch = default_needsSearch;
    sns->getSensorAngle = NULL;
    sns->init = default_init;

    sns->min_elapsed_time = 100e-6f;
    sns->velocity = 0;
    sns->angle_prev = 0;
    sns->angle_prev_ts = 0;
    sns->vel_angle_prev = 0;
    sns->full_rotations = 0;
    sns->vel_full_rotations = 0;
}
