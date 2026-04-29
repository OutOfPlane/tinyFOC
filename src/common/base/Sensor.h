#ifndef SENSOR_H
#define SENSOR_H

#include "common/foc_utils.h"

/**
 *  Direction structure
 */
enum Direction {
    Direction_CW      = 1,  // clockwise
    Direction_CCW     = -1, // counter clockwise
    Direction_UNKNOWN = 0   // not yet known or invalid state
};


/**
 *  Pullup configuration structure
 */
enum Pullup {
    USE_INTERN = 0x00, //!< Use internal pullups
    USE_EXTERN = 0x01  //!< Use external pullups
};

/**
 * Sensor abstract class defintion
 * 
 * This class is purposefully kept simple, as a base for all kinds of sensors. Currently we have
 * Encoders, Magnetic Encoders and Hall Sensor implementations. This base class extracts the
 * most basic common features so that a FOC driver can obtain the data it needs for operation.
 * 
 * To implement your own sensors, create a sub-class of this class, and implement the getSensorAngle()
 * method. getSensorAngle() returns a FIXP value, in radians, representing the current shaft angle in the
 * range 0 to 2*PI (one full turn). 
 * 
 * To function correctly, the sensor class update() method has to be called sufficiently quickly. Normally,
 * the BLDCMotor's loopFOC() function calls it once per iteration, so you must ensure to call loopFOC() quickly
 * enough, both for correct motor and sensor operation.
 * 
 * The Sensor base class provides an implementation of getVelocity(), and takes care of counting full
 * revolutions in a precise way, but if you wish you can additionally override these methods to provide more
 * optimal implementations for your hardware.
 * 
 */
typedef struct s_Sensor{
        /**
         * Get mechanical shaft angle in the range 0 to 2PI. This value will be as precise as possible with
         * the hardware. Base implementation uses the values returned by update() so that 
         * the same values are returned until update() is called again.
         */
        FIXP (*getMechanicalAngle)(struct s_Sensor *sns);

        /**
         * Get current position (in rad) including full rotations and shaft angle.
         * Base implementation uses the values returned by update() so that the same
         * values are returned until update() is called again.
         * Note that this value has limited precision as the number of rotations increases,
         * because the limited precision of FIXP can't capture the large angle of the full 
         * rotations and the small angle of the shaft angle at the same time.
         */
        FIXP (*getAngle)(struct s_Sensor *sns);
        
        /** 
         * On architectures supporting it, this will return a double precision position value,
         * which should have improved precision for large position values.
         * Base implementation uses the values returned by update() so that the same
         * values are returned until update() is called again.
         */
        FIXP (*getPreciseAngle)(struct s_Sensor *sns);

        /** 
         * Get current angular velocity (rad/s)
         * Can be overridden in subclasses. Base implementation uses the values 
         * returned by update() so that it only makes sense to call this if update()
         * has been called in the meantime.
         */
        FIXP (*getVelocity)(struct s_Sensor *sns);

        /**
         * Get the number of full rotations
         * Base implementation uses the values returned by update() so that the same
         * values are returned until update() is called again. 
         */
        int32_t (*getFullRotations)(struct s_Sensor *sns);

        /**
         * Updates the sensor values by reading the hardware sensor.
         * Some implementations may work with interrupts, and not need this.
         * The base implementation calls getSensorAngle(), and updates internal
         * fields for angle, timestamp and full rotations.
         * This method must be called frequently enough to guarantee that full
         * rotations are not "missed" due to infrequent polling.
         * Override in subclasses if alternative behaviours are required for your
         * sensor hardware.
         */
        void (*update)(struct s_Sensor *sns);

        /** 
         * returns 0 if it does need search for absolute zero
         * 0 - magnetic sensor (& encoder with index which is found)
         * 1 - ecoder with index (with index not found yet)
         */
        int (*needsSearch)(struct s_Sensor *sns);

        /**
         * Minimum time between updates to velocity. If time elapsed is lower than this, the velocity is not updated.
         */
        uint32_t min_elapsed_time; // default is 100 microseconds, or 10kHz

        /** 
         * Get current shaft angle from the sensor hardware, and 
         * return it as a FIXP in radians, in the range 0 to 2PI.
         * 
         * This method is pure virtual and must be implemented in subclasses.
         * Calling this method directly does not update the base-class internal fields.
         * Use update() when calling from outside code.
         */
        FIXP (*getSensorAngle)(struct s_Sensor *sns);
        /**
         * Call Sensor::init() from your sensor subclass's init method if you want smoother startup
         * The base class init() method calls getSensorAngle() several times to initialize the internal fields
         * to current values, ensuring there is no discontinuity ("jump from zero") during the first calls
         * to sensor.getAngle() and sensor.getVelocity()
         */
        void (*init)(struct s_Sensor *sns);

        // velocity calculation variables
        FIXP velocity;
        FIXP angle_prev; // result of last call to getSensorAngle(), used for full rotations and velocity
        uint32_t angle_prev_ts; // timestamp of last call to getAngle, used for velocity
        FIXP vel_angle_prev; // angle at last call to getVelocity, used for velocity
        uint32_t vel_angle_prev_ts; // last velocity calculation timestamp
        int32_t full_rotations; // full rotation tracking
        int32_t vel_full_rotations; // previous full rotation value for velocity calculation
        void *parent; // pointer to parent struct, e.g. HallSensor, MagneticSensor, Encoder, etc. Can be used to access specific fields of the subclass from the base class methods.
}Sensor;

void Sensor_load_default(Sensor *sns);

#endif
