#ifndef PID_H
#define PID_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  PID controller class
 */
typedef struct s_PIDController
{
    FIXP P; //!< Proportional gain 
    FIXP I; //!< Integral gain 
    FIXP D; //!< Derivative gain 
    FIXP output_ramp; //!< Maximum speed of change of the output value
    FIXP limit; //!< Maximum output value
    uint32_t Ts; //!< Fixed sampling time in

    FIXP error_prev; //!< last tracking error value
    FIXP output_prev;  //!< last pid output value
    FIXP integral_prev; //!< last integral component value
    uint32_t timestamp_prev; //!< Last execution timestamp
} PIDController;

/**
 *  
 * @param P - Proportional gain 
 * @param I - Integral gain
 * @param D - Derivative gain 
 * @param ramp - Maximum speed of change of the output value
 * @param limit - Maximum output value
 * @param sampling_time - sampling time in us
 * 
 * @note If sampling time is not set the filter will measure the 
 *       elapsed time between each call. If yes it will consider it fixed. 
 * @note Sampling time can be changed dynamically as well by modifying the 
 *       variable Ts in runtime.
 */
PIDController_init(PIDController *pid, FIXP P, FIXP I, FIXP D, FIXP ramp, FIXP limit, uint32_t sampling_time);
FIXP PIDController_update(PIDController *pid, FIXP error);
void PIDController_reset(PIDController *pid);

#endif // PID_H