#ifndef PID_H
#define PID_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  PID controller class
 */
typedef struct s_PIDController
{
    int32_t P; //!< Proportional gain 
    int32_t I; //!< Integral gain 
    int32_t D; //!< Derivative gain 
    int32_t output_ramp; //!< Maximum speed of change of the output value
    int32_t limit; //!< Maximum output value
    uint32_t Ts; //!< Fixed sampling time in

    int32_t error_prev; //!< last tracking error value
    int32_t output_prev;  //!< last pid output value
    int32_t integral_prev; //!< last integral component value
    unsigned long timestamp_prev; //!< Last execution timestamp
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
PIDController_init(PIDController *pid, int32_t P, int32_t I, int32_t D, int32_t ramp, int32_t limit, uint32_t sampling_time);
int32_t PIDController_update(PIDController *pid, int32_t error);
void PIDController_reset(PIDController *pid);

#endif // PID_H