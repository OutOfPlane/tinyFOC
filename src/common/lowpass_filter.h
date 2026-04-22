#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


#include "time_utils.h"
#include "foc_utils.h"

/**
 *  Low pass filter class
 */
typedef struct s_LowPassFilter
{
    float Tf; //!< Low pass filter time constant
    float Ts; //!< Fixed sampling time (optional default NOT_SET)

    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
} LowPassFilter;

/**
 * @param Tf - Low pass filter time constant
 * @param Ts - Filter sampling time
 * 
 * @note If sampling time Ts is not set the filter will measure the 
 *       elapsed time between each call. 
 * @note Ts can be changed dynamically as well by modifying the 
 *       variable in runtime.
 */
LowPassFilter_init(LowPassFilter *lpf, float Tf, float Ts);
float LowPassFilter_update(LowPassFilter *lpf, float x);

#endif // LOWPASS_FILTER_H