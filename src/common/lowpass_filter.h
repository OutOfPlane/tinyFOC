#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "foc_utils.h"

/**
 *  Low pass filter class
 */
typedef struct s_LowPassFilter
{
    uint32_t Tf; //!< Low pass filter time constant
    uint32_t Ts; //!< Fixed sampling time (optional default NOT_SET)

    uint32_t timestamp_prev;  //!< Last execution timestamp
    FIXP y_prev; //!< filtered value in previous execution step 
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
void LowPassFilter_init(LowPassFilter *lpf, uint32_t Tf_us, uint32_t Ts_us);
FIXP LowPassFilter_update(LowPassFilter *lpf, FIXP x);

#endif // LOWPASS_FILTER_H