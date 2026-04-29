#include "lowpass_filter.h"

LowPassFilter_init(LowPassFilter *lpf, uint32_t time_constant_us, uint32_t sampling_time_us)
{
    lpf->Tf = time_constant_us;
    lpf->Ts = sampling_time_us;
    lpf->y_prev = 0;
    lpf->timestamp_prev = _micros();
}


int32_t LowPassFilter_update(LowPassFilter *lpf, int32_t x)
{
    // initalise the elapsed time with the fixed sampling tims Ts
    uint32_t dt = lpf->Ts; 
    // if Ts is not set, use adaptive sampling time
    // calculate the ellapsed time dt
    if(dt == NOT_SET){
        uint32_t timestamp = _micros();
        dt = (timestamp - lpf->timestamp_prev);
        

        if(dt > 65535){ // if more than 65.535 ms passed since last update, reset the filter to avoid spikes{
            lpf->y_prev = x;
            lpf->timestamp_prev = timestamp;
            return x;
        }
        lpf->timestamp_prev = timestamp;
    }

    // calculate the first order filer
    // alpha = Tf/(Tf + dt)
    int32_t alpha = FIX_DIV(lpf->Tf, lpf->Tf + dt);
    // y = alpha*y_prev + (1 - alpha)*x
    int32_t one_minus_alpha = FIX_ONE - alpha;
    int32_t y = FIX_MUL(alpha, lpf->y_prev) + FIX_MUL(one_minus_alpha, x);

    // save the variables for the future steps
    lpf->y_prev = y;
    return y;
}
