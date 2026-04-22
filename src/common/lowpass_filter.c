#include "lowpass_filter.h"

LowPassFilter_init(LowPassFilter *lpf, float time_constant, float sampling_time)
{
    lpf->Tf = time_constant;
    lpf->Ts = sampling_time;
    lpf->y_prev = 0;
    lpf->timestamp_prev = _micros();
}


float LowPassFilter_update(LowPassFilter *lpf, float x)
{
    // initalise the elapsed time with the fixed sampling tims Ts
    float dt = lpf->Ts; 
    // if Ts is not set, use adaptive sampling time
    // calculate the ellapsed time dt
    if(!_isset(dt)){
        unsigned long timestamp = _micros();
        dt = (timestamp - lpf->timestamp_prev)*1e-6f;

        if (dt < 0.0f ) dt = 1e-3f;
        else if(dt > 0.3f) {
            lpf->y_prev = x;
            lpf->timestamp_prev = timestamp;
            return x;
        }
        lpf->timestamp_prev = timestamp;
    }

    // calculate the first order filer
    float alpha = lpf->Tf/(lpf->Tf + dt);
    float y = alpha*lpf->y_prev + (1.0f - alpha)*x;

    // save the variables for the future steps
    lpf->y_prev = y;
    return y;
}
