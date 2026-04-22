#include "pid.h"

PIDController_init(PIDController *pid, float P, float I, float D, float ramp, float limit, float sampling_time)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->Ts = sampling_time;
    
    pid->integral_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->error_prev = 0.0f;

    pid->timestamp_prev = _micros();
}

// PID controller function
float PIDController_update(PIDController *pid, float error){
    // initalise the elapsed time with the fixed sampling tims Ts
    float dt = pid->Ts; 
    // if Ts is not set, use adaptive sampling time
    // calculate the ellapsed time dt
    if(!_isset(dt)){
        unsigned long timestamp_now = _micros();
        dt = (timestamp_now - pid->timestamp_prev) * 1e-6f;
        // quick fix for strange cases (micros overflow)
        if(dt <= 0 || dt > 0.5f) dt = 1e-3f;
        pid->timestamp_prev = timestamp_now;
    }

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = pid->P * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = pid->integral_prev + pid->I*dt*0.5f*(error + pid->error_prev);
    // antiwindup - limit the output
    if(_isset(pid->limit)) integral = _constrain(integral, -pid->limit, pid->limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = pid->D*(error - pid->error_prev)/dt;

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    if(_isset(pid->limit)) output = _constrain(output, -pid->limit, pid->limit);

    // if output ramp defined
    if(_isset(pid->output_ramp) && pid->output_ramp > 0){
        // limit the acceleration by ramping the output
        float output_rate = (output - pid->output_prev)/dt;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp*dt;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp*dt;
    }
    // saving for the next pass
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    return output;
}

void PIDController_reset(PIDController *pid){
    pid->integral_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->error_prev = 0.0f;
}
