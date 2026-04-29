#include "pid.h"

PIDController_init(PIDController *pid, int32_t P, int32_t I, int32_t D, int32_t ramp, int32_t limit, uint32_t sampling_time)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->Ts = sampling_time;
    
    pid->integral_prev = 0;
    pid->output_prev = 0;
    pid->error_prev = 0;

    pid->timestamp_prev = _micros();
}

// PID controller function
int32_t PIDController_update(PIDController *pid, int32_t error){
    // initalise the elapsed time with the fixed sampling tims Ts
    uint32_t dt = pid->Ts; 
    // if Ts is not set, use adaptive sampling time
    // calculate the ellapsed time dt
    if(dt == NOT_SET){
        uint32_t timestamp_now = _micros();
        dt = (timestamp_now - pid->timestamp_prev);
        pid->timestamp_prev = timestamp_now;
    }

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    int32_t proportional = FIX_MUL(pid->P, error);
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    int32_t half_dt = dt / 2;
    int32_t error_sum = error + pid->error_prev;
    int32_t integral_increment = FIX_MUL(FIX_MUL_DIV_INT(pid->I, half_dt, 1000000), error_sum);
    int32_t integral = pid->integral_prev + integral_increment;
    // antiwindup - limit the output
    if(pid->limit != NOT_SET) integral = FIX_CONSTRAIN(integral, -pid->limit, pid->limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    int32_t error_diff = error - pid->error_prev;
    int32_t derivative = FIX_MUL_DIV_INT(FIX_MUL(pid->D, error_diff), 1000000,dt);

    // sum all the components
    int32_t output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    if(pid->limit != NOT_SET) output = FIX_CONSTRAIN(output, -pid->limit, pid->limit);

    // if output ramp defined
    if(pid->output_ramp != NOT_SET && pid->output_ramp > 0){
        // limit the acceleration by ramping the output
        int32_t output_diff = output - pid->output_prev;
        int32_t output_rate = FIX_MUL_DIV_INT(output_diff, 1000000, dt);
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + FIX_MUL_DIV_INT(pid->output_ramp, dt, 1000000);
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - FIX_MUL_DIV_INT(pid->output_ramp, dt, 1000000);
    }
    // saving for the next pass
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    return output;
}

void PIDController_reset(PIDController *pid){
    pid->integral_prev = 0;
    pid->output_prev = 0;
    pid->error_prev = 0;
}
