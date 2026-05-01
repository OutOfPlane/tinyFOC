#include "pid.h"
#include "time_utils.h"

void PIDController_init(PIDController *pid, FIXP P, FIXP I, FIXP D, FIXP ramp, FIXP output_max, uint32_t sampling_time)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->output_max = output_max;
    pid->Ts = sampling_time;
    
    pid->integral_prev = 0;
    pid->output_prev = 0;
    pid->error_prev = 0;

    pid->timestamp_prev = _micros();
}

// PID controller function
FIXP PIDController_update(PIDController *pid, FIXP error){
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
    FIXP proportional = FIX_MUL(pid->P, error);
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    uint32_t half_dt = dt / 2;
    FIXP error_sum = error + pid->error_prev;
    FIXP integral_increment = FIX_MUL(FIX_MUL_DIV_INT(pid->I, half_dt, us_per_s), error_sum);
    FIXP integral = pid->integral_prev + integral_increment;
    // antiwindup - limit the output
    if(pid->output_max != NOT_SET) integral = FIX_CONSTRAIN(integral, -pid->output_max, pid->output_max);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    FIXP error_diff = error - pid->error_prev;
    FIXP derivative = FIX_MUL_DIV_INT(FIX_MUL(pid->D, error_diff), us_per_s,dt);

    // sum all the components
    FIXP output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    if(pid->output_max != NOT_SET) output = FIX_CONSTRAIN(output, -pid->output_max, pid->output_max);

    // if output ramp defined
    if(pid->output_ramp != NOT_SET && pid->output_ramp > 0){
        // limit the acceleration by ramping the output
        FIXP output_diff = output - pid->output_prev;
        FIXP output_rate = FIX_MUL_DIV_INT(output_diff, us_per_s, dt);
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + FIX_MUL_DIV_INT(pid->output_ramp, dt, us_per_s);
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - FIX_MUL_DIV_INT(pid->output_ramp, dt, us_per_s);
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
