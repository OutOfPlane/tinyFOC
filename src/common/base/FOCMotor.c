#include "common/base/FOCMotor.h"

// time measuring function
// It filters the value using low pass filtering alpha = 0.1
static void updateTime(uint32_t *elapsed_time_filetered, uint32_t *elapsed_time, uint32_t *last_timestamp_us, float alpha)
{
    uint32_t now = _micros();
    *elapsed_time = now - *last_timestamp_us;
    *elapsed_time_filetered = (uint32_t)((1 - alpha) * *elapsed_time_filetered + alpha * *elapsed_time);
    *last_timestamp_us = now;
}

static void linkCustomMotionControl(FOCMotor *motor, float (*controlMethod)(FOCMotor *motor))
{
    motor->customMotionControlCallback = controlMethod;
}

static void updateLoopFOCTime(FOCMotor *motor)
{
    updateTime(&(motor->loopfoc_time_us), &(motor->last_loopfoc_time_us), &(motor->last_loopfoc_timestamp_us), 0.1f);
}

static void updateMotionControlTime(FOCMotor *motor)
{
    updateTime(&(motor->move_time_us), &(motor->last_move_time_us), &(motor->last_move_timestamp_us), 0.1f);
}

static void default_init(FOCMotor *motor)
{
}

/**
    Sensor linking method
*/
static void default_linkSensor(FOCMotor *motor, Sensor *_sensor)
{
    motor->sensor = _sensor;
}

/**
    Driver linking method
*/
static void default_linkDriver(FOCMotor *motor, FOCDriver *_driver)
{
    motor->driver = _driver;
}

/**
 CurrentSense linking method
 */
static void default_linkCurrentSense(FOCMotor *motor, CurrentSense *_current_sense)
{
    motor->current_sense = _current_sense;
}

// shaft angle calculation
static float default_shaftAngle(FOCMotor *motor)
{
    // if no sensor linked return previous value ( for open loop )
    if (!motor->sensor)
        return motor->shaft_angle;
    return motor->sensor_direction * motor->sensor->getAngle(motor->sensor) - motor->sensor_offset;
}

// shaft velocity calculation
static float default_shaftVelocity(FOCMotor *motor)
{
    // if no sensor linked return previous value ( for open loop )
    if (!motor->sensor)
        return motor->shaft_velocity;
    return motor->sensor_direction * LowPassFilter_update(&motor->LPF_velocity, motor->sensor->getVelocity(motor->sensor));
}

static float default_electricalAngle(FOCMotor *motor)
{
    // if no sensor linked return previous value ( for open loop )
    if (!motor->sensor)
        return motor->electrical_angle;
    return _normalizeAngle((float)(motor->sensor_direction * motor->pole_pairs) * motor->sensor->getMechanicalAngle(motor->sensor) - motor->zero_electric_angle);
}

/**
 *  Monitoring functions
 */
// function implementing the monitor_port setter
static void default_useMonitoring(FOCMotor *motor, Print *print)
{
    motor->monitor_port = print; // operate on the address of print
    //   #ifndef TinyFOC_DISABLE_DEBUG
    //   TinyFOCDebug::enable(&print);
    //   TinyFOC_MOTOR_DEBUG("Monitor enabled!");
    //   #endif
}

// Measure resistance and inductance of a motor
static int default_characteriseMotor(FOCMotor *motor, float voltage, float correction_factor)
{
    if (!motor->current_sense || !motor->current_sense->initialized)
    {
        TinyFOC_MOTOR_ERROR("Fail. CS not init.");
        return 1;
    }

    if (voltage <= 0.0f)
    {
        TinyFOC_MOTOR_ERROR("Fail. Volt. <= 0");
        return 2;
    }
    voltage = _constrain(voltage, 0.0f, motor->voltage_limit);

    TinyFOC_MOTOR_DEBUG("Meas R..");

    float current_electric_angle = motor->electricalAngle(motor);

    // Apply zero volts and measure a zero reference
    motor->setPhaseVoltage(motor, 0, 0, current_electric_angle);
    _delay(500);

    PhaseCurrent_s zerocurrent_raw = CurrentSense_readAverageCurrents(motor->current_sense, CURRENT_SENSE_N);
    DQCurrent_s zerocurrent = CurrentSense_getDQCurrents(motor->current_sense, CurrentSense_getABCurrents(motor->current_sense, zerocurrent_raw), current_electric_angle);

    // Ramp and hold the voltage to measure resistance
    // 300 ms of ramping
    current_electric_angle = motor->electricalAngle(motor);
    for (int i = 0; i < 100; i++)
    {
        motor->setPhaseVoltage(motor, 0, voltage / 100 * ((float)i), current_electric_angle);
        _delay(3);
    }
    _delay(10);
    // PhaseCurrent_s r_currents_raw = CurrentSense_readAverageCurrents(motor->current_sense, CURRENT_SENSE_N);
    DQCurrent_s r_currents = CurrentSense_getDQCurrents(motor->current_sense, CurrentSense_getABCurrents(motor->current_sense, zerocurrent_raw), current_electric_angle);

    // Zero again
    motor->setPhaseVoltage(motor, 0, 0, current_electric_angle);

    if (_abs(r_currents.d - zerocurrent.d) < 0.2f)
    {
        TinyFOC_MOTOR_ERROR("Fail. current too low");
        return 3;
    }

    float resistance = voltage / (correction_factor * (r_currents.d - zerocurrent.d));
    if (resistance <= 0.0f)
    {
        TinyFOC_MOTOR_ERROR("Fail. Est. R<= 0");
        return 4;
    }

    TinyFOC_MOTOR_DEBUG("Est. R: ", 2.0f * resistance);
    _delay(100);

    // Start inductance measurement
    TinyFOC_MOTOR_DEBUG("Meas L...");

    unsigned long t0 = 0;
    unsigned long t1 = 0;
    float Ltemp = 0;
    float Ld = 0;
    float Lq = 0;
    float d_electrical_angle = 0;

    unsigned int iterations = 40;    // how often the algorithm gets repeated.
    unsigned int cycles = 3;         // averaged measurements for each iteration
    unsigned int risetime_us = 200;  // initially short for worst case scenario with low inductance
    unsigned int settle_us = 100000; // initially long for worst case scenario with high inductance

    // Pre-rotate the angle to the q-axis (only useful with sensor, else no harm in doing it)
    current_electric_angle += 0.5f * _PI;
    current_electric_angle = _normalizeAngle(current_electric_angle);

    for (size_t axis = 0; axis < 2; axis++)
    {
        for (size_t i = 0; i < iterations; i++)
        {
            // current_electric_angle = i * _2PI / iterations; // <-- Do a sweep of the inductance. Use eg. for graphing
            float inductanced = 0.0f;

            float qcurrent = 0.0f;
            float dcurrent = 0.0f;
            for (size_t j = 0; j < cycles; j++)
            {
                // read zero current
                zerocurrent_raw = CurrentSense_readAverageCurrents(motor->current_sense, 20);
                zerocurrent = CurrentSense_getDQCurrents(motor->current_sense, CurrentSense_getABCurrents(motor->current_sense, zerocurrent_raw), current_electric_angle);

                // step the voltage
                motor->setPhaseVoltage(motor, 0, voltage, current_electric_angle);
                t0 = _micros();
                _delay_us(risetime_us); // wait a little bit

                PhaseCurrent_s l_currents_raw = motor->current_sense->getPhaseCurrents(motor->current_sense);
                t1 = _micros();
                motor->setPhaseVoltage(motor, 0, 0, current_electric_angle);

                DQCurrent_s l_currents = CurrentSense_getDQCurrents(motor->current_sense, CurrentSense_getABCurrents(motor->current_sense, l_currents_raw), current_electric_angle);
                _delay_us(settle_us); // wait a bit for the currents to go to 0 again

                if (t0 > t1)
                    continue; // safety first

                // calculate the inductance
                float dt = (t1 - t0) / 1000000.0f;
                if (l_currents.d - zerocurrent.d <= 0 || (voltage - resistance * (l_currents.d - zerocurrent.d)) <= 0)
                {
                    continue;
                }

                inductanced += _abs(-(resistance * dt) / _log((voltage - resistance * (l_currents.d - zerocurrent.d)) / voltage)) / correction_factor;

                qcurrent += l_currents.q - zerocurrent.q; // average the measured currents
                dcurrent += l_currents.d - zerocurrent.d;
            }
            qcurrent /= cycles;
            dcurrent /= cycles;

            float delta = qcurrent / (_abs(dcurrent) + _abs(qcurrent));

            inductanced /= cycles;
            Ltemp = i < 2 ? inductanced : Ltemp * 0.6 + inductanced * 0.4;

            float timeconstant = _abs(Ltemp / resistance); // Timeconstant of an RL circuit (L/R)
            // TinyFOC_MOTOR_DEBUG("Estimated time constant in us: ", 1000000.0f * timeconstant);

            // Wait as long as possible (due to limited timing accuracy & sample rate), but as short as needed (while the current still changes)
            risetime_us = _constrain(risetime_us * 0.6f + 0.4f * 1000000 * 0.6f * timeconstant, 100, 10000);
            settle_us = 10 * risetime_us;

            // Serial.printf(">inductance:%f:%f|xy\n", current_electric_angle, Ltemp * 1000.0f); // <-- Plot an angle sweep

            /**
             * How this code works:
             * If we apply a current spike in the d´-axis, there will be cross coupling to the q´-axis current, if we didn´t use the actual d-axis (ie. d´ != d).
             * This has to do with saliency (Ld != Lq).
             * The amount of cross coupled current is somewhat proportional to the angle error, which means that if we iteratively change the angle to min/maximise this current, we get the correct d-axis (and q-axis).
             */
            if (axis)
            {
                qcurrent *= -1.0f; // to d or q axis
            }

            if (qcurrent < 0)
            {
                current_electric_angle += _abs(delta);
            }
            else
            {
                current_electric_angle -= _abs(delta);
            }
            current_electric_angle = _normalizeAngle(current_electric_angle);

            // Average the d-axis angle further for calculating the electrical zero later
            if (axis)
            {
                d_electrical_angle = i < 2 ? current_electric_angle : d_electrical_angle * 0.9 + current_electric_angle * 0.1;
            }
        }

        // We know the minimum is 0.5*PI radians away, so less iterations are needed.
        current_electric_angle += 0.5f * _PI;
        current_electric_angle = _normalizeAngle(current_electric_angle);
        iterations /= 2;

        if (axis == 0)
        {
            Lq = Ltemp;
        }
        else
        {
            Ld = Ltemp;
        }
    }

    if (motor->sensor)
    {
        /**
         * The d_electrical_angle should now be aligned to the d axis or the -d axis. We can therefore calculate two possible electrical zero angles.
         * We then report the one closest to the actual value. This could be useful if the zero search method is not reliable enough (eg. high pole count).
         */

        float estimated_zero_electric_angle_A = _normalizeAngle((float)(motor->sensor_direction * motor->pole_pairs) * motor->sensor->getMechanicalAngle(motor->sensor) - d_electrical_angle);
        float estimated_zero_electric_angle_B = _normalizeAngle((float)(motor->sensor_direction * motor->pole_pairs) * motor->sensor->getMechanicalAngle(motor->sensor) - d_electrical_angle + _PI);
        float estimated_zero_electric_angle = 0.0f;
        if (_abs(estimated_zero_electric_angle_A - motor->zero_electric_angle) < _abs(estimated_zero_electric_angle_B - motor->zero_electric_angle))
        {
            estimated_zero_electric_angle = estimated_zero_electric_angle_A;
        }
        else
        {
            estimated_zero_electric_angle = estimated_zero_electric_angle_B;
        }

        TinyFOC_MOTOR_DEBUG("New el. zero: ", estimated_zero_electric_angle);
        TinyFOC_MOTOR_DEBUG("Curr. el. zero: ", motor->zero_electric_angle);
    }

    TinyFOC_MOTOR_DEBUG("Ld [mH]: ", Ld * 1000.0f);
    TinyFOC_MOTOR_DEBUG("Lq [mH]: ", Lq * 1000.0f);
    if (Ld > Lq)
    {
        TinyFOC_MOTOR_WARN("Ld>Lq. Likely error.");
    }
    if (Ld * 2.0f < Lq)
    {
        TinyFOC_MOTOR_WARN("Lq > 2*Ld. Likely error.");
    }

    // store the measured values
    motor->phase_resistance = 2.0f * resistance;
    motor->axis_inductance.d = Ld;
    motor->axis_inductance.q = Lq;
    motor->phase_inductance = (Ld + Lq) / 2.0f; // FOR BACKWARDS COMPATIBILITY
    return 0;
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
static void default_monitor(FOCMotor *motor)
{
    if (!motor->monitor_downsample || motor->monitor_cnt++ < (motor->monitor_downsample - 1))
        return;
    motor->monitor_cnt = 0;
    if (!motor->monitor_port)
        return;
    bool printed = 0;

    if (motor->monitor_variables & _MON_TARGET)
    {
        if (!printed && motor->monitor_start_char)
            motor->monitor_port->write(motor->monitor_start_char);
        motor->monitor_port->print_f(motor->target, motor->monitor_decimals);
        printed = true;
    }
    if (motor->monitor_variables & _MON_VOLT_Q)
    {
        if (!printed && motor->monitor_start_char)
            motor->monitor_port->write(motor->monitor_start_char);
        else if (printed)
            motor->monitor_port->write(motor->monitor_separator);
        motor->monitor_port->print_f(motor->voltage.q, motor->monitor_decimals);
        printed = true;
    }
    if (motor->monitor_variables & _MON_VOLT_D)
    {
        if (!printed && motor->monitor_start_char)
            motor->monitor_port->write(motor->monitor_start_char);
        else if (printed)
            motor->monitor_port->write(motor->monitor_separator);
        motor->monitor_port->print_f(motor->voltage.d, motor->monitor_decimals);
        printed = true;
    }
    // read currents if possible - even in voltage mode (if current_sense available)
    if (motor->monitor_variables & _MON_CURR_Q || motor->monitor_variables & _MON_CURR_D)
    {
        DQCurrent_s c = motor->current;
        if (motor->current_sense && motor->torque_controller != TorqueControlType_foc_current)
        {

            c = CurrentSense_getFOCCurrents(motor->current_sense, motor->electrical_angle);
            c.q = LowPassFilter_update(&(motor->LPF_current_q), c.q);
            c.d = LowPassFilter_update(&(motor->LPF_current_d), c.d);
        }
        if (motor->monitor_variables & _MON_CURR_Q)
        {
            if (!printed && motor->monitor_start_char)
                motor->monitor_port->write(motor->monitor_start_char);
            else if (printed)
                motor->monitor_port->write(motor->monitor_separator);
            motor->monitor_port->print_f(c.q * 1000, motor->monitor_decimals); // mAmps
            printed = true;
        }
        if (motor->monitor_variables & _MON_CURR_D)
        {
            if (!printed && motor->monitor_start_char)
                motor->monitor_port->write(motor->monitor_start_char);
            else if (printed)
                motor->monitor_port->write(motor->monitor_separator);
            motor->monitor_port->print_f(c.d * 1000, motor->monitor_decimals); // mAmps
            printed = true;
        }
    }

    if (motor->monitor_variables & _MON_VEL)
    {
        if (!printed && motor->monitor_start_char)
            motor->monitor_port->write(motor->monitor_start_char);
        else if (printed)
            motor->monitor_port->write(motor->monitor_separator);
        motor->monitor_port->print_f(motor->shaft_velocity, motor->monitor_decimals);
        printed = true;
    }
    if (motor->monitor_variables & _MON_ANGLE)
    {
        if (!printed && motor->monitor_start_char)
            motor->monitor_port->write(motor->monitor_start_char);
        else if (printed)
            motor->monitor_port->write(motor->monitor_separator);
        motor->monitor_port->print_f(motor->shaft_angle, motor->monitor_decimals);
        printed = true;
    }
    if (printed)
    {
        if (motor->monitor_end_char)
        {
            motor->monitor_port->write(motor->monitor_end_char);
            motor->monitor_port->newline();
        }
        else
            motor->monitor_port->newline();
    }
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float FOCMotor_velocityOpenloop(FOCMotor *motor, float target_velocity)
{
    // get current timestamp
    unsigned long now_us = _micros();
    // calculate the sample time from last call
    float Ts = (now_us - motor->open_loop_timestamp) * 1e-6f;
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // calculate the necessary angle to achieve target velocity
    motor->shaft_angle = _normalizeAngle(motor->shaft_angle + target_velocity * Ts);
    // for display purposes
    motor->shaft_velocity = target_velocity;

    // save timestamp for next call
    motor->open_loop_timestamp = now_us;

    if (motor->torque_controller == TorqueControlType_voltage)
        return motor->voltage_limit;
    else
        return motor->current_limit;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float FOCMotor_angleOpenloop(FOCMotor *motor, float target_angle)
{
    // get current timestamp
    unsigned long now_us = _micros();
    // calculate the sample time from last call
    float Ts = (now_us - motor->open_loop_timestamp) * 1e-6f;
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // calculate the necessary angle to move from current position towards target angle
    // with maximal velocity (velocity_limit)
    // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
    //                        where small position changes are no longer captured by the precision of floats
    //                        when the total position is large.
    if (_abs(target_angle - motor->shaft_angle) > _abs(motor->velocity_limit * Ts))
    {
        motor->shaft_angle += _sign(target_angle - motor->shaft_angle) * _abs(motor->velocity_limit) * Ts;
        motor->shaft_velocity = motor->velocity_limit;
    }
    else
    {
        motor->shaft_angle = target_angle;
        motor->shaft_velocity = 0;
    }

    // save timestamp for next call
    motor->open_loop_timestamp = now_us;

    // use voltage limit or current limit
    if (motor->torque_controller == TorqueControlType_voltage)
        return motor->voltage_limit;
    else
        return motor->current_limit;
}

// Update limit values in controllers when changed
void FOCMotor_updateVelocityLimit(FOCMotor *motor, float new_velocity_limit)
{
    motor->velocity_limit = new_velocity_limit;
    if (motor->controller != MotionControlType_angle_nocascade)
        motor->P_angle.limit = _abs(motor->velocity_limit); // if angle control but no velocity cascade, limit the angle controller by the velocity limit
}

// Update limit values in controllers when changed
void FOCMotor_updateCurrentLimit(FOCMotor *motor, float new_current_limit)
{
    motor->current_limit = new_current_limit;
    if (motor->torque_controller != TorqueControlType_voltage)
    {
        // if current control
        motor->PID_velocity.limit = new_current_limit;
        if (motor->controller == MotionControlType_angle_nocascade)
            // if angle control but no velocity cascade, limit the angle controller by the current limit
            motor->P_angle.limit = new_current_limit;
    }
}

// Update limit values in controllers when changed
// PID values and limits
void FOCMotor_updateVoltageLimit(FOCMotor *motor, float new_voltage_limit)
{
    motor->voltage_limit = new_voltage_limit;
    motor->PID_current_q.limit = new_voltage_limit;
    motor->PID_current_d.limit = new_voltage_limit;
    if (motor->torque_controller == TorqueControlType_voltage)
    {
        // if voltage control
        motor->PID_velocity.limit = new_voltage_limit;
        if (motor->controller == MotionControlType_angle_nocascade)
            // if angle control but no velocity cascade, limit the angle controller by the voltage limit
            motor->P_angle.limit = new_voltage_limit;
    }
}

// Update torque control type and related controller limit values
void FOCMotor_updateTorqueControlType(FOCMotor *motor, enum TorqueControlType new_torque_controller)
{
    motor->torque_controller = new_torque_controller;
    // update the
    if (motor->torque_controller == TorqueControlType_voltage)
        // voltage control
        FOCMotor_updateVoltageLimit(motor, motor->voltage_limit);
    else
        // current control
        FOCMotor_updateCurrentLimit(motor, motor->current_limit);
}

// Update motion control type and related target values
// - if changing to angle control set target to current angle
// - if changing to velocity control set target to zero
// - if changing to torque control set target to zero
void FOCMotor_updateMotionControlType(FOCMotor *motor, enum MotionControlType new_motion_controller)
{

    if (motor->controller == new_motion_controller)
        return; // no change

    switch (new_motion_controller)
    {
    case (MotionControlType_angle_nocascade):
        if (motor->controller == MotionControlType_angle || motor->controller == MotionControlType_angle_openloop)
            break;
    case MotionControlType_angle:
        if (motor->controller == MotionControlType_angle_openloop || motor->controller == MotionControlType_angle_nocascade)
            break;
    case MotionControlType_angle_openloop:
        if (motor->controller == MotionControlType_angle || motor->controller == MotionControlType_angle_nocascade)
            break;
        // if the previous controller was not angle control
        // set target to current angle
        motor->target = motor->shaft_angle;
        break;
    case MotionControlType_velocity:
        if (motor->controller == MotionControlType_velocity_openloop)
            break; // nothing to do if we are already in velocity control
    case MotionControlType_velocity_openloop:
        if (motor->controller == MotionControlType_velocity)
            break;
        // if the previous controller was not velocity control
        // stop the motor
        motor->target = 0;
        break;
    case MotionControlType_torque:
        // if torque control set target to zero
        motor->target = 0;
        break;
    default:
        // if torque control set target to zero
        motor->target = 0;
        break;
    }

    // finally set the new controller
    motor->controller = new_motion_controller;
    // update limits in case they need to be changed for the new controller
    FOCMotor_updateVelocityLimit(motor, motor->velocity_limit);
    FOCMotor_updateCurrentLimit(motor, motor->current_limit);
    FOCMotor_updateVoltageLimit(motor, motor->voltage_limit);
}

int FOCMotor_tuneCurrentController(FOCMotor *motor, float bandwidth)
{
    if (bandwidth <= 0.0f)
    {
        // check bandwidth is positive
        TinyFOC_MOTOR_ERROR("Fail. BW <= 0");
        return 1;
    }
    if (motor->loopfoc_time_us && bandwidth > 0.5f * (1e6f / motor->loopfoc_time_us))
    {
        // check bandwidth is not too high for the control loop frequency
        TinyFOC_MOTOR_ERROR("Fail. BW too high, current loop freq:", (1e6f / motor->loopfoc_time_us));
        return 2;
    }
    if (!_isset(motor->phase_resistance) || (!_isset(motor->phase_inductance) && !_isset(motor->axis_inductance.q)))
    {
        // need motor parameters to tune the controller
        TinyFOC_MOTOR_WARN("Motor params missing!");
        if (motor->characteriseMotor(motor, motor->voltage_sensor_align, 1.5))
        {
            return 3;
        }
    }
    else if (_isset(motor->phase_inductance) && !(_isset(motor->axis_inductance.q)))
    {
        // if only single inductance value is set, use it for both d and q axis
        motor->axis_inductance.d = motor->phase_inductance;
        motor->axis_inductance.q = motor->phase_inductance;
    }

    motor->PID_current_q.P = motor->axis_inductance.q * (_2PI * bandwidth);
    motor->PID_current_q.I = motor->phase_resistance * (_2PI * bandwidth);
    motor->PID_current_d.P = motor->axis_inductance.d * (_2PI * bandwidth);
    motor->PID_current_d.I = motor->phase_resistance * (_2PI * bandwidth);
    motor->LPF_current_d.Tf = 1.0f / (_2PI * bandwidth * 5.0f); // filter cutoff at 5x bandwidth
    motor->LPF_current_q.Tf = 1.0f / (_2PI * bandwidth * 5.0f); // filter cutoff at 5x bandwidth

    TinyFOC_MOTOR_DEBUG("Tuned PI params for BW [Hz]: ", bandwidth);
    TinyFOC_MOTOR_DEBUG("Pq: ", motor->PID_current_q.P);
    TinyFOC_MOTOR_DEBUG("Iq: ", motor->PID_current_q.I);
    TinyFOC_MOTOR_DEBUG("Pd: ", motor->PID_current_d.P);
    TinyFOC_MOTOR_DEBUG("Id: ", motor->PID_current_d.I);

    return 0;
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
static void default_loopFOC(FOCMotor *motor)
{
    if (motor->motion_cnt++ > motor->motion_downsample)
        motor->move(motor, NOT_SET);
    
    // update loop time measurement
    updateLoopFOCTime(motor);

    // if initFOC is being executed at the moment, do nothing
    if (motor->motor_status == FOCMotorStatus_calibrating)
        return;

    // update sensor - do this even in open-loop mode, as user may be switching between modes and we could lose track
    //                 of full rotations otherwise.
    if (motor->sensor)
        motor->sensor->update(motor->sensor);

    // if disabled do nothing
    if (!motor->enabled)
        return;

    // if open-loop do nothing
    if (motor->controller == MotionControlType_angle_openloop || motor->controller == MotionControlType_velocity_openloop)
        // calculate the open loop electirical angle
        motor->electrical_angle = _electricalAngle((motor->shaft_angle), motor->pole_pairs);
    else
        // Needs the update() to be called first
        // This function will not have numerical issues because it uses Sensor::getMechanicalAngle()
        // which is in range 0-2PI
        motor->electrical_angle = motor->electricalAngle(motor);

    switch (motor->torque_controller)
    {
    case TorqueControlType_voltage:
        motor->voltage.q = _constrain(motor->current_sp, -motor->voltage_limit, motor->voltage_limit) + motor->feed_forward_voltage.q;
        motor->voltage.d = motor->feed_forward_voltage.d;
        break;
    case TorqueControlType_estimated_current:
        if (!_isset(motor->phase_resistance))
            return;
        // constrain current setpoint
        motor->current_sp = _constrain(motor->current_sp, -motor->current_limit, motor->current_limit) + motor->feed_forward_current.q; // desired current is the setpoint
        // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
        if (_isset(motor->KV_rating))
            motor->voltage_bemf = motor->estimateBEMF(motor, motor->shaft_velocity);
        // filter the value values
        motor->current.q = LowPassFilter_update(&(motor->LPF_current_q), motor->current_sp);
        // calculate the phase voltage
        motor->voltage.q = motor->current.q * motor->phase_resistance + motor->voltage_bemf;
        // constrain voltage within limits
        motor->voltage.q = _constrain(motor->voltage.q, -motor->voltage_limit, motor->voltage_limit) + motor->feed_forward_voltage.q;
        // d voltage  - lag compensation
        if (_isset(motor->axis_inductance.q))
            motor->voltage.d = _constrain(-motor->current_sp * motor->shaft_velocity * motor->pole_pairs * motor->axis_inductance.q, -motor->voltage_limit, motor->voltage_limit) + motor->feed_forward_voltage.d;
        else
            motor->voltage.d = motor->feed_forward_voltage.d;
        break;
    case TorqueControlType_dc_current:
        if (!motor->current_sense)
            return;
        // constrain current setpoint
        motor->current_sp = _constrain(motor->current_sp, -motor->current_limit, motor->current_limit) + motor->feed_forward_current.q;
        // read overall current magnitude
        motor->current.q = motor->current_sense->getDCCurrent(motor->current_sense, motor->electrical_angle);
        // filter the value values
        motor->current.q = LowPassFilter_update(&(motor->LPF_current_q), motor->current.q);
        // calculate the phase voltage
        motor->voltage.q = PIDController_update(&(motor->PID_current_q), motor->current_sp - motor->current.q) + motor->feed_forward_voltage.q;
        // d voltage  - lag compensation
        if (_isset(motor->axis_inductance.q))
            motor->voltage.d = _constrain(-motor->current_sp * motor->shaft_velocity * motor->pole_pairs * motor->axis_inductance.q, -motor->voltage_limit, motor->voltage_limit) + motor->feed_forward_voltage.d;
        else
            motor->voltage.d = motor->feed_forward_voltage.d;
        break;
    case TorqueControlType_foc_current:
        if (!motor->current_sense)
            return;
        // constrain current setpoint
        motor->current_sp = _constrain(motor->current_sp, -motor->current_limit, motor->current_limit) + motor->feed_forward_current.q;
        // read dq currents
        motor->current = CurrentSense_getFOCCurrents(motor->current_sense, motor->electrical_angle);
        // filter values
        motor->current.q = LowPassFilter_update(&(motor->LPF_current_q), motor->current.q);
        motor->current.d = LowPassFilter_update(&(motor->LPF_current_d), motor->current.d);
        // calculate the phase voltages
        motor->voltage.q = PIDController_update(&(motor->PID_current_q), motor->current_sp - motor->current.q);
        motor->voltage.d = PIDController_update(&(motor->PID_current_d), motor->feed_forward_current.d - motor->current.d);
        // d voltage - lag compensation
        if (_isset(motor->axis_inductance.q))
            motor->voltage.d = _constrain(motor->voltage.d - motor->current_sp * motor->shaft_velocity * motor->pole_pairs * motor->axis_inductance.q, -motor->voltage_limit, motor->voltage_limit);
        // q voltage - cross coupling compensation - TODO verify
        if (_isset(motor->axis_inductance.d))
            motor->voltage.q = _constrain(motor->voltage.q + motor->current.d * motor->shaft_velocity * motor->pole_pairs * motor->axis_inductance.d, -motor->voltage_limit, motor->voltage_limit);
        // add feed forward
        motor->voltage.q += motor->feed_forward_voltage.q;
        motor->voltage.d += motor->feed_forward_voltage.d;
        break;
    default:
        // no torque control selected
        TinyFOC_MOTOR_ERROR("no torque control selected!");
        break;
    }
    // set the phase voltage - FOC heart function :)
    motor->setPhaseVoltage(motor, motor->voltage.q, motor->voltage.d, motor->electrical_angle);
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or voltage loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
static void default_move(FOCMotor *motor, float new_target)
{
    motor->motion_cnt = 1;

    // set internal target variable
    if (_isset(new_target))
        motor->target = new_target;


    // if initFOC is being executed at the moment, do nothing
    if (motor->motor_status == FOCMotorStatus_calibrating)
        return;

    // calculate the elapsed time between the calls
    // TODO replace downsample by runnind the code at
    // a specific frequency (or almost)
    updateMotionControlTime(motor);

    // read value even if motor is disabled to keep the monitoring updated
    // except for the open loop modes where the values are updated within angle/velocityOpenLoop functions

    // shaft angle/velocity need the update() to be called first
    // get shaft angle
    // TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a float
    //                        For this reason it is NOT precise when the angles become large.
    //                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a problem
    //                        when switching to a 2-component representation.
    if (motor->controller != MotionControlType_angle_openloop && motor->controller != MotionControlType_velocity_openloop)
    {
        // read the values only if the motor is not in open loop
        // because in open loop the shaft angle/velocity is updated within angle/velocityOpenLoop functions
        motor->shaft_angle = motor->shaftAngle(motor);
        motor->shaft_velocity = motor->shaftVelocity(motor);
    }

    // if disabled do nothing
    // and if
    if (!motor->enabled)
        return;

    // upgrade the current based voltage limit
    switch (motor->controller)
    {
    case MotionControlType_torque:
        motor->current_sp = motor->target;
        break;
    case MotionControlType_angle_nocascade:
        // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
        //                        the angles are large. This results in not being able to command small changes at high position values.
        //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
        // angle set point
        motor->shaft_angle_sp = motor->target;
        // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
        motor->current_sp = PIDController_update(&(motor->P_angle), motor->shaft_angle_sp - LowPassFilter_update(&motor->LPF_angle, motor->shaft_angle));
        break;
    case MotionControlType_angle:
        // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
        //                        the angles are large. This results in not being able to command small changes at high position values.
        //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
        // angle set point
        motor->shaft_angle_sp = motor->target;
        // calculate velocity set point
        motor->shaft_velocity_sp = motor->feed_forward_velocity + PIDController_update(&(motor->P_angle), motor->shaft_angle_sp - LowPassFilter_update(&motor->LPF_angle, motor->shaft_angle));
        motor->shaft_velocity_sp = _constrain(motor->shaft_velocity_sp, -motor->velocity_limit, motor->velocity_limit);
        // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
        motor->current_sp = PIDController_update(&(motor->PID_velocity), motor->shaft_velocity_sp - motor->shaft_velocity);
        break;
    case MotionControlType_velocity:
        // velocity set point - sensor precision: this calculation is numerically precise.
        motor->shaft_velocity_sp = motor->target;
        // calculate the torque command
        motor->current_sp = PIDController_update(&(motor->PID_velocity), motor->shaft_velocity_sp - motor->shaft_velocity);
        break;
    case MotionControlType_velocity_openloop:
        // velocity control in open loop - sensor precision: this calculation is numerically precise.
        motor->shaft_velocity_sp = motor->target;
        // this function updates the shaft_angle and shaft_velocity
        // returns the voltage or current that is to be set to the motor (depending on torque control mode)
        // returned values correspond to the voltage_limit and current_limit
        motor->current_sp = FOCMotor_velocityOpenloop(motor, motor->shaft_velocity_sp);
        break;
    case MotionControlType_angle_openloop:
        // angle control in open loop -
        // TODO sensor precision: this calculation NOT numerically precise, and subject
        //                        to the same problems in small set-point changes at high angles
        //                        as the closed loop version.
        motor->shaft_angle_sp = motor->target;
        // this function updates the shaft_angle and shaft_velocity
        // returns the voltage or current that is to be set to the motor (depending on torque control mode)
        // returned values correspond to the voltage_limit and current_limit
        motor->current_sp = FOCMotor_angleOpenloop(motor, motor->shaft_angle_sp);
        break;
    case MotionControlType_custom:
        // custom control - user provides the function that calculates the current_sp
        // based on the target value and the motor state
        // user makes sure to use it with appropriate torque control mode
        if (motor->customMotionControlCallback)
            motor->current_sp = motor->customMotionControlCallback(motor);
        break;
    }
}

// FOC initialization function
static int default_initFOC(FOCMotor *motor)
{
    int exit_flag = 1;

    motor->motor_status = FOCMotorStatus_calibrating;

    // align motor if necessary
    // alignment necessary for encoders!
    // sensor and motor alignment - can be skipped
    // by setting motor.sensor_direction and motor.zero_electric_angle
    if (motor->sensor)
    {
        exit_flag *= FOCMotor_alignSensor(motor);
        // added the shaft_angle update
        motor->sensor->update(motor->sensor);
        motor->shaft_angle = motor->shaftAngle(motor);
    }
    else
    {
        TinyFOC_MOTOR_DEBUG("No sensor.");
        if ((motor->controller == MotionControlType_angle_openloop || motor->controller == MotionControlType_velocity_openloop))
        {
            exit_flag = 1;
        }
        else
        {
            TinyFOC_MOTOR_ERROR("Only openloop allowed!");
            exit_flag = 0; // no FOC without sensor
        }
    }

    // aligning the current sensor - can be skipped
    // checks if driver phases are the same as current sense phases
    // and checks the direction of measuremnt.
    if (exit_flag)
    {
        if (motor->current_sense)
        {
            if (!motor->current_sense->initialized)
            {
                motor->motor_status = FOCMotorStatus_calib_failed;
                TinyFOC_MOTOR_ERROR("Current sense not init!");
                exit_flag = 0;
            }
            else
            {
                exit_flag *= FOCMotor_alignCurrentSense(motor);
            }
        }
        else
        {
            TinyFOC_MOTOR_ERROR("No current sense.");
        }
    }

    if (exit_flag)
    {
        TinyFOC_MOTOR_DEBUG("Ready.");
        motor->motor_status = FOCMotorStatus_ready;
    }
    else
    {
        TinyFOC_MOTOR_ERROR("Init FOC fail");
        motor->motor_status = FOCMotorStatus_calib_failed;
        motor->disable(motor);
    }

    return exit_flag;
}

// Calibarthe the motor and current sense phases
int FOCMotor_alignCurrentSense(FOCMotor *motor)
{
    int exit_flag = 1; // success

    TinyFOC_MOTOR_DEBUG("Align current sense.");

    // align current sense and the driver
    exit_flag = motor->current_sense->driverAlign(motor->current_sense, motor->voltage_sensor_align, motor->modulation_centered);
    if (!exit_flag)
    {
        // error in current sense - phase either not measured or bad connection
        TinyFOC_MOTOR_ERROR("Align error!");
        exit_flag = 0;
    }
    else
    {
        // output the alignment status flag
        TinyFOC_MOTOR_DEBUG("Success: ", exit_flag);
    }

    return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int FOCMotor_alignSensor(FOCMotor *motor)
{
    int exit_flag = 1; // success
    TinyFOC_MOTOR_DEBUG("Align sensor.");

    // check if sensor needs zero search
    if (motor->sensor->needsSearch(motor->sensor))
        exit_flag = FOCMotor_absoluteZeroSearch(motor);
    // stop init if not found index
    if (!exit_flag)
        return exit_flag;

    // v2.3.3 fix for R_AVR_7_PCREL against symbol" bug for AVR boards
    // TODO figure out why this works
    float voltage_align = motor->voltage_sensor_align;

    // if unknown natural direction
    if (motor->sensor_direction == Direction_UNKNOWN)
    {

        // find natural direction
        // move one electrical revolution forward
        for (int i = 0; i <= 500; i++)
        {
            float angle = _3PI_2 + _2PI * i / 500.0f;
            motor->setPhaseVoltage(motor, voltage_align, 0, angle);
            motor->sensor->update(motor->sensor);
            _delay(2);
        }
        // take and angle in the middle
        motor->sensor->update(motor->sensor);
        float mid_angle = motor->sensor->getAngle(motor->sensor);
        // move one electrical revolution backwards
        for (int i = 500; i >= 0; i--)
        {
            float angle = _3PI_2 + _2PI * i / 500.0f;
            motor->setPhaseVoltage(motor, voltage_align, 0, angle);
            motor->sensor->update(motor->sensor);
            _delay(2);
        }
        motor->sensor->update(motor->sensor);
        float end_angle = motor->sensor->getAngle(motor->sensor);
        // setPhaseVoltage(0, 0, 0);
        _delay(200);
        // determine the direction the sensor moved
        float moved = _abs(mid_angle - end_angle);
        if (moved < MIN_ANGLE_DETECT_MOVEMENT)
        { // minimum angle to detect movement
            TinyFOC_MOTOR_ERROR("Failed to notice movement");
            return 0; // failed calibration
        }
        else if (mid_angle < end_angle)
        {
            TinyFOC_MOTOR_DEBUG("sensor dir: CCW");
            motor->sensor_direction = Direction_CCW;
        }
        else
        {
            TinyFOC_MOTOR_DEBUG("sensor dir: CW");
            motor->sensor_direction = Direction_CW;
        }
        // check pole pair number
        motor->pp_check_result = !(fabs(moved * motor->pole_pairs - _2PI) > 0.5f); // 0.5f is arbitrary number it can be lower or higher!
        if (motor->pp_check_result == false)
        {
            TinyFOC_MOTOR_WARN("PP check: fail - est. pp: ", _2PI / moved);
        }
        else
        {
            TinyFOC_MOTOR_DEBUG("PP check: OK!");
        }
    }
    else
        TinyFOC_MOTOR_DEBUG("Skip dir calib.");

    // zero electric angle not known
    if (!_isset(motor->zero_electric_angle))
    {
        // align the electrical phases of the motor and sensor
        // set angle -90(270 = 3PI/2) degrees
        motor->setPhaseVoltage(motor, voltage_align, 0, _3PI_2);
        _delay(700);
        // read the sensor
        motor->sensor->update(motor->sensor);
        // get the current zero electric angle
        motor->zero_electric_angle = 0;
        motor->zero_electric_angle = motor->electricalAngle(motor);
        _delay(20);
        TinyFOC_MOTOR_DEBUG("Zero elec. angle: ", motor->zero_electric_angle);
        // stop everything
        motor->setPhaseVoltage(motor, 0, 0, 0);
        _delay(200);
    }
    else
    {
        TinyFOC_MOTOR_DEBUG("Skip offset calib.");
    }
    return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int FOCMotor_absoluteZeroSearch(FOCMotor *motor)
{
    // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
    //                    of float is sufficient.
    TinyFOC_MOTOR_DEBUG("Index search...");
    // search the absolute zero with small velocity
    float limit_vel = motor->velocity_limit;
    float limit_volt = motor->voltage_limit;
    motor->velocity_limit = motor->velocity_index_search;
    motor->voltage_limit = motor->voltage_sensor_align;
    motor->shaft_angle = 0;
    while (motor->sensor->needsSearch(motor->sensor) && motor->shaft_angle < _2PI)
    {
        FOCMotor_angleOpenloop(motor, 1.5f * _2PI);
        // call important for some sensors not to loose count
        // not needed for the search
        motor->sensor->update(motor->sensor);
    }
    // disable motor
    motor->setPhaseVoltage(motor, 0, 0, 0);
    // reinit the limits
    motor->velocity_limit = limit_vel;
    motor->voltage_limit = limit_volt;
    // check if the zero found
    if (motor->monitor_port)
    {
        if (motor->sensor->needsSearch(motor->sensor))
        {
            TinyFOC_MOTOR_ERROR("Not found!");
        }
        else
        {
            TinyFOC_MOTOR_DEBUG("Success!");
        }
    }
    return !motor->sensor->needsSearch(motor->sensor);
}

void FOCMotor_load_default(FOCMotor *motor)
{
    // maximum angular velocity to be used for positioning
    motor->velocity_limit = DEF_VEL_LIM;
    // maximum voltage to be set to the motor
    motor->voltage_limit = DEF_POWER_SUPPLY;
    // not set on the begining
    motor->current_limit = DEF_CURRENT_LIM;

    // index search velocity
    motor->velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
    // sensor and motor align voltage
    motor->voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

    // default modulation is SinePWM
    motor->foc_modulation = FOCModulationType_SinePWM;

    // default target value
    motor->target = 0;
    motor->voltage.d = 0;
    motor->voltage.q = 0;
    // current target values
    motor->current_sp = 0;
    motor->current.q = 0;
    motor->current.d = 0;

    // voltage bemf
    motor->voltage_bemf = 0;
    motor->feed_forward_velocity = 0.0f;

    // Initialize phase voltages U alpha and U beta used for inverse Park and Clarke transform
    motor->Ualpha = 0;
    motor->Ubeta = 0;

    // monitor_port
    motor->monitor_port = NULL;
    motor->monitor_downsample = DEF_MON_DOWNSMAPLE;
    motor->monitor_start_char = '\0';
    motor->monitor_end_char = '\0';
    motor->monitor_separator = '\t';
    motor->monitor_decimals = 4;
    motor->monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
    motor->monitor_cnt = 0;

    motor->motion_downsample = 10;
    motor->motion_cnt = 1;

    motor->loopfoc_time_us = 0;
    motor->move_time_us = 0;
    motor->last_loopfoc_timestamp_us = 0;
    motor->last_loopfoc_time_us = 0;
    motor->last_move_timestamp_us = 0;
    motor->last_move_time_us = 0;

    motor->enabled = 1;

    // sensor
    motor->sensor_offset = 0.0f;
    motor->sensor = NULL;
    // current sensor
    motor->current_sense = NULL;

    //filters and controllers
    PIDController_init(&motor->PID_current_q, DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY, NOT_SET);
    PIDController_init(&motor->PID_current_d, DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY, NOT_SET);
    PIDController_init(&motor->PID_velocity, DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT, NOT_SET);
    PIDController_init(&motor->P_angle, DEF_P_ANGLE_P, 0, 0, 0, DEF_VEL_LIM, NOT_SET);

    LowPassFilter_init(&motor->LPF_current_q, DEF_CURR_FILTER_Tf, NOT_SET);
    LowPassFilter_init(&motor->LPF_current_d, DEF_CURR_FILTER_Tf, NOT_SET);
    LowPassFilter_init(&motor->LPF_velocity, DEF_VEL_FILTER_Tf, NOT_SET);
    LowPassFilter_init(&motor->LPF_angle, 0.0f, NOT_SET);


    // default implementation
    motor->init = default_init;
    motor->linkSensor = default_linkSensor;
    motor->linkCurrentSense = default_linkCurrentSense;
    motor->linkDriver = default_linkDriver;
    motor->shaftAngle = default_shaftAngle;
    motor->shaftVelocity = default_shaftVelocity;
    motor->electricalAngle = default_electricalAngle;
    motor->useMonitoring = default_useMonitoring;
    motor->characteriseMotor = default_characteriseMotor;
    motor->monitor = default_monitor;
    motor->loopFOC = default_loopFOC;
    motor->initFOC = default_initFOC;
    motor->move = default_move;
}