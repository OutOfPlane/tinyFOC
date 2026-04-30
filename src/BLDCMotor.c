#include "BLDCMotor.h"
#include "./communication/TinyFOCDebug.h"


// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,_HIGH_IMPEDANCE,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,_HIGH_IMPEDANCE,-1} 
};

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,1,1},
    {-1,_HIGH_IMPEDANCE,1},
    {-1,-1,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,-1,-1},
    {1,_HIGH_IMPEDANCE,-1},
    {1,1,-1} 
};


// init hardware pins
int BLDCMotor_init(FOCMotor *motor) {
  if (!motor->driver || !motor->driver->initialized) {
    motor->motor_status = FOCMotorStatus_init_failed;
    TinyFOC_MOTOR_ERROR("Init not possible, driver not init");
    return 0;
  }
  motor->motor_status = FOCMotorStatus_initializing;
  TinyFOC_MOTOR_DEBUG("Init");

  // sanity check for the voltage limit configuration
  if(motor->voltage_limit > motor->driver->voltage_limit) motor->voltage_limit =  motor->driver->voltage_limit;
  // constrain voltage for sensor alignment
  if(motor->voltage_sensor_align > motor->voltage_limit) motor->voltage_sensor_align = motor->voltage_limit;

  // update limits in the motor controllers
  FOCMotor_updateVelocityLimit(motor, motor->velocity_limit);
  FOCMotor_updateCurrentLimit(motor, motor->current_limit);
  FOCMotor_updateVoltageLimit(motor, motor->voltage_limit);
  
  if(_isset(motor->phase_inductance) && !(_isset(motor->axis_inductance.q))) {
    // if only single inductance value is set, use it for both d and q axis
    motor->axis_inductance.q = motor->phase_inductance;
    motor->axis_inductance.d = motor->phase_inductance;
  } 
  
  // if using open loop control, set a CW as the default direction if not already set
  // only if no sensor is used
  if(!motor->sensor){
    if ((motor->controller==MotionControlType_angle_openloop
      ||motor->controller==MotionControlType_velocity_openloop)
      && (motor->sensor_direction == Direction_UNKNOWN)) {
        motor->sensor_direction = Direction_CW;
    }
  }

  _delay(500);
  // enable motor
  TinyFOC_MOTOR_DEBUG("Enable driver.");
  motor->enable(motor);
  _delay(500);
  motor->motor_status = FOCMotorStatus_uncalibrated;
  return 1;
}


// disable motor driver
void BLDCMotor_disable(FOCMotor *motor)
{
  // disable the current sense
  if(motor->current_sense) motor->current_sense->disable(motor->current_sense);
  // set zero to PWM
  motor->driver->setPwm(motor->driver, 0, 0, 0);
  // disable the driver
  motor->driver->disable(motor->driver);
  // motor status update
  motor->enabled = 0;
}
// enable motor driver
void BLDCMotor_enable(FOCMotor *motor)
{
  // enable the driver
  motor->driver->enable(motor->driver);
  // set zero to PWM
  motor->driver->setPwm(motor->driver, 0, 0, 0);
  // enable the current sense
  if(motor->current_sense) motor->current_sense->enable(motor->current_sense);
  // reset the pids
  PIDController_reset(&motor->PID_velocity);
  PIDController_reset(&motor->P_angle);
  PIDController_reset(&motor->PID_current_q);
  PIDController_reset(&motor->PID_current_d);
  // motor status update
  motor->enabled = 1;
}

/**
  FOC functions
*/

FIXP BLDCMotor_estimateBEMF(FOCMotor *motor, FIXP vel){
  // bemf constant is approximately 1/KV rating
  // V_bemf = K_bemf * velocity
  return FIX_DIV(FIX_DIV(vel, FIX_RPM_TO_RADS),(motor->KV_rating*FIX_SQRT3));
}


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM, Sine PWM and Trapezoidal commutation algorithms
void BLDCMotor_setPhaseVoltage(FOCMotor *motor, FIXP Uq, FIXP Ud, FIXP angle_el) {

  FIXP center;
  int sector;
  FIXP _ca,_sa;

  switch (motor->foc_modulation)
  {
    case FOCModulationType_Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      // determine the sector
      sector = (fix_normalize_angle(angle_el + FIX_PI_6 ) * 6) / FIX_2PI; // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = motor->modulation_centered ? (motor->driver->voltage_limit)/2 : Uq;

      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
        motor->Ua= center;
        motor->Ub = trap_120_map[sector][1] * Uq + center;
        motor->Uc = trap_120_map[sector][2] * Uq + center;
        motor->driver->setPhaseState(motor->driver, PHASE_OFF, PHASE_ON, PHASE_ON); // disable phase if possible
      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
        motor->Ua = trap_120_map[sector][0] * Uq + center;
        motor->Ub = center;
        motor->Uc = trap_120_map[sector][2] * Uq + center;
        motor->driver->setPhaseState(motor->driver, PHASE_ON, PHASE_OFF, PHASE_ON);// disable phase if possible
      }else{
        motor->Ua = trap_120_map[sector][0] * Uq + center;
        motor->Ub = trap_120_map[sector][1] * Uq + center;
        motor->Uc = center;
        motor->driver->setPhaseState(motor->driver, PHASE_ON, PHASE_ON, PHASE_OFF);// disable phase if possible
      }

    break;

    case FOCModulationType_Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      // determine the sector
      sector = (fix_normalize_angle(angle_el + FIX_PI_6 ) * 12) / FIX_2PI; // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = motor->modulation_centered ? (motor->driver->voltage_limit)/2 : Uq;

      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
        motor->Ua= center;
        motor->Ub = trap_150_map[sector][1] * Uq + center;
        motor->Uc = trap_150_map[sector][2] * Uq + center;
        motor->driver->setPhaseState(motor->driver, PHASE_OFF, PHASE_ON, PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
        motor->Ua = trap_150_map[sector][0] * Uq + center;
        motor->Ub = center;
        motor->Uc = trap_150_map[sector][2] * Uq + center;
        motor->driver->setPhaseState(motor->driver, PHASE_ON, PHASE_OFF, PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][2]  == _HIGH_IMPEDANCE){
        motor->Ua = trap_150_map[sector][0] * Uq + center;
        motor->Ub = trap_150_map[sector][1] * Uq + center;
        motor->Uc = center;
        motor->driver->setPhaseState(motor->driver, PHASE_ON, PHASE_ON, PHASE_OFF); // disable phase if possible
      }else{
        motor->Ua = trap_150_map[sector][0] * Uq + center;
        motor->Ub = trap_150_map[sector][1] * Uq + center;
        motor->Uc = trap_150_map[sector][2] * Uq + center;
        motor->driver->setPhaseState(motor->driver, PHASE_ON, PHASE_ON, PHASE_ON); // enable all phases
      }

    break;

    case FOCModulationType_SinePWM :
    case FOCModulationType_SpaceVectorPWM :

        // Sinusoidal PWM modulation
        // Inverse Park + Clarke transformation
        fix_sincos(angle_el, &_sa, &_ca);

        // Inverse park transform
        motor->Ualpha = FIX_MUL(_ca, Ud) - FIX_MUL(_sa, Uq);  // -sin(angle) * Uq;
        motor->Ubeta = FIX_MUL(_sa, Ud) + FIX_MUL(_ca, Uq);    //  cos(angle) * Uq;

        // Clarke transform
        motor->Ua = motor->Ualpha;
        motor->Ub = -(motor->Ualpha / 2) + FIX_MUL(FIX_SQRT3_2, motor->Ubeta);
        motor->Uc = -(motor->Ualpha / 2) - FIX_MUL(FIX_SQRT3_2, motor->Ubeta);

        // centering the voltages around either
        // - centered modulation: around driver.voltage_limit/2
        // - non-centered modulation: pulls the lowest voltage to 0 
        //     - Can be useful for low-side current sensing 
        //       in cases where the ADC had long sample time
        //     - The part of the duty cycle in which all phases are 
        //       off is longer than in centered modulation   
        //     - Both SinePWM and SpaceVectorPWM have the same form for non-centered modulation
      if (motor->modulation_centered) {
        center = motor->driver->voltage_limit >> 1;
        if (motor->foc_modulation == FOCModulationType_SpaceVectorPWM){
          // discussed here: https://community.TinyFOC.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
          // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
          // Midpoint Clamp
          FIXP Umin = min(motor->Ua, min(motor->Ub, motor->Uc));
          FIXP Umax = max(motor->Ua, max(motor->Ub, motor->Uc));
          center -= (Umax+Umin) / 2;
        } 
        motor->Ua += center;
        motor->Ub += center;
        motor->Uc += center;
      }else{
        FIXP Umin = min(motor->Ua, min(motor->Ub, motor->Uc));
        motor->Ua -= Umin;
        motor->Ub -= Umin;
        motor->Uc -= Umin;
      }
      break;
  }

  // set the voltages in driver
  motor->driver->setPwm(motor->driver, motor->Ua, motor->Ub, motor->Uc);
}


void BLDCMotor_load_default(FOCMotor *motor)
{
  FOCMotor_load_default(motor);
  // save pole pairs number
  motor->pole_pairs = NOT_SET;
  // save phase resistance number
  motor->phase_resistance = NOT_SET;
  // save back emf constant KV = 1/KV
  // 1/sqrt(3) - rms value
  motor->KV_rating = NOT_SET;
  // save phase inductance
  motor->axis_inductance.d = NOT_SET;
  motor->axis_inductance.q = NOT_SET;
  motor->phase_inductance = NOT_SET;  // FOR BACKWARDS COMPATIBILITY

  // torque control type is voltage by default
  motor->torque_controller = TorqueControlType_voltage;

  motor->estimateBEMF = BLDCMotor_estimateBEMF;
  motor->setPhaseVoltage = BLDCMotor_setPhaseVoltage;
  motor->init = BLDCMotor_init;
  motor->enable = BLDCMotor_enable;
  motor->disable = BLDCMotor_disable;
}
