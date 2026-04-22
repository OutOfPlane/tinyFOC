#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "Sensor.h"
#include "CurrentSense.h"
#include "stdint.h"
#include "stdbool.h"

#include "../time_utils.h"
#include "../foc_utils.h"
#include "../defaults.h"
#include "../pid.h"
#include "../lowpass_filter.h"

#define MOT_ERR "ERR-MOT:"
#define MOT_WARN "WARN-MOT:"
#define MOT_DEBUG "MOT:"

#define SIMPLEFOC_DISABLE_DEBUG

#ifndef SIMPLEFOC_DISABLE_DEBUG
#define SIMPLEFOC_MOTOR_WARN(msg, ...)  \
      SimpleFOCDebug::print(MOT_WARN); \
      SIMPLEFOC_DEBUG(msg, ##__VA_ARGS__)

#define SIMPLEFOC_MOTOR_ERROR(msg, ...)  \
      SimpleFOCDebug::print(MOT_ERR); \
      SIMPLEFOC_DEBUG(msg, ##__VA_ARGS__)

#define SIMPLEFOC_MOTOR_DEBUG(msg, ...)  \
      SimpleFOCDebug::print(MOT_DEBUG); \
      SIMPLEFOC_DEBUG(msg, ##__VA_ARGS__)
      
#else
#define SIMPLEFOC_MOTOR_DEBUG(msg, ...)
#define SIMPLEFOC_MOTOR_ERROR(msg, ...)
#define SIMPLEFOC_MOTOR_WARN(msg, ...)
#endif

// monitoring bitmap
#define _MON_TARGET 0b1000000 // monitor target value
#define _MON_VOLT_Q 0b0100000 // monitor voltage q value
#define _MON_VOLT_D 0b0010000 // monitor voltage d value
#define _MON_CURR_Q 0b0001000 // monitor current q value - if measured
#define _MON_CURR_D 0b0000100 // monitor current d value - if measured
#define _MON_VEL    0b0000010 // monitor velocity value
#define _MON_ANGLE  0b0000001 // monitor angle value

/**
 *  Motion control type
 */
enum MotionControlType {
  MotionControlType_torque            = 0x00,     //!< Torque control
  MotionControlType_velocity          = 0x01,     //!< Velocity motion control
  MotionControlType_angle             = 0x02,     //!< Position/angle motion control
  MotionControlType_velocity_openloop = 0x03,
  MotionControlType_angle_openloop    = 0x04,
  MotionControlType_angle_nocascade   = 0x05,     //!< Position/angle motion control without velocity cascade
  MotionControlType_custom            = 0x06      //!< Custom control method - control method added by user
};

/**
 *  Motion control type
 */
enum TorqueControlType { 
  TorqueControlType_voltage            = 0x00,     //!< Torque control using voltage
  TorqueControlType_dc_current         = 0x01,     //!< Torque control using DC current (one current magnitude)
  TorqueControlType_foc_current        = 0x02,     //!< torque control using dq currents
  TorqueControlType_estimated_current  = 0x03      //!< torque control using estimated current (provided motor parameters)
};

/**
 *  FOC modulation type
 */
enum FOCModulationType {
  FOCModulationType_SinePWM            = 0x00,     //!< Sinusoidal PWM modulation
  FOCModulationType_SpaceVectorPWM     = 0x01,     //!< Space vector modulation method
  FOCModulationType_Trapezoid_120      = 0x02,     
  FOCModulationType_Trapezoid_150      = 0x03,     
};



enum FOCMotorStatus {
  FOCMotorStatus_uninitialized = 0x00,     //!< Motor is not yet initialized
  FOCMotorStatus_initializing  = 0x01,     //!< Motor intiialization is in progress
  FOCMotorStatus_uncalibrated  = 0x02,     //!< Motor is initialized, but not calibrated (open loop possible)
  FOCMotorStatus_calibrating   = 0x03,     //!< Motor calibration in progress
  FOCMotorStatus_ready         = 0x04,     //!< Motor is initialized and calibrated (closed loop possible)
  FOCMotorStatus_error         = 0x08,     //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
  FOCMotorStatus_calib_failed  = 0x0E,     //!< Motor calibration failed (possibly recoverable)
  FOCMotorStatus_init_failed   = 0x0F,     //!< Motor initialization failed (not recoverable)
};



/**
 Generic motor class
*/
typedef struct s_FOCMotor{
  /**  Motor hardware init function */
  void (*init)(struct s_FOCMotor *motor);

  /** Motor disable function */
  void (*disable)(struct s_FOCMotor *motor);

  /** Motor enable function */
  void (*enable)(struct s_FOCMotor *motor);

  /**
  * Method using FOC to set Uq to the motor at the optimal angle
  * Heart of the FOC algorithm
  * 
  * @param Uq Current voltage in q axis to set to the motor
  * @param Ud Current voltage in d axis to set to the motor
  * @param angle_el current electrical angle of the motor
  */
 void (*setPhaseVoltage)(struct s_FOCMotor *motor, float Uq, float Ud, float angle_el);

  /**
  * Estimation of the Back EMF voltage
  * 
  * @param velocity - current shaft velocity
  */
  float (*estimateBEMF)(struct s_FOCMotor *motor, float velocity);

  /**
   * Function initializing FOC algorithm
   * and aligning sensor's and motors' zero position 
   * 
   * - If zero_electric_offset parameter is set the alignment procedure is skipped
   */  
  void (*initFOC)(struct s_FOCMotor *motor);

  /**
   * Function running FOC algorithm in real-time
   * it calculates the gets motor angle and sets the appropriate voltages 
   * to the phase pwm signals
   * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
   */ 
  void (*loopFOC)(struct s_FOCMotor *motor);

  /**
   * Function executing the control loops set by the controller.
   * 
   * @param target  Either voltage, angle or velocity based on the motor.controller
   *                If it is not set the motor will use the target set in its variable motor.target
   * 
   * This function doesn't need to be run upon each loop execution - depends of the use case
   */
  void (*move)(struct s_FOCMotor *motor, float target);

  /**
   * Function linking a motor and a sensor 
   * 
   * @param sensor Sensor class  wrapper for the FOC algorihtm to read the motor angle and velocity
   */
  void (*linkSensor)(struct s_FOCMotor *motor, Sensor* sensor);

  /**
   * Function linking a motor and current sensing 
   * 
   * @param current_sense CurrentSense class wrapper for the FOC algorihtm to read the motor current measurements
   */
  void (*linkCurrentSense)(struct s_FOCMotor *motor, CurrentSense* current_sense);


  // State calculation methods 
  /** Shaft angle calculation in radians [rad] */
  float (*shaftAngle)(struct s_FOCMotor *motor);
  /** 
   * Shaft angle calculation function in radian per second [rad/s]
   * It implements low pass filtering
   */
  float (*shaftVelocity)(struct s_FOCMotor *motor);

  /** 
   * Electrical angle calculation  
   */
  float (*electricalAngle)(struct s_FOCMotor *motor);


  /**
   * Measure resistance and inductance of a motor and print results to debug.
   * If a sensor is available, an estimate of zero electric angle will be reported too.
   * @param voltage The voltage applied to the motor
   * @param correction_factor  Is 1.5 for 3 phase motors, because we measure over a series-parallel connection. TODO: what about 2 phase motors?
   * @returns 0 for success, >0 for failure
   */
  int (*characteriseMotor)(struct s_FOCMotor *motor, float voltage, float correction_factor);

  /**
   * Auto-tune the current controller PID parameters based on desired bandwidth.
   * Uses a simple method that assumes a first order system and requires knowledge of
   * the motor phase resistance and inductance (if not set, the characteriseMotor function can be used).
   * 
   * @param bandwidth Desired closed-loop bandwidth in Hz.
   * @returns returns 0 for success, >0 for failure
   */
  int (*tuneCurrentController)(struct s_FOCMotor *motor, float bandwidth);

  // state variables
  float target; //!< current target value - depends of the controller
  float feed_forward_velocity; //!< current feed forward velocity
  float shaft_angle;//!< current motor angle
  float electrical_angle;//!< current electrical angle
  float shaft_velocity;//!< current motor velocity 
  float current_sp;//!< target current ( q current )
  float shaft_velocity_sp;//!< current target velocity
  float shaft_angle_sp;//!< current target angle
  DQVoltage_s voltage;//!< current d and q voltage set to the motor
  DQCurrent_s current;//!< current d and q current measured
  float voltage_bemf; //!< estimated backemf voltage (if provided KV constant)
  float	Ualpha, Ubeta; //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

  DQCurrent_s feed_forward_current;//!< current d and q current measured
  DQVoltage_s feed_forward_voltage;//!< current d and q voltage set to the motor

  // motor configuration parameters
  float voltage_sensor_align;//!< sensor and motor align voltage parameter
  float velocity_index_search;//!< target velocity for index search 
  
  // motor physical parameters
  float	phase_resistance; //!< motor phase resistance
  int pole_pairs;//!< motor pole pairs number
  float KV_rating; //!< motor KV rating
  float	phase_inductance; //!< motor phase inductance q axis - FOR BACKWARDS COMPATIBILITY
  struct DQ_s axis_inductance; //!< motor direct axis phase inductance

  // limiting variables
  float voltage_limit; //!< Voltage limiting variable - global limit
  float current_limit; //!< Current limiting variable - global limit
  float velocity_limit; //!< Velocity limiting variable - global limit

  // motor status vairables
  int8_t enabled;//!< enabled or disabled motor flag
  enum FOCMotorStatus motor_status; //!< motor status
  
  // pwm modulation related variables
  enum FOCModulationType foc_modulation;//!<  parameter determining modulation algorithm
  int8_t modulation_centered;//!< flag (1) centered modulation around driver limit /2  or  (0) pulled to 0


  // configuration structures
  enum TorqueControlType torque_controller; //!< parameter determining the torque control type
  enum MotionControlType controller; //!< parameter determining the control loop to be used

  // controllers and low pass filters
  PIDController PID_current_q;//!< parameter determining the q current PID config
  PIDController PID_current_d;//!< parameter determining the d current PID config
  LowPassFilter LPF_current_q;//!<  parameter determining the current Low pass filter configuration 
  LowPassFilter LPF_current_d;//!<  parameter determining the current Low pass filter configuration 
  PIDController PID_velocity;//!< parameter determining the velocity PID configuration
  PIDController P_angle;	//!< parameter determining the position PID configuration 
  LowPassFilter LPF_velocity;//!<  parameter determining the velocity Low pass filter configuration 
  LowPassFilter LPF_angle;//!<  parameter determining the angle low pass filter configuration 
  unsigned int motion_downsample; //!< parameter defining the ratio of downsampling for move commad
  unsigned int motion_cnt; //!< counting variable for downsampling for move commad

  // sensor related variabels
  float sensor_offset; //!< user defined sensor zero offset
  float zero_electric_angle;//!< absolute zero electric angle - if available
  enum Direction sensor_direction; //!< default is CW. if sensor_direction == Direction::CCW then direction will be flipped compared to CW. Set to UNKNOWN to set by calibration
  bool pp_check_result; //!< the result of the PP check, if run during loopFOC

  /**
   * Function providing BLDCMotor class with the 
   * Serial interface and enabling monitoring mode
   * 
   * @param serial Monitoring Serial class reference
   */
  void (*useMonitoring)(struct s_FOCMotor *motor, Print *serial);
  // monitoring functions
  Print* monitor_port; //!< Serial terminal variable if provided

  /**
   * Utility function intended to be used with serial plotter to monitor motor variables
   * significantly slowing the execution down!!!!
   */
  void (*monitor)(struct s_FOCMotor *motor);
  unsigned int monitor_downsample; //!< show monitor outputs each monitor_downsample calls 
  char monitor_start_char; //!< monitor starting character 
  char monitor_end_char; //!< monitor outputs ending character 
  char monitor_separator; //!< monitor outputs separation character
  unsigned int  monitor_decimals; //!< monitor outputs decimal places
  // initial monitoring will display target, voltage, velocity and angle
  uint8_t monitor_variables; //!< Bit array holding the map of variables the user wants to monitor
  // monitor counting variable
  unsigned int monitor_cnt; //!< counting variable
  
  /** 
    * Sensor link:
    * - Encoder 
    * - MagneticSensor*
    * - HallSensor
  */
  Sensor* sensor; 
  //!< CurrentSense link
  CurrentSense* current_sense; 


  //!< time between two loopFOC executions in microseconds
  uint32_t loopfoc_time_us; //!< filtered loop times
  uint32_t move_time_us; // filtered motion control times  
  uint32_t last_loopfoc_timestamp_us; //!< timestamp of the last loopFOC execution in microseconds
  uint32_t last_loopfoc_time_us; //!< last elapsed time of loopFOC in microseconds
  uint32_t last_move_timestamp_us; //!< timestamp of the last move execution in microseconds
  uint32_t last_move_time_us; //!< last elapsed time of move in microseconds


  // open loop variables
  uint32_t open_loop_timestamp;
  
  // function pointer for custom control method
  float (*customMotionControlCallback)(struct s_FOCMotor *motor);

} FOCMotor;

void FOCMotor_load_default(FOCMotor *motor);


/**
 * Update limit values in controllers when changed
 * @param new_velocity_limit - new velocity limit value
 * 
 * @note Updates velocity limit in:
 *  - motor.velocity_limit
 *  - motor.P_angle.limit
 */
void FOCMotor_updateVelocityLimit(FOCMotor *motor, float new_velocity_limit);

/**
 * Update limit values in controllers when changed
 * @param new_current_limit - new current limit value
 * 
 * @note Updates current limit in:
 *  - motor.current_limit
 *  - motor.PID_velocity.limit (if current control)
 */
void FOCMotor_updateCurrentLimit(FOCMotor *motor, float new_current_limit);
/**
 * Update limit values in controllers when changed
 * @param new_voltage_limit - new voltage limit value
 * 
 * @note Updates voltage limit in:
 *  - motor.voltage_limit
 *  - motor.PID_current_q.limit
 *  - motor.PID_current_d.limit
 *  - motor.PID_velocity.limit (if voltage control)
 */
void FOCMotor_updateVoltageLimit(FOCMotor *motor, float new_voltage_limit);

/**
 * Update torque control type and related controller limit values
 * @param new_torque_controller - new torque control type
 * 
 * @note Updates motor.torque_controller and motor.PID_velocity.limit
 */
void FOCMotor_updateTorqueControlType(FOCMotor *motor, TorqueControlType new_torque_controller);
/**
 * Update motion control type and related target values
 * @param new_motion_controller - new motion control type
 * 
 * @note Updates the target value based on the new controller type
 * - if velocity control: target is set to 0rad/s
 * - if angle control: target is set to the current shaft_angle
 * - if torque control: target is set to 0V or 0A depending on torque control type
 */
void FOCMotor_updateMotionControlType(FOCMotor *motor, MotionControlType new_motion_controller);

// Open loop motion control    
/**
 * Function (iterative) generating open loop movement for target velocity
 * it uses voltage_limit variable
 * 
 * @param target_velocity - rad/s
 */
float FOCMotor_velocityOpenloop(FOCMotor *motor, float target_velocity);
/**
 * Function (iterative) generating open loop movement towards the target angle
 * it uses voltage_limit and velocity_limit variables
 * 
 * @param target_angle - rad
 */
float FOCMotor_angleOpenloop(FOCMotor *motor, float target_angle);


/**
 * Function setting a custom motion control method defined by the user
 * @note the custom control method has to be defined by the user and should follow the signature: float controlMethod(FOCMotor* motor)
 * @param controlMethod - pointer to the custom control method function defined by the user
 */
void FOCMotor_linkCustomMotionControl(FOCMotor *motor, float (*controlMethod)(FOCMotor* motor));

/**
 * Function udating loop time measurement
 * time between two loopFOC executions in microseconds
 * It filters the value using low pass filtering alpha = 0.1
 * @note - using _micros() function - be aware of its overflow every ~70 minutes
 */
void FOCMotor_updateLoopFOCTime();

void FOCMotor_updateMotionControlTime();

/** Sensor alignment to electrical 0 angle of the motor */
int FOCMotor_alignSensor();
/** Current sense and motor phase alignment */
int FOCMotor_alignCurrentSense();
/** Motor and sensor alignment to the sensors absolute 0 angle  */
int FOCMotor_absoluteZeroSearch();


#endif
