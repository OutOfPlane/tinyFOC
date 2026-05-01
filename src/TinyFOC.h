/*!
 * @file TinyFOC.h
 *
 * @mainpage Simple Field Oriented Control BLDC motor control library
 *
 * @section intro_sec Introduction
 *
 * Proper low-cost and low-power FOC supporting boards are very hard to find these days and even may not exist.<br> Even harder to find is a stable and simple FOC algorithm code capable of running on Arduino devices. Therefore this is an attempt to:
 * - Demystify FOC algorithm and make a robust but simple Arduino library: Arduino TinyFOC library
 * - Develop a modular BLDC driver board: Arduino TinyFOC shield.
 *
 * @section features Features
 *  - Arduino compatible: Arduino library code
 *  - Easy to setup and configure:
 *     - Easy hardware configuration
 *     - Easy tuning the control loops
 *  - Modular:
 *     - Supports as many sensors , BLDC motors and driver boards as possible
 *     - Supports as many application requirements as possible
 *  - Plug & play: Arduino TinyFOC shield
 * 
 * @section dependencies Supported Hardware
 *  - Motors 
 *    - BLDC motors
 *    - Stepper motors
 *    - Hybrid Stepper motors
 * - Drivers 
 *    - BLDC drivers
 *    - Gimbal drivers
 *    - Stepper drivers
 * - Position sensors 
 *    - Encoders
 *    - Magnetic sensors
 *    - Hall sensors
 * - Microcontrollers 
 *    - Arduino
 *    - STM32
 *    - ESP32
 *    - Teensy
 *    - many more ...
 * 
 * @section getting_examples How to Use Examples
 * 
 * 1. **Arduino IDE**: Go to `File > Examples > Simple FOC > [category] > [example]`
 * 2. **PlatformIO**: Copy example code from the `examples/` directory
 * 3. **GitHub**: Browse examples at https://github.com/TinyFOC/Arduino-FOC/tree/master/examples
 * 
 * Each example includes:
 * - Detailed comments explaining the code
 * - Hardware connection diagrams (where applicable)
 * - Expected behavior description
 * - Common troubleshooting tips
 * 
 * @section example_code Example code
 * @code
#include <TinyFOC.h>

//  BLDCMotor( pole_pairs )
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional) )
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
//  Encoder(pin_A, pin_B, CPR)
Encoder encoder = Encoder(2, 3, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // initialise driver hardware
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  // initialize motor
  motor.init();
  
  // align encoder and start FOC
  motor.initFOC();
}

void loop() {
  // FOC algorithm function
  motor.loopFOC();

  // velocity control loop function
  // setting the target velocity or 2rad/s
  motor.move(2);
}
 * @endcode 
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef TinyFOC_H
#define TinyFOC_H

#include "common/CurrentSense.h"
#include "common/FOCDriver.h"
#include "common/FOCMotor.h"
#include "common/Sensor.h"
#include "communication/TinyFOCDebug.h"

#endif
