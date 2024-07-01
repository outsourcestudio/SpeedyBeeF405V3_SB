/*
 * stepper.h
 *
 *  Created on: Feb 20, 2024
 *      Author: WANG
 */

#ifndef SRC_DRIVERS_STPPER_STEPPER_H_
#define SRC_DRIVERS_STPPER_STEPPER_H_

#include "hw.h"

typedef enum
{
  DIRECTION_CCW = 0,  ///< Counter-Clockwise
  DIRECTION_CW  = 1   ///< Clockwise
} Direction;

typedef struct
{
  /// Current direction motor is spinning in
  /// Protected because some peoples subclasses need it to be so
  bool _direction; // 1 == CW

  /// The current interval between steps in microseconds.
  /// 0 means the motor is currently stopped with _speed == 0
  unsigned long  _stepInterval;

  /// The current absolution position in steps.
  long           _currentPos;    // Steps

  /// The target position in steps. The AccelStepper library will move the
  /// motor from the _currentPos to the _targetPos, taking into account the
  /// max speed, acceleration and deceleration
  long           _targetPos;     // Steps

  /// The current motos speed in steps per second
  /// Positive is clockwise
  float          _speed;         // Steps per second

  /// The maximum permitted speed in steps per second. Must be > 0.
  float          _maxSpeed;

  /// The acceleration to use to accelerate or decelerate the motor in steps
  /// per second per second. Must be > 0
  float          _acceleration;
  float          _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

  /// The last step time in microseconds
  unsigned long  _lastStepTime;

  /// The minimum allowed pulse width in microseconds
  unsigned int   _minPulseWidth;

  /// Is the enable pin inverted?
  bool           _enableInverted;

  /// Enable pin for stepper driver, or 0xFF if unused.
  uint8_t        _enablePin;
  uint8_t        _dirPin;
  uint8_t        _stepPin;

  /// The pointer to a forward-step procedure
  void (*_forward)();

  /// The pointer to a backward-step procedure
  void (*_backward)();

  /// The step counter for speed calculations
  long _n;

  /// Initial step size in microseconds
  float _c0;

  /// Last step size in microseconds
  float _cn;

  /// Min step size in microseconds based on maxSpeed
  float _cmin; // at max speed

}AccelStepper;

bool stepper_Init(AccelStepper *stepper, uint8_t enablePin, uint8_t dirPin, uint8_t stepPin);

long distanceToGo(AccelStepper *stepper);
long targetPosition(AccelStepper *stepper);
long currentPosition(AccelStepper *stepper);

void moveTo(AccelStepper *stepper, long absolute);
void step_move(AccelStepper *stepper, long relative);

bool run(AccelStepper *stepper);

void setMaxSpeed(AccelStepper *stepper, float speed);
void setAcceleration(AccelStepper *stepper, float acceleration);
float acceleration(AccelStepper *stepper);
void setSpeed(AccelStepper *stepper, float speed);
float speed(AccelStepper *stepper);

void disableOutputs(AccelStepper *stepper);
void enableOutputs(AccelStepper *stepper);

unsigned long computeNewSpeed(AccelStepper *stepper);

void step(AccelStepper *stepper, long step);

bool runSpeed(AccelStepper *stepper);
bool runSpeedToPosition(AccelStepper *stepper);



#endif /* SRC_DRIVERS_STPPER_STEPPER_H_ */
