/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hw.h"

#include "config/config.h"

#include "common/maths.h"

#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "drivers/motor.h"

#include "rx/rx.h"

motorConfig_t motorConfig;

uint8_t DRIVER_PIN[5] = {5,6,7,8,4};   //STEP1 (PORTD 5), STEP2 (PORTD 6), DIR1 (PORTD 7), DIR2 (PORTB 0), ENABLE

// *************************
// motor and servo functions
// *************************
//int16_t output;
int16_t motor[2];
//int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1000};

int16_t actualMotorSpeed[2];     // actual speed of motors
uint8_t actualMotorDir[2];       // actual direction of steppers motors

int16_t actualSpeed;     // actual speed of robot

/**************************************************************************************/
/************  Calculate and writes the motors values                ******************/
/**************************************************************************************/

uint16_t periodsCounter[2];      // counters for periods
uint16_t subPeriod[2][8];        // eight subperiodPaddings
uint8_t subPeriodIndex[2];       // index for subperiodPaddings

#define ZERO_SPEED  65535
#define MAX_ACCEL   4
//#define REVERSE_MOTORS_DIRECTION


// Divided into 8 sub-periods to increase the resolution at high speeds (short periods)
// subperiodPadding = ((1000 % vel)*8)/vel;
void calculateSubperiods(uint8_t motor) {

  uint8_t subperiodPadding;
  uint16_t absSpeed;
  uint8_t i;

  if (actualMotorSpeed[motor] == 0) {
    for (i=0; i<8; i++) {
      subPeriod[motor][i] = ZERO_SPEED;
    }
    return;
  }

  #ifdef REVERSE_MOTORS_DIRECTION
    actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 1 : 0;
  #else
    actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 0 : 1;
  #endif

  absSpeed = abs(actualMotorSpeed[motor]);

  subPeriod[motor][0] = 1000/absSpeed;
  for (i=1; i<8; i++) {
    subPeriod[motor][i] = subPeriod[motor][0];
  }
  // Calculate the sub-period padding.
  subperiodPadding = ((1000 % absSpeed)*8)/absSpeed;
  if (subperiodPadding > 0) {
    subPeriod[motor][1]++;
  }
  if (subperiodPadding > 1) {
    subPeriod[motor][5]++;
  }
  if (subperiodPadding > 2) {
    subPeriod[motor][3]++;
  }
  if (subperiodPadding > 3) {
    subPeriod[motor][7]++;
  }
  if (subperiodPadding > 4) {
    subPeriod[motor][0]++;
  }
  if (subperiodPadding > 5) {
    subPeriod[motor][4]++;
  }
  if (subperiodPadding > 6) {
    subPeriod[motor][2]++;
  }
}

static void TimerCallbackISR(void)
{
  periodsCounter[0]++;
  periodsCounter[1]++;

  if (periodsCounter[0] >= subPeriod[0][subPeriodIndex[0]]) {
    periodsCounter[0] = 0;

    if (subPeriod[0][0] != ZERO_SPEED) {
      if (actualMotorDir[0]) {
        gpioPinWrite(Step_left_DIR, _DEF_HIGH);
        //SET(PORTD,7);  // DIR Motor 1
      } else {
        gpioPinWrite(Step_left_DIR, _DEF_LOW);
        //CLR(PORTD,7);
      }
      // We need to wait at lest 200ns to generate the Step pulse...
      subPeriodIndex[0] = (subPeriodIndex[0]+1)&0x07; // subPeriodIndex from 0 to 7

      gpioPinWrite(Step_left_STEP, _DEF_HIGH);
      //SET(PORTD,5); // STEP Motor 1
      delayMicroseconds(1);
      gpioPinWrite(Step_left_STEP, _DEF_LOW);
      //CLR(PORTD,5);
    }
  }

  if (periodsCounter[1] >= subPeriod[1][subPeriodIndex[1]]) {
    periodsCounter[1] = 0;

    if (subPeriod[1][0] != ZERO_SPEED) {

      if (actualMotorDir[1]) {
        gpioPinWrite(Step_right_DIR, _DEF_HIGH);
        //SET(PORTB,0);   // DIR Motor 2
      } else {
        gpioPinWrite(Step_right_DIR, _DEF_LOW);
        //CLR(PORTB,0);
      }
      subPeriodIndex[1] = (subPeriodIndex[1]+1)&0x07;

      gpioPinWrite(Step_right_STEP, _DEF_HIGH);
      //SET(PORTD,6); // STEP Motor 1
      delayMicroseconds(1);
      gpioPinWrite(Step_right_STEP, _DEF_LOW);
      //CLR(PORTD,6);
    }
  }

}

void motorConfig_Init(void)
{
  motorConfig.minthrottle = 1070;
  motorConfig.maxthrottle = 2000;
  motorConfig.mincommand = 1000;
  motorConfig.digitalIdleOffsetValue = 550;
  motorConfig.motorPoleCount = 14;   // Most brushes motors that we use are 14 poles

  timAttachInterrupt(TimerCallbackISR);
}

void setMotorSpeed(uint8_t motorNum, int16_t speed) {

  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  // LIMIT MAX ACCELERATION
  int16_t acceleration = speed - actualMotorSpeed[motorNum];
  if (acceleration > MAX_ACCEL) {
    actualMotorSpeed[motorNum] += MAX_ACCEL;
  } else if (acceleration < -MAX_ACCEL) {
    actualMotorSpeed[motorNum] -= MAX_ACCEL;
  } else {
    actualMotorSpeed[motorNum] = speed;
  }

  calculateSubperiods(motorNum);  // We use four subperiodPaddings to increase resolution

  // To save energy when its not running...
  if ((actualMotorSpeed[0] == 0) && (actualMotorSpeed[1] == 0)) {
    gpioPinWrite(Step_left_EN, _DEF_HIGH);   // Disable motors
    gpioPinWrite(Step_right_EN, _DEF_HIGH);   // Disable motors
  } else {
    gpioPinWrite(Step_left_EN, _DEF_LOW);   // Enable motors
    gpioPinWrite(Step_right_EN, _DEF_LOW);   // Enable motors
  }
}

void motorShutdown(void)
{

}

void motorWriteAll(void)
{
  if(ARMING_FLAG(ARMED))
  {

  }
}

void motorDisable(void)
{
  TIM4->CCR1 = 10500;
  TIM4->CCR2 = 10500;
  TIM4->CCR3 = 10500;
  TIM4->CCR4 = 10500;
}

void motorEnable(void)
{

}

bool motorIsEnabled(void)
{
    return false;
}
