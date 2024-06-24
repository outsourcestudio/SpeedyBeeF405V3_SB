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

#include "common/maths.h"

#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "drivers/motor.h"

#include "rx/rx.h"

motorConfig_t motorConfig;

unsigned short LF, LR, RR, RF;

//TIM4->CCR1 // RR
//TIM4->CCR2 // RF
//TIM4->CCR3 // LR
//TIM4->CCR4 // LF

//ROLL
//angle : +, gyro : +, rx : +

//PITCH
//angle : +, gyro : +, rx : +

//YAW
//angle : +, gyro : -, rx : +  //gyro mul negative sign

void motorConfig_Init(void)
{
  motorConfig.minthrottle = 1070;
  motorConfig.maxthrottle = 2000;
  motorConfig.mincommand = 1000;
  motorConfig.digitalIdleOffsetValue = 550;
  motorConfig.motorPoleCount = 14;   // Most brushes motors that we use are 14 poles
}

void motorShutdown(void)
{

}

void motorWriteAll(void)
{
  if(ARMING_FLAG(ARMED))
  {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Control calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pid.pid_output_left = pid.pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
    pid.pid_output_right = pid.pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

    if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
      pid.pid_output_left += pid.turning_speed;                                       //Increase the left motor speed
      pid.pid_output_right -= pid.turning_speed;                                      //Decrease the right motor speed
    }
    if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
      pid.pid_output_left -= pid.turning_speed;                                       //Decrease the left motor speed
      pid.pid_output_right += pid.turning_speed;                                      //Increase the right motor speed
    }

    if(received_byte & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
      if(pid.pid_setpoint > -2.5)pid.pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
      if(pid.pid_output > pid.max_target_speed * -1)pid.pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
    }
    if(received_byte & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
      if(pid.pid_setpoint < 2.5)pid.pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
      if(pid.pid_output < pid.max_target_speed)pid.pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
    }

    if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
      if(pid.pid_setpoint > 0.5)pid.pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
      else if(pid.pid_setpoint < -0.5)pid.pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
      else pid.pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
    }

    //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
    if(pid.pid_setpoint == 0){                                                    //If the setpoint is zero degrees
      if(pid.pid_output < 0)pid.self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
      if(pid.pid_output > 0)pid.self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor pulse calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
    if(pid.pid_output_left > 0)pid.pid_output_left = 405 - (1/(pid.pid_output_left + 9)) * 5500;
    else if(pid.pid_output_left < 0)pid.pid_output_left = -405 - (1/(pid.pid_output_left - 9)) * 5500;

    if(pid.pid_output_right > 0)pid.pid_output_right = 405 - (1/(pid.pid_output_right + 9)) * 5500;
    else if(pid.pid_output_right < 0)pid.pid_output_right = -405 - (1/(pid.pid_output_right - 9)) * 5500;

    //Calculate the needed pulse time for the left and right stepper motor controllers
    if(pid.pid_output_left > 0)pid.left_motor = 400 - pid.pid_output_left;
    else if(pid.pid_output_left < 0)pid.left_motor = -400 - pid.pid_output_left;
    else pid.left_motor = 0;

    if(pid.pid_output_right > 0)pid.right_motor = 400 - pid.pid_output_right;
    else if(pid.pid_output_right < 0)pid.right_motor = -400 - pid.pid_output_right;
    else pid.right_motor = 0;

    //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
    pid.throttle_left_motor = pid.left_motor;
    pid.throttle_right_motor = pid.right_motor;
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
