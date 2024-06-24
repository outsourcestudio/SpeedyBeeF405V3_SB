//References:
// https://www.servotecnica.com/en/resources/white-papers-en-mobile/dual-loop-advanced-control-techniques-for-real-world-drivetrains/
// https://controlguru.com/the-cascade-control-architecture/

/**
 * PID control.c
 * @author ChrisP @ M-HIVE

 * This library source code is for cascade double loop pid control for STM32 Drone Development online course.
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2020
 * Rev. 1.0
 *
 * Where to take the online course.
 * https://www.inflearn.com/course/STM32CubelDE-STM32F4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C (Korean language supported only)
 *
 * Where to buy MH-FC V2.2 STM32F4 Drone Flight Controller.
 * https://smartstore.naver.com/mhivestore/products/4961922335
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
*/

#include "pid.h"
#include "sensors/gyro.h"
#include "flight/imu.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "drivers/motor.h"

#include "fc/runtime_config.h"


pid_t pid;


void PID_Calculation(float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{
  pid.pid_error_temp = rate - pid.self_balance_pid_setpoint - pid.pid_setpoint;
  if(pid.pid_output > 10 || pid.pid_output < -10)pid.pid_error_temp += pid.pid_output * 0.015 ;

  pid.pid_i_mem += pid.pid_i_gain * pid.pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid.pid_i_mem > 400)pid.pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid.pid_i_mem < -400)pid.pid_i_mem = -400;
  //Calculate the PID output value
  pid.pid_output = pid.pid_p_gain * pid.pid_error_temp + pid.pid_i_mem + pid.pid_d_gain * (pid.pid_error_temp - pid.pid_last_d_error);
  if(pid.pid_output > 400)pid.pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid.pid_output < -400)pid.pid_output = -400;

  pid.pid_last_d_error = pid.pid_error_temp;                                        //Store the error for the next loop

  if(pid.pid_output < 5 && pid.pid_output > -5)pid.pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(rate > 30 || rate < -30 || pid.start == 0){    //If the robot tips over or the start variable is zero or the battery is empty
    pid.pid_output = 0;                             //Set the PID controller output to 0 so the motors stop moving
    pid.pid_i_mem = 0;                              //Reset the I-controller memory
    pid.start = 0;                                  //Set the start variable to 0
    pid.self_balance_pid_setpoint = 0;              //Reset the self_balance_pid_setpoint variable
  }
}

void Reset_PID_Integrator(pid_t * axis)
{
	axis->pid_i_mem = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&pid);
}

void pidInit(void)
{
  pid.pid_p_gain = 15;
  pid.pid_i_gain = 1.5;
  pid.pid_d_gain = 30;
}



// Function for loop trigger
void taskMainPidLoop(timeUs_t currentTimeUs)
{
	float imu_pitch;

	imu_pitch = (float)attitude.values.pitch/10;

  static timeUs_t previousUpdateTimeUs;
  const float dT = US2S(currentTimeUs - previousUpdateTimeUs);
  previousUpdateTimeUs = currentTimeUs;

  PID_Calculation(rcCommand[PITCH], imu_pitch, bmi270.gyroADCf[Y]);

  if(rcData[THROTTLE] < 1030 || !ARMING_FLAG(ARMED))
  {
	  Reset_All_PID_Integrator();
  }

  if(rcData[YAW] < 1485 || rcData[YAW] > 1515)
  {

	  //LF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;
	  //LR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
	  //RR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
	  //RF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
  }
  else
  {
	  //LF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result;
	  //LR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
	  //RR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result;
	  //RF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
  }

  motorWriteAll();
}
