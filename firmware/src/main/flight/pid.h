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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "hw.h"
#include "common/time.h"
#include "common/axis.h"

typedef struct pid_s {
  bool start;

  float pid_p_gain;
  float pid_i_gain;
  float pid_d_gain;

  float self_balance_pid_setpoint;
  float pid_setpoint;


  float pid_error_temp;
  float pid_i_mem;
  float pid_output;
  float pid_last_d_error;

  float pid_output_left;
  float pid_output_right;

  float turning_speed;
  float max_target_speed;

  int left_motor;
  int right_motor;

  int throttle_left_motor;
  int throttle_right_motor;
} pid_t;


extern pid_t pid;



void PID_Calculation(float set_point_angle, float angle, float rate);


void Reset_PID_Integrator(pid_t* axis);
void Reset_All_PID_Integrator(void);

void pidInit(void);
void taskMainPidLoop(timeUs_t currentTimeUs);

#ifdef __cplusplus
}
#endif
#endif /*__PID_CONTROL_H */
