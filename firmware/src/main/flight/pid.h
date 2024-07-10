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

enum pid {
  PIDSPEED,
  //PIDROLL
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDANGLE,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

typedef struct pid_ {
  uint8_t P8;
  uint8_t I8;
  uint8_t D8;
} pid__t;

typedef struct pid_s {
  pid__t pid[PIDITEMS];

  float POSHOLD_P;

  int32_t gyro_pitch;

  int16_t error;
  timeUs_t previousUpdateTimeUs;
  int32_t cycleTime;
  int16_t targetSpeed;
  int16_t steering;
  float positionError;
  float actualAveragedSpeed;
  int16_t angleError;
  int16_t acceleration;
  float speed;
  int16_t angleErrorI;
  int16_t speedErrorI;
  int16_t targetAngle;
  int16_t currAngle;
  int16_t outputSpeed;

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
