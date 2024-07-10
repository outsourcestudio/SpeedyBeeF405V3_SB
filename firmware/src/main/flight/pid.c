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

#include "config/config.h"

#include "sensors/gyro.h"
#include "flight/imu.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "drivers/motor.h"

#include "fc/runtime_config.h"


pid_t pid;


void PID_Calculation(float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{

}

void Reset_PID_Integrator(pid_t * axis)
{

}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&pid);
}

void pidInit(void)
{
  pid.POSHOLD_P = 60.0f;

  pid.pid[PIDSPEED].P8    = 80;   pid.pid[PIDSPEED].I8    = 0; pid.pid[PIDSPEED].D8    =  0;
  pid.pid[PIDANGLE].P8    = 70;   pid.pid[PIDANGLE].I8    = 0; pid.pid[PIDANGLE].D8    = 40;


  pid.pid[PIDPOS].P8  = pid.POSHOLD_P * 100;

  pid.angleErrorI = 0;
  pid.speedErrorI = 0;
  pid.positionError = 0.0f;
  pid.actualAveragedSpeed = 0.0f;
  pid.speed = 0.0f;
}

//#define INVERT_CURRENT_AXIS

// Function for loop trigger
void taskMainPidLoop(timeUs_t currentTimeUs)
{
  pid.cycleTime = currentTimeUs - pid.previousUpdateTimeUs;
  pid.previousUpdateTimeUs = currentTimeUs;

  //***********************************//
  //****       BalancingWii       *****//
  //***********************************//

  /****************** PI_speed + PD_angle regulator *****************/
  pid.targetSpeed = constrain(rcCommand[PITCH], -MAX_SPEED, MAX_SPEED);
  pid.steering = constrain((int)-rcCommand[ROLL]>>2, -MAX_STEERING, MAX_STEERING);
  pid.steering = FLIGHT_MODE(SIMPLE_MODE) ? (pid.steering*2/3) : pid.steering;

  actualSpeed = (actualMotorSpeed[1] - actualMotorSpeed[0])/2;  // Positive: forward


  /**** position hold mode ****/
  //static float positionError = 0.0f;
  if(FLIGHT_MODE(POSHOLD_MODE) && abs(pid.targetSpeed) < 15 && abs(pid.steering) < 15) {
    pid.positionError += actualSpeed * (float)pid.cycleTime * 0.000001f;
  } else {
    pid.positionError = 0.0f;
  }


  /**** PI_speed regulator ****/
  //static float actualAveragedSpeed = 0.0f;
  pid.actualAveragedSpeed = pid.actualAveragedSpeed * 0.92f + (float)actualSpeed * 0.08f;
  pid.error = pid.targetSpeed - pid.actualAveragedSpeed -(pid.positionError * pid.pid[PIDPOS].P8 * 0.01f);  //16 bits is ok here

  pid.speedErrorI = constrain(pid.speedErrorI + (int16_t)(((int32_t)pid.error * pid.cycleTime)>>11), -20000, 20000);    //16 bits is ok here

  int16_t maxTargetAngle = FLIGHT_MODE(SIMPLE_MODE) ? (MAX_TARGET_ANGLE*2/3) : MAX_TARGET_ANGLE;

  pid.targetAngle = // PTerm + ITerm
        (((int32_t)pid.error * pid.pid[PIDSPEED].P8)>>7)           // 32 bits is needed for calculation: angleError*P8 could exceed 32768   16 bits is ok for result
        + constrain( (((int32_t)pid.speedErrorI * pid.pid[PIDSPEED].I8)>>14), -maxTargetAngle/6, maxTargetAngle/6);   // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

  pid.targetAngle = constrain(pid.targetAngle, -maxTargetAngle, maxTargetAngle);


  /**** PD_angle regulator ****/
  pid.currAngle = (int16_t)attitude.values.pitch - 40;// + conf.angleTrim[CURRENT_AXIS];
  pid.gyro_pitch = bmi270.gyroADCf[Y] * 10;
  #ifdef INVERT_CURRENT_AXIS
    pid.currAngle = -pid.currAngle;
    pid.gyro_pitch = -pid.gyro_pitch;
  #endif
  pid.angleError =  pid.targetAngle - pid.currAngle; //16 bits is ok here

  pid.acceleration = // PTerm - DTerm
        (((int32_t)pid.angleError * pid.pid[PIDANGLE].P8)>>4)                      // 32 bits is needed for calculation: error*P8 could exceed 32768   16 bits is ok for result
        - (((int32_t)pid.gyro_pitch * pid.pid[PIDANGLE].D8)>>5);     // 32 bits is needed for calculation

  //static float speed = 0.0f;
  pid.speed = constrain(pid.speed + ((float)pid.acceleration * (float)pid.cycleTime * 0.000001f), -MAX_SPEED, MAX_SPEED);


  /**** rise mode ****/

  #define MAX_RISE_SPEED    140
  #define MAX_REVERSED_RISE_SPEED 100

  static uint8_t risePhase = 2; // to prevent rising without switching off before
  float dynK = 0.0f;
  if(ARMING_FLAG(ARMED)) {
    int16_t currAbsAngle = abs(pid.currAngle);
    if(currAbsAngle < 250) {  // if angle less than 25 degree
      dynK = 1.0f;

    } else if(currAbsAngle < 800) { // help to rise with less speed but more torque
      dynK = (1000.0f - currAbsAngle) / 1000.0f + 0.08f;
      risePhase = 2; // to prevent rising without switching off before

    } else {
      dynK = 1.0f;

      if(FLIGHT_MODE(RISE_MODE)) { // if robot fell, use it to auto rise! ;)
        static float riseSpeed = 0;
        if(risePhase == 0) { // get direct acceleration
          riseSpeed = constrain(riseSpeed + (0.7f * RISE_SPEED_K), 0, MAX_RISE_SPEED);
          pid.speed = (pid.currAngle > 0) ? riseSpeed : -riseSpeed; // forward direction
          if(riseSpeed >= MAX_RISE_SPEED) {
            riseSpeed = 0.0f; // force stop (it will throw up the robot) and prepare for next phase in reverse
            risePhase = 1;
          }
        } else if(risePhase == 1) { // get reversed acceleration to rise
          riseSpeed = constrain(riseSpeed + (0.85f * RISE_SPEED_K), 0, MAX_REVERSED_RISE_SPEED);
          pid.speed = (pid.currAngle > 0) ? -riseSpeed : riseSpeed; // backward direction
          if(riseSpeed >= MAX_REVERSED_RISE_SPEED) {
            risePhase = 2;
          }
        } else if(risePhase == 2) { // prepare for the next rise
          riseSpeed = 0.0f;
          pid.speed = 0.0f;
        }
        pid.steering = 0; // to prevent turning during auto rising

      } else { // if manual mode for rising
        pid.speed = constrain(-pid.targetSpeed/2, -MAX_SPEED/2, MAX_SPEED/2);
        pid.steering = (abs(pid.targetSpeed) < 100) ? pid.steering/2 : 0; // to prevent turning during acceleration
        risePhase = 0; // reset rise phase
      }
    }

  } else { // turn off the motors
    pid.speed = 0.0f;
    pid.steering = 0;
    risePhase = 2; // to prevent rising without switching off before
  }

  pid.outputSpeed = constrain(pid.speed * dynK, -MAX_SPEED, MAX_SPEED); ;

  // to don't lost a control on big speeds and not overlimit the MAX_SPEED
  if((abs(pid.outputSpeed) + abs(pid.steering)) > MAX_SPEED) {
    pid.outputSpeed = (pid.outputSpeed > 0) ? (MAX_SPEED - abs(pid.steering)) : (-MAX_SPEED + abs(pid.steering));
  }

  // apply both motor speed
  setMotorSpeed(0, pid.outputSpeed + pid.steering);    // right motor
  setMotorSpeed(1, -pid.outputSpeed + pid.steering);   // left motor
}
