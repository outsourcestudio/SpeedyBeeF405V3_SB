/*
 * config.h
 *
 *  Created on: Jul 1, 2024
 *      Author: WANG
 */

#ifndef SRC_MAIN_CONFIG_CONFIG_H_
#define SRC_MAIN_CONFIG_CONFIG_H_

#include "hw.h"

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

    #define CURRENT_AXIS    PITCH       // possible to choose ROLL or PITCH axis as current.

    //#define INVERT_CURRENT_AXIS       // invert current axis sign, i.e. instead of turning sensor board

    //#define REVERSE_MOTORS_DIRECTION  // reverse both motors direction

    #define MAX_SPEED           350  // should be <= 500
    #define MAX_TARGET_ANGLE    130  // where 10 = 1 degree, should be <= 15 degree (i.e. <= 150)
    #define MAX_STEERING        90   // should be <= 100

    #define RISE_SPEED_K    1.0f // this coefficient means how faster robot will be during the auto rising...  should be >= 0.5 but <= 2.0


#endif /* SRC_MAIN_CONFIG_CONFIG_H_ */
