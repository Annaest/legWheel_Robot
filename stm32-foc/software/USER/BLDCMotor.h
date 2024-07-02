#ifndef BLDCMotor_H
#define BLDCMotor_H

#include <stdint.h>
/******************************************************************************/
/**
 *  Direction structure
 */
typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

/******************************************************************************/
extern long sensor_direction;
extern float voltage_power_supply;
extern float voltage_limit;
extern float voltage_sensor_align;
extern int  pole_pairs;
extern unsigned long open_loop_timestamp;
extern float velocity_limit;

extern float targetVotage;    //当前目标电压
extern float targetPosition;  //当前目标位置
extern uint8_t motorID;
extern uint8_t start_flag;
extern float speed;
/******************************************************************************/
void Motor_init(void);
void Motor_initFOC(void);
void loopFOC(void);
void setTargetVotage(float new_target);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
/******************************************************************************/

#endif
