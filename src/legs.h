#ifndef __LEG_H__
#define __LEG_H__
#include "motor.h"
#include <stdio.h>

typedef struct LegModel Leg;
struct LegModel{
    Motor *frontJoint;
    Motor *behindJoint;
    Motor *wheel;
    float wheelPos_x;
    float wheelPos_y;
    float pos;
};


void Leg_InitAll(void);
void Leg_Init(Leg *leg, Motor *leftJoint, Motor *rightJoint, Motor *wheel);
void Leg_SetPosition(Leg *leg, float pos);
void leg_pos(float phi1, float phi4, float pos[2]);
float Leg_GetPosition(Leg* leg);
void Leg_UpdateData(Leg *leg, float pos);

extern Leg leftLeg;
extern Leg rightLeg;
extern float legPos;

#endif