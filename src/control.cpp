#include "config.h"
#include "control.h"
#include "imu.h"
#include "motor.h"
#include "legs.h"
#include "math.h"
#include "ble.h"

#define BALANCE_OFFSET  -0.065//-0.045   // 增大往后，减小往前
#define BALANCE_OFFSET1 -0.065//-0.045   // 1.2
#define BALANCE_OFFSET2 -0.85   
#define LEG_LENGTH_OFFSET 0.03


//#define BALANCE_OFFSET 0
static bool GroundTouch_Chech(float set_Pos);

void Balance_Ctrl(Motor *motor, float targetValue, float *outValue);
void Velocity_Ctrl(Motor *motor, float targetValue,  float *outValue);
void PidModelReset(PidModle *pid);
void LegLength_Ctrl(float setPos, float *leftLength, float *rightLength);
bool StopProtect_Checking();
float BalanceOffset_Select(float legPos);
void Robot_Jump();
void Run_Ctrl(float targetValue,  float *outValue);
// PidModle PidBal = {50.f,0,80.f,0,0,0,0,0};
// PidModle PidBal_Out = {1.1f,0.00005f,2.3f,0,0,0,0,0};   //{1.13,0,0.25,0,0,0,0,0}
float PidVel_Kp = 0.03f;
float PidVel_Ki = PidVel_Kp / 200.0;
float PidVel_Kd = 0.0f;
PidModle PidVel_left = {PidVel_Kp,PidVel_Ki,PidVel_Kd,0,0,0,0,0};
PidModle PidVel_right = {PidVel_Kp,PidVel_Ki,PidVel_Kd,0,0,0,0,0};
// PidModle PidYaw = {2.5,0,1.5,0,0,0,0,0};
float speed_all = 0;
float speed[2] = {speed_all, speed_all};
float targetYaw = 0;
//1.5
float PidBal_Kp = 9.3f;   //9.3
float PidBal_Ki = 0.f;
float PidBal_Kd = 130.f;  //100.0
PidModle PidBal_Left = {PidBal_Kp, PidBal_Ki, PidBal_Kd,0,0,0,0,0};
PidModle PidBal_Right = {PidBal_Kp, PidBal_Ki, PidBal_Kd,0,0,0,0,0};
//1.2
float PidBal_Kp1 = 15.f;
float PidBal_Ki1 = 0.f;
float PidBal_Kd1 = 18.f;
PidModle PidBal_Left1 = {PidBal_Kp1, PidBal_Ki1, PidBal_Kd1,0,0,0,0,0};
PidModle PidBal_Right1 = {PidBal_Kp1, PidBal_Ki1, PidBal_Kd1,0,0,0,0,0};
//1.0
float PidBal_Kp2 = 17.f;
float PidBal_Ki2 = 0.f;
float PidBal_Kd2 = 21.f;
PidModle PidBal_Left2 = {PidBal_Kp2, PidBal_Ki2, PidBal_Kd2,0,0,0,0,0};
PidModle PidBal_Right2 = {PidBal_Kp2, PidBal_Ki2, PidBal_Kd2,0,0,0,0,0};
PidModle PidBal[3][2] = {{PidBal_Left,PidBal_Right}, {PidBal_Left1, PidBal_Right1},{PidBal_Left2, PidBal_Right2}};

PidModle PidVel = {0,0,0,0,0,0,0,0};
PidModle PidYaw = {3.0,0,90.0,0,0,0,0,0};
PidModle PidLegs = {1.0,0.03,0,0,0,0,0,0};
PidModle PidRun = {1.0,0,0,0,0,0,0,0};
//float setLegPos = 1.2; 
//0.9
// PidModle PidBal = {50.f,0,80.f,0,0,0,0,0};
// PidModle PidBal_Out = {1.15f,0.00001f,2.0f,0,0,0,0,0};   //{1.13,0,0.25,0,0,0,0,0}
// PidModle PidVel = {0,0,0,0,0,0,0,0};
// PidModle PidYaw = {1.5,0,0.9,0,0,0,0,0};
float setLegPos = 1.5; 
float balanceOffset = BALANCE_OFFSET;
float workState = STATE_NORMAL;
    
void Ctrl_Init()
{
    xTaskCreate(Ctrl_Task, "Ctrl_Task", 4096, NULL, 2, NULL);
    //xTaskCreate(Operator_Task, "Ctrl_Task", 2048, NULL, 5, NULL);
}

void Ctrl_Task(void *arg)
{
    float balanceOut_left = 0;
    float balanceOut_right = 0;

    float velocityOut_left = 0;
    float velocityOut_right = 0;
    float yawPidOut = 0;
    float leftLength = 0;
    float rightLength = 0;
    float runOut = 0;
    targetYaw = imuData.yaw;
    while(1){
        // if(GroundTouch_Chech(setLegPos)){
        //     Motor_SetTargetVel(&leftWheel, 0);
        //     Motor_SetTargetVel(&rightWheel, 0);
        //     digitalWrite(LED, HIGH);
        //     vTaskDelay(5);
        //     continue;
        // }
        // else{
        //     digitalWrite(LED, LOW);
        // }
   
        
        if(StopProtect_Checking()){
            continue;
        }
        // 遥控环
        // Run_Ctrl(speed[0],  &runOut);
        // 速度环
        Velocity_Ctrl(&leftWheel, speed[0], &velocityOut_left);
        Velocity_Ctrl(&rightWheel, speed[1], &velocityOut_right);
        // 平衡环
        Balance_Ctrl(&leftWheel, velocityOut_left, &balanceOut_left);
        Balance_Ctrl(&rightWheel, velocityOut_right, &balanceOut_right);
        // 角度环
        Yaw_Ctrl(targetYaw, yawPidOut);
        //Motor_SetTorque(&leftWheel, balanceOut - yawPidOut);
        Motor_SetTargetVel(&leftWheel, balanceOut_left - yawPidOut);
        Motor_SetTargetVel(&rightWheel, balanceOut_right + yawPidOut);
        // float in[2];
        // in[0] = imuData.yaw;
        // in[1] = imuData.yAccel;
        // vofaPrint((char*)in,2);
        // Motor_SetPosition(&leftJoint[0], 0);
        // Motor_SetPosition(&leftJoint[1], 0);
        // 腿长环
        LegLength_Ctrl(setLegPos, &leftLength, &rightLength);
        Leg_SetPosition(&leftLeg, leftLength);
        Leg_SetPosition(&rightLeg, rightLength);
     
        vTaskDelay(5);

    }

}


void Balance_Ctrl(Motor *motor, float targetValue, float *outValue)
{
    PidModle *pidBal;
   
    uint8_t selectPid = 0;
    // if(setLegPos <= 1.5 && setLegPos > 1.3){
    //     selectPid = 0;
    // }
    // else if(setLegPos <= 1.3 && setLegPos > 1.1){
    //     selectPid = 1;
    // }
    //  else if(setLegPos <= 1.1){
    //     selectPid = 2;
    // }
  
    if(motor->id == 3){
        pidBal = &PidBal[selectPid][0];
        //PidModelReset(&PidBal[1-selectPid][0]);
    }
    else if(motor->id == 7){
        pidBal = &PidBal[selectPid][1];
        //PidModelReset(&PidBal[1-selectPid][1]);
    }
    else{
        return;
    }

    float realValue = -asin(imuData.ygravity + balanceOffset) / 1.0;
    pidBal->error = realValue  - targetValue;
    pidBal->pid_out = pidBal->kp * pidBal->error + pidBal->kd * (pidBal->error - pidBal->p_error);
    pidBal->p_error = pidBal->error;
    
    *outValue = pidBal->pid_out;
   
}

void Velocity_Ctrl(Motor *motor, float targetValue,  float *outValue)
{

    PidModle *pidWheelVel = nullptr;
    static float pidOutLeft_last = 0;
    static float pidOutRight_last = 0;
    float pidOut_last = 0;
	float motor_UpMap = 0;
	float motor_DownMap = 0;
	if(motor->id == 3){
		pidWheelVel = &PidVel_left;
		motor_UpMap = LEFT_MOTOR_UP_MAP;
		motor_DownMap = LEFT_MOTOR_DOWN_MAP;
        pidOut_last = pidOutLeft_last;
	}
	else if(motor->id == 7){
		pidWheelVel = &PidVel_right;
		motor_UpMap = RIGHT_MOTOR_UP_MAP;
		motor_DownMap = RIGHT_MOTOR_DOWN_MAP;
        pidOut_last = pidOutRight_last;
	}
	if(pidWheelVel == nullptr) return;
	pidWheelVel->error =  motor->speed / motor_UpMap - targetValue;
    pidWheelVel->I += pidWheelVel->error;
    float I = pidWheelVel->ki * pidWheelVel->I;
    float limit = fabs(pidWheelVel->error * 0.005);
    I = I > limit ? limit : I;
    I = I < -limit ? -limit : I;
    pidWheelVel->pid_out = pidWheelVel->kp * pidWheelVel->error + I + pidWheelVel->kd * (pidWheelVel->error - pidWheelVel->p_error); 
	pidWheelVel->p_error = pidWheelVel->error;
    pidWheelVel->I = I / pidWheelVel->ki;    // 同时更新积分限幅
	//float targetV = (int)((targetValue + pidWheelVel->pid_out)*1000) / 1000.0;
	// targetV = targetV > 0.3 ? 0.3 : targetV;
	// targetV = targetV < -0.3 ? -0.3 : targetV;
	*outValue = flitter(0.1, pidWheelVel->pid_out, pidOut_last);
    if(motor->id == 3){
        pidOutLeft_last = pidWheelVel->pid_out;
    }
    else if(motor->id == 7){
        pidOutRight_last = pidWheelVel->pid_out;
    }
  
}


float nonlinear(float value)
{
    if(value >= 0){
        return int(pow(value, 1/1.7)*1000)/1000.0;
    }
    else {
        return int(-pow(-value, 1/1.7)*1000)/1000.0;
    }
}

float nolinar_error(float k,float error){
	//非线性
	int8_t k1 = 1;
	if(error < 0)
	{		
		k1= -k1;
		k = -k;
	}
	error = int(log(error*k1+1)*k*1000)/1000.0;
	return error;
}

void Yaw_Ctrl(float targetYaw,float &out)
{
    float seTatget = -(targetYaw - imuData.yaw) / 2.0;
    PidYaw.error = imuData.yaw - seTatget;
    PidYaw.pid_out = PidYaw.kp * PidYaw.error + PidYaw.kd * (PidYaw.error - PidYaw.p_error);
    PidYaw.p_error = PidYaw.error;
    out = PidYaw.pid_out;
}
/*
** true:  离地
** false: 触地
*/
static bool GroundTouch_Chech(float set_Pos)
{
    float limit = 0.01;
    // Serial.print(Motor_GetPosition(&leftJoint[0]));
    // Serial.print("   ");
    // Serial.print(Motor_GetPosition(&leftJoint[1]));
    // Serial.print("   ");
    // Serial.print(Motor_GetPosition(&rightJoint[0]));
    // Serial.print("   ");
    // Serial.println(Motor_GetPosition(&rightJoint[1]));
    if(fmod(Motor_GetPosition(&leftJoint[1]), PI*2) - limit > set_Pos 
        && fmod(Motor_GetPosition(&rightJoint[1]), PI*2) - limit > set_Pos){
        return false;
    }
    return true;
}

void PidModelReset(PidModle *pid)
{
    pid->error = 0;
    pid->I = 0;
    pid->p_error = 0;
    pid->pid_out = 0;
    pid->pp_error = 0;
}

void LegLength_Ctrl(float setPos, float *leftLength, float *rightLength)
{
    static float I = 0;
    PidLegs.error = asin(imuData.xgravity - LEG_LENGTH_OFFSET);
    PidLegs.I += PidLegs.error;
    I = PidLegs.ki * PidLegs.I;
    PidLegs.pid_out = PidLegs.kp * PidLegs.p_error + I;
    PidLegs.p_error = PidLegs.error;
    *leftLength = setPos + PidLegs.pid_out;
    *rightLength = setPos - PidLegs.pid_out;
    // if(*leftLength > *rightLength){
    //    balanceOffset = BalanceOffset_Select(*leftLength);
    // }
    // else if(*leftLength < *rightLength)
    // {
    //     balanceOffset = BalanceOffset_Select(*rightLength);
    // }
}

bool StopProtect_Checking()
{
    if(fabs(asin(imuData.ygravity + BALANCE_OFFSET)/1.0) > 0.25 ){
        Motor_SetTargetVel(&leftWheel, 0);
        Motor_SetTargetVel(&rightWheel, 0);
        return true;
    }
    return false;
}

float BalanceOffset_Select(float legPos)
{
    if(legPos <= 1.6 && legPos > 1.3){
        return BALANCE_OFFSET;
    }
    else if(legPos <= 1.3 && legPos > 1.0){
        return BALANCE_OFFSET1;
    }
    else{
        return BALANCE_OFFSET2;
    }
   

}

void Robot_Jump()
{
    digitalWrite(LED, HIGH);
    Leg_SetPosition(&leftLeg, JUMP_POSITON_LOW);
    Leg_SetPosition(&rightLeg, JUMP_POSITON_LOW);
    workState = STATE_JUMP;
    vTaskDelay(1000);
    Leg_UpdateData(&leftLeg, JUMP_POSITON_HIGH);
    Leg_UpdateData(&rightLeg, JUMP_POSITON_HIGH);
    vTaskDelay(500);
    Leg_UpdateData(&leftLeg, JUMP_POSITON_LOW);
    Leg_UpdateData(&rightLeg, JUMP_POSITON_LOW);
    vTaskDelay(500);
    workState = STATE_NORMAL;
    Leg_SetPosition(&leftLeg, JUMP_POSITON_LOW);
    Leg_SetPosition(&rightLeg, JUMP_POSITON_LOW);
    digitalWrite(LED, LOW);
}

void Operator_Task(void * args)
{
    int count = 0;
    while(1){
        count++;
        if(count == 30){
            Robot_Jump();
        }
        vTaskDelay(100);
    }
}

void Run_Ctrl(float targetValue,  float *outValue)
{
    PidRun.error = -imuData.yAccel + targetValue/100.0;
    PidRun.pid_out = PidRun.kp * PidRun.error + PidRun.kd * (PidRun.error - PidRun.p_error);
    PidRun.p_error = PidRun.error;
    *outValue = PidRun.pid_out;
}