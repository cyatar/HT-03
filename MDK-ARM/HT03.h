#ifndef HT03_H
#define HT03_H
#include <stdint.h>

#include "stm32F4xx_hal.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


typedef struct 
{
    uint16_t motorID;
    uint16_t position;
    uint16_t velocity;
    uint16_t current;
}MotorData;



void can_filter_init();
void CanComm_Init(CAN_HandleTypeDef hcan);
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint16_t CAN_SLAVE_ID,CAN_HandleTypeDef hcan);
void CanComm_ControlCmd(uint8_t cmd,uint16_t CAN_SLAVE_ID,CAN_HandleTypeDef hcan);
void CanComm_DataUpdate(void);
void CanTransmit(uint8_t *buf, uint8_t len,uint16_t CAN_SLAVE_ID,CAN_HandleTypeDef hcan);






#endif