#ifndef STEP_ENGINE_H
#define STEP_ENGINE_H

#include <stm32f405xx.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "math.h"
//#include "mbport.h"
#include "config_DWIN.h"

/* DEFINE BEGIN */

#define BASE_FREQ 									1e+5
#define ACCEL_TIME_SEC 							1.0
#define DECEL_TIME_SEC 							1.0
#define ACCEL_TIME 							  	(ACCEL_TIME_SEC*BASE_FREQ)
#define DECEL_TIME 									DECEL_TIME_SEC*BASE_FREQ
#define ACCEL 											0.000000001	
#define SPEED_MIN								  	10.0/65535
#define SPEED_MAX 									0.1

#define ANFLE_ONE_STEP							0.9
#define TOGGLE_DIVIDER							2

#define STOP 1
#define SPEEDUP 2
#define RUN 3
#define SLOWDOWN 4

/* DEFINE END */

typedef struct step_engine {
	float vel;
	float accel;
	size_t accel_size;
	size_t cur_accel_size;
	uint8_t mode;
	int dir;
	uint16_t speedupbuf[10240];
	uint16_t slowdownbuf[10240];
	uint32_t speedupCNT;
	uint32_t runCNT;
	uint32_t slowdownCNT;
	int16_t cnt;
	uint8_t manual_mode;
	uint8_t start_pose_mode;
	uint8_t manual_move_right;
	uint8_t manual_move_left;
	TIM_HandleTypeDef* engine_TIM_master; //TIM3
	TIM_HandleTypeDef* engine_TIM_slave; //TIM2
} t_step_engine;

void init_step_engine(t_step_engine* step_eng);
void led_init(); /* LED-pin init */

void move_step_engine(t_step_engine* step_eng, int16_t pos, float vel);

int16_t calc_steps(int16_t pos);

inline float mysqrt(float value);


#endif //!STEP_ENGINE_H