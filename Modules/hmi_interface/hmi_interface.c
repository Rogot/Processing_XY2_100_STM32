#include "hmi_interface.h"
#include <stdlib.h>

t_hmi_reg programs[MAX_PRGRMS_NUM];
t_control ctrl;
extern t_step_engine step_engine;
extern uint8_t is_start_pos;
t_hmi_reg* current_program;

/*
* @bref: write prigram from FLASH
* @param prog - programm parameters 
* @param num - number of saved programs
*/

uint8_t write_program(t_hmi_reg* prog, uint16_t num){

	uint8_t struct_size = sizeof(t_hmi_reg);
	
	uint8_t* data_ptr = malloc(num);
	
	CMSIS_flash_allow_access();
	CMSIS_internal_flash_erase(FLASH_SNB_SEC_3);
	
	data_ptr = (uint8_t*)&(*prog);
	CMSIS_internal_flash_write(data_ptr, BASE_PROGRAM_ADDRESS, num * sizeof(t_hmi_reg));
	
	//t_hmi_reg* temp[2];
	//for (uint8_t i = 0; i < size; i++) {
	//	temp[i] = (t_hmi_reg*)&(*(data_ptr[i])); 
	//}
	//free(temp);
	free(data_ptr);
	
	return 1;
}

/*
* @bref: read prigram from FLASH
* @param program - programm parameters;
* @param num - number of the program read
*/

void read_program(t_hmi_reg* program, uint8_t num) {
	t_hmi_reg* temp_program;
	uint32_t* temp = malloc(sizeof(t_hmi_reg));
	
	for (uint16_t i = 0; i < sizeof(t_hmi_reg) / sizeof(uint32_t); i++){
		temp[i] = CMSIS_internal_flash_read(BASE_PROGRAM_ADDRESS
											+ (sizeof(t_hmi_reg) * num) + sizeof(uint32_t) * i);
	}
	
	temp_program = (t_hmi_reg*)&(*(temp));
	
	memcpy(&program[num], temp_program, (sizeof(t_hmi_reg)));
}

/*
* @bref: execute storage program
* @param (comtrl) - all parameters for controlling by HMI
*/
void execute_program(t_control* comtrl) {
	
	#if STEP_ENGINE_ENABLE
		#if STEP_ENGINE_TEST_ENABLE
			TIM2->CCR1=TIM2->CNT+step_engine.dir*step_engine.speedupCNT;
			HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], 
				(uint32_t)step_engine.speedupbuf, 
				(uint32_t)&TIM3->ARR, 
				step_engine.speedupCNT);
		  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
			step_engine.mode=SPEEDUP;
			HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
		#endif
			
		#if !STEP_ENGINE_TEST_ENABLE
			//read_program(comtrl->programms, comtrl->exe_prog);
			for (uint8_t i = 0; i < STAGE_NUM; i++) {
				if (comtrl->programms[comtrl->exe_prog].vel[i] != 0x00) {
					if (comtrl->programms[comtrl->exe_prog].vel[i] > MAX_VEL_PROG) {
						comtrl->programms[comtrl->exe_prog].vel[i] = MAX_VEL_PROG;
					}
					move_step_engine(comtrl->dev->step_engine, 
						calc_steps(comtrl->programms[comtrl->exe_prog].moving[i]), (float)(
						(float)comtrl->programms[comtrl->exe_prog].vel[i] / BASE_FREQ / ANFLE_ONE_STEP * 2 ));
				//while(comtrl->dev->step_engine->mode != STOP);
				}
			}				
		#endif
	#endif
}

/*
* @bref: save struct data in FLASH
*	@param (comtrl) - all parameters for controlling by HMI
*/

void refresh_prog_parameters_FLASH(t_control* comtrl) {
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		write_program(comtrl->programms, i);
	}
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		read_program(comtrl->programms, i);
	} 
}

/*
* @bref: only load program from FLASH without write
*	@param (comtrl) - all parameters for controlling by HMI
*/

void load_prog_FLASH(t_control* comtrl) {
	for (uint8_t i = 0; i < MAX_PRGRMS_NUM; i++) {
		read_program(comtrl->programms, i);
	}
}

/*
* @bref: manual mode control function
*	@param (comtrl) - all parameters for controlling by HMI
*/
void munual_mode(t_control* comtrl) {
		comtrl->dev->step_engine->manual_mode = 0x01;
	if (comtrl->current_vel != 0x00) {
		if (comtrl->dev->step_engine->manual_move_left) {
			move_step_engine(comtrl->dev->step_engine, 2, (float)(
										(float)comtrl->current_vel / (float)2550));
		}
		else if (comtrl->dev->step_engine->manual_move_right) {
			move_step_engine(comtrl->dev->step_engine, -2, (float)(
										(float)comtrl->current_vel / (float)2550));
		}
	}
}

/*
* @bref: "home" position of coordinate
*	@param (comtrl) - all parameters for controlling by HMI
*/
void move_start_pos(t_control* comtrl) {
	int32_t start_pos = 2147483647;
	int32_t cur_pos = TIM2->CCR1;
	
	int32_t res = (int16_t)start_pos - (int16_t )cur_pos;
	
	if ((res > START_POS_LOCALITY || res < -START_POS_LOCALITY)
				&& comtrl->current_vel != 0x0) {
		move_step_engine(comtrl->dev->step_engine, (int16_t )(res), (float)(
							(float)comtrl->current_vel / (float)2550));
	}
}

/*
* @bref: "home" position of coordinate
*	@param (comtrl) - all parameters for controlling by HMI
*	@param (usRegBuf) - MODBUS array
*/
void eHMIPoll(t_control* comtrl, int* usRegBuf) {
	
	refresh_reg(comtrl, usRegBuf);
	
	#if STEP_ENGINE_ENABLE
	if( comtrl->is_manual == 0x01 && comtrl->is_launch == 0x00
				&& comtrl->dev->step_engine->mode == STOP){
		comtrl->is_manual = 0x00;
		comtrl->dev->step_engine->start_pose_mode = 0x00;			
		munual_mode(comtrl);		
	}
	else if(comtrl->is_manual == 0x00 && comtrl->is_launch == 0x01 
			&& comtrl->dev->step_engine->mode == STOP){
		execute_program(comtrl);
		comtrl->is_launch = 0x00; 
		usRegBuf[LAUNCH_PROGRAM] = 0x0;
	}
	
	if (comtrl->start_pos_step_engine == 0x01 
			&& comtrl->dev->step_engine->mode == STOP) {
		comtrl->dev->step_engine->start_pose_mode = 0x01;	
		move_start_pos(comtrl);
		usRegBuf[STEP_ENGINE_START_POS_MS] = 0x0;	
	}
	#endif
			
	#if FLASH_ENABLE
	if (comtrl->save_prog != 0) {
		usRegBuf[SAVE_PROGRAM] = 0x0;
		refresh_prog_parameters_FLASH(comtrl);
	}
	#endif
}

/*
* @bref: function for refresh struct parameters for HMI
* @param comtrl - all parameters for controlling by HMI
* @param usRegBuf - MODBUS buffer pointer
*/
void refresh_reg(t_control* comtrl, int* usRegBuf) {
	
	if (comtrl->dev->step_engine->mode != STOP){
		usRegBuf[LAUNCH_PROGRAM] = 0x0;
	}
	
	comtrl->save_prog = usRegBuf[SAVE_PROGRAM];
	comtrl->start_pos_step_engine = usRegBuf[STEP_ENGINE_START_POS_MS];
	
	comtrl->is_launch = usRegBuf[LAUNCH_PROGRAM];
	comtrl->is_manual = usRegBuf[STEP_ENGINE_ON_MC];
	
	comtrl->current_vel = usRegBuf[STEP_ENGINE_VEL_MC];
	comtrl->current_pos = usRegBuf[STEP_ENGINE_POS_MC];
	
	comtrl->programms[comtrl->exe_prog].vel[0] = usRegBuf[STAGE_1_VEL];
	comtrl->programms[comtrl->exe_prog].vel[1] = usRegBuf[STAGE_2_VEL];
	comtrl->programms[comtrl->exe_prog].vel[2] = usRegBuf[STAGE_3_VEL];

	comtrl->programms[comtrl->exe_prog].moving[0] = (int16_t)usRegBuf[STAGE_1_POS];
	comtrl->programms[comtrl->exe_prog].moving[1] = (int16_t)usRegBuf[STAGE_2_POS];
	comtrl->programms[comtrl->exe_prog].moving[2] = (int16_t)usRegBuf[STAGE_3_POS];
	
	comtrl->exe_prog = usRegBuf[NUM_EXE_PROGRAM] - 1;
	
	comtrl->dev->step_engine->manual_move_left = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_LEFT];
	comtrl->dev->step_engine->manual_move_right = (uint8_t)usRegBuf[STEP_ENGINE_MOVE_RIGHT];
	
}

/*
* @bref: inital function for HMI
* @param comtrl - all parameters for controlling by HMI
*/
void init_HMI(t_control* comtrl) {
/*
	comtrl->programms[0].mixing_speed = 0x33;
	comtrl->programms[0].mixing_time= 0x34;
	comtrl->programms[0].vacuum_time = 0x35;
	comtrl->programms[0].air_pubping_lvl = 0x36;
	comtrl->programms[0].moving[0] = 100;
	comtrl->programms[0].vel[0] = 20;
	comtrl->programms[0].moving[1] = 200;
	comtrl->programms[0].vel[1] = 100;
	comtrl->programms[0].moving[2] = -100;
	comtrl->programms[0].vel[2] = 20;
	
	comtrl->programms[1].mixing_speed = 0x23;
	comtrl->programms[1].mixing_time= 0x24;
	comtrl->programms[1].vacuum_time = 0x25;
	comtrl->programms[1].air_pubping_lvl = 0x26;
	*/
	
	read_program(current_program, 0);
}


/*
* @bref: search start position - "0" via end cap
* @param comtrl - all parameters for controlling by HMI
*/
void search_home(t_control* comtrl) {
	while(!is_start_pos) {
		move_step_engine(comtrl->dev->step_engine, 
					2, (float)((float)50 / (float)2550));
	}
	step_engine.engine_TIM_slave->Instance->CCR1 = step_engine.engine_TIM_slave->Instance->CNT 
								+ step_engine.dir * step_engine.slowdownCNT;
	step_engine.mode = SLOWDOWN;
				
	HAL_DMA_Start_IT(step_engine.engine_TIM_master->hdma[TIM_DMA_ID_UPDATE], 
				(uint32_t)(step_engine.slowdownbuf 
								+ step_engine.accel_size - step_engine.slowdownCNT),
				(uint32_t)&step_engine.engine_TIM_master->Instance->ARR, step_engine.slowdownCNT);
				
	__HAL_TIM_ENABLE_DMA(step_engine.engine_TIM_master, TIM_DMA_UPDATE);
	
}