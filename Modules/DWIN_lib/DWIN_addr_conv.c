#include "DWIN_addr_conv.h"

t_addr_conv PLC_addr[PLC_ADDR_MAX] = {

			{  STEP_ENGINE_ON_MC, 			 		0x5002	},
			{  STEP_ENGINE_VEL_MC,  				0x5010	},
			{  STEP_ENGINE_POS_MC, 					0x5012	},
			{  STEP_ENGINE_START_POS_MS,  	0x5000	},
			{  STEP_ENGINE_MOVE_RIGHT,  		0x5056	},
			{  NUM_EXE_PROGRAM,  						0x5014	},
			{  LAUNCH_PROGRAM,  						0x5050	},
			{  SAVE_PROGRAM,  							0x5052	},
			{  STAGE_1_POS,  								0x5032	},
			{  STAGE_1_VEL, 								0x5034	},
			{  STAGE_1_DEL,  								0x5036	},
			{  STAGE_2_POS,  								0x5038	},
			{  STAGE_2_VEL,  								0x5040	},
			{  STAGE_2_DEL,  								0x5042	},
			{  STAGE_3_POS,  								0x5044	},
			{  STAGE_3_VEL,  								0x5046	},
			{  STAGE_3_DEL,  								0x5048	},
			{  TRANSFUSE_ANGLE,  						0x5020	},
			{  MIXING_TIME,  								0x5022	},
			{  VACUUM_TIME,  								0x5024	},
			{  STEP_ENGINE_MOVE_LEFT,  			0x5054	},
			//{  START_POS_LOCALITY,  				0x5054	},
};

USHORT conv_addr(t_addr_conv* addr_conv, USHORT hmi_addr) {
	
	uint8_t i = 0;
	uint8_t is_find = 0;
	
	while (!is_find) {
		if (addr_conv[i++].HMI_addr == hmi_addr){
			is_find = 1;
		}
	}
	
	return addr_conv[i - 1].PLC_addr; 
}