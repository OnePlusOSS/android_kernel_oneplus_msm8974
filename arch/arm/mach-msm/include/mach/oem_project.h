/* 
 *
 * yixue.ge add for oppo project
 *
 *
 */
#ifndef _OPPO_PROJECT_H_
#define _OPPO_PROJECT_H_

enum{
	HW_VERSION__UNKNOWN,
	HW_VERSION__10, 	//1452mV
	HW_VERSION__11, 	//1636 mV
	HW_VERSION__12, 	//1224 mV
	HW_VERSION__13, 	//900 mV
	HW_VERSION__14, 	//720 mV
	HW_VERSION__15, 	
	HW_VERSION__16, 
	HW_VERSION__17,
};

enum{
	RF_VERSION__UNKNOWN,
	RF_VERSION__11,		
	RF_VERSION__12,		
	RF_VERSION__13,
	RF_VERSION__21,		
	RF_VERSION__22,		
	RF_VERSION__23,
	RF_VERSION__31,		
	RF_VERSION__32,		
	RF_VERSION__33,
};


#define GET_PCB_VERSION() (get_PCB_Version())
#define GET_PCB_VERSION_STRING() (get_PCB_Version_String())

#define GET_MODEM_VERSION() (get_Modem_Version())
#define GET_OPERATOR_VERSION() (get_Operator_Version())



enum OPPO_PROJECT {
	OPPO_UNKOWN = 0,
	OPPO_13095 = 13095,
	OPPO_14013 = 14013,
	OPPO_14017 = 14017,
	OPPO_14027 = 14027,
	OPPO_14029 = 14029,
	OPPO_14033 = 14033,
	OPPO_14035 = 14035,
	OPPO_14016 = 14016,
	OPPO_14018 = 14018,	
	OPPO_14031 = 14031,		
	OPPO_13084 = 13084,
	OPPO_14005 = 14005,
	OPPO_14023 = 14023,
	OPPO_14045 = 14045,
	OPPO_14046 = 14046,
	OPPO_14047 = 14047,
	OPPO_14006 = 14006,
	OPPO_14043 = 14043,
	OPPO_14042 = 14042,
	OPPO_14041 = 14041,
	OPPO_14037 = 14037,
	OPPO_14039 = 14039,
	OPPO_14040 = 14040,
	OPPO_14051 = 14051,
	OPPO_15005 = 15005,
	OPPO_15011 = 15011,
	OPPO_15057 = 15057,
    OPPO_15018 = 15018,
	OPPO_15025 = 15025,
};

enum OPPO_OPERATOR {
	OPERATOR_UNKOWN 			= 0,
	OPERATOR_OPEN_MARKET 		= 1,
	OPERATOR_CHINA_MOBILE 		= 2,
	OPERATOR_CHINA_UNICOM 		= 3,
	OPERATOR_CHINA_TELECOM 		= 4,
	OPERATOR_FOREIGN 			= 5,
	OPERATOR_FOREIGN_WCDMA 		= 6,
};

typedef enum OPPO_PROJECT OPPO_PROJECT;

typedef struct
{
  unsigned int                  nProject;
  unsigned char                 nModem;
  unsigned char                 nOperator;
  unsigned char                 nPCBVersion;
} ProjectInfoCDTType;

#ifdef CONFIG_OPPO_COMMON_SOFT
unsigned int init_project_version(void);
unsigned int get_project(void);
unsigned int is_project(OPPO_PROJECT project );
unsigned char get_PCB_Version(void);
unsigned char get_Modem_Version(void);
unsigned char get_Operator_Version(void);
#else
unsigned int init_project_version(void) { return 0;}
unsigned int get_project(void) { return 0;}
unsigned int is_project(OPPO_PROJECT project ) { 
	if(project ==15011)
		return true;
	else
		return 0;
}
unsigned char get_PCB_Version(void) { return 0;}
unsigned char get_Modem_Version(void) { return 0;}
unsigned char get_Operator_Version(void) { return 0;}
#endif
#endif