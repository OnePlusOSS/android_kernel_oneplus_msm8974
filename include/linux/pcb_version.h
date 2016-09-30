/************************************************************ 
** Copyright (C), 2008-2012
** VENDOR_EDIT
** File: - pcb_version.h
* Description: head file for pcb_version.
				
** Version: 1.0
** Date : 2013/10/15	
** Author: yuyi@Dep.Group.Module
************************************************************/
#ifndef _PCB_VERSION_H
#define _PCB_VERSION_H

enum {
	PCB_VERSION_UNKNOWN,
	HW_VERSION__10,		//729mV	
	HW_VERSION__11,		//900 mV	
	HW_VERSION__12,		//1200 mV	
	HW_VERSION__13,		//1484 mV

	HW_VERSION__20 = 20,		
	HW_VERSION__21 = 21,		
	HW_VERSION__22,		
	HW_VERSION__23,	
/* wenxian.zhen@Onlinerd.Driver, 2014/06/18  Add begin for N3  PCB	version */
	HW_VERSION__30 = 30,
	HW_VERSION__31 = 31,
	HW_VERSION__32,
	HW_VERSION__33,
	HW_VERSION__34,

	HW_VERSION__40 = 40,
	HW_VERSION__41 = 41,
	HW_VERSION__42,
	HW_VERSION__43,
	HW_VERSION__44,
/* wenxian.zhen@Onlinerd.Driver, 2014/06/18  Add end for N3  PCB	version */

	
};
enum {
	RF_VERSION_UNKNOWN,
	RF_VERSION__11,		//WCDMA_GSM_China	
	RF_VERSION__12,		//WCDMA_GSM_LTE_Europe	
	RF_VERSION__13,		//WCDMA_GSM_LTE_America
	RF_VERSION__21,		//WCDMA_GSM_CDMA_China	
	RF_VERSION__22,		//WCDMA_GSM_Europe	
	RF_VERSION__23,		//WCDMA_GSM_America
	RF_VERSION__31,		//TD_GSM	
	RF_VERSION__32,		//TD_GSM_LTE	
	RF_VERSION__33,		//
	RF_VERSION__44 = 19,		//
	RF_VERSION__66 = 30,	//add for 14001 TDD-LTE+FDD-LTE+TDS+W+G
	RF_VERSION__67,
	RF_VERSION__76,
	RF_VERSION__77,
	RF_VERSION__87,
	RF_VERSION__88,
	RF_VERSION__89,
	RF_VERSION__98,
	RF_VERSION__99,
/* wenxian.zhen@Onlinerd.Driver, 2014/06/18  Add begin for N3  PCB RF version */
	RF_VERSION__90_CHINA_MOBILE= 90,
	RF_VERSION__91_UNICOM,
	RF_VERSION__92_CHINA_RESERVED1,
	RF_VERSION__93_CHINA_RESERVED2,
	RF_VERSION__94_CHINA_RESERVED3,
	RF_VERSION__95_EUROPE,
	RF_VERSION__96_AMERICA,
	RF_VERSION__97_TAIWAN,
	RF_VERSION__98_INDONESIA,
	RF_VERSION__99_OVERSEA_RESERVED1,
/* wenxian.zhen@Onlinerd.Driver, 2014/06/18  Add end for N3  PCB RF version */

};

struct ddr_info{
	char ddr_manufacture[64];
	char ddr_row_info[8];
};
extern int get_pcb_version(void);
extern int get_rf_version(void);
extern int get_ddr_info(struct ddr_info *str);

#endif /* _PCB_VERSION_H */


