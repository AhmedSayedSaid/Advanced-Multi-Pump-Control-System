/*
 * VFD.h
 *
 *  Created on: Aug 14, 2023
 *      Author: ahmed Sayed
 *      info :contain the addresses of the various vfd that is comptable with the system
 */

#ifndef INC_VFD_H_
#define INC_VFD_H_


/************************start danfoss 101 addresses************************************/

typedef enum{

	danfoss101Voltage_V_Read=16119,
	danfoss101Speed_RPM_Read=16169,
	danfoss101Current_MA_Read=16139,
	danfoss101Freqency_HZ_Read=16129,
	danfoss101ControlWord=49999,
	danfoss101Speed=50009,
}danfossAddress;






typedef enum{
	danfoss101_stop=1084,
	danfoss101_runForward=1148,
	danfoss101_runReverse=33916,
	danfoss101_VFDReset=1212,

}danfossControlWord;

/************************End danfoss 101 addresses************************************/



#endif /* INC_VFD_H_ */
