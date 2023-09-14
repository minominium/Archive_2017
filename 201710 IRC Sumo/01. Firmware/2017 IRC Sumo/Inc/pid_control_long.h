/*==============================================================================
 *
 *   pid_control_long.h
 *
 *   	File Name   	: pid_control_long.h
 *  	Version        	: 1.0
 *    	Date           	: Apr 28, 2015
 *		Modified       	: Apr 28, 2015 By ParkSuhan
 *		Author         	: ParkSuhan  (ROBIT 8th, Kwangwoon University)
 *		MPU_Type       	: MC56F8257 (60MHz)
 *		Compiler		: Freescale CodeWarrior 10.6
 *		Copyright(c) 2015 ROBIT, Kwangwoon University.
 *    	All Rights Reserved.
 *
==============================================================================*/

/*!
** @file pid_control_long.h
** @version 1.01
** @brief
**         Long 데이터 형식 지원 PID제어기 헤더 \n
**         이 모듈은 Long만을 사용하는 PID제어기의 함수와 구조체가 들어있습니다. 
*/         

#ifndef PID_CONTROL_LONG_H_
#define PID_CONTROL_LONG_H_

 

/* Part of PID Struct */

/** Long 지원 PID 제어기 데이터 구조체 */
typedef struct _LONGPID{
	long nowValue;		//!< 현재 값
	long pastValue;		//!< 이전 값
	
	long nowError;		//!< 현재 에러값
	long pastError;		//!< 이전 에러값
	long target;		//!< 목표값
	
	long errorSum;		//!< 에러 누적값
	long errorSumLimit;	//!< 에러 누적값 제한 (0일 경우 제한 없음)
	long errorDiff;		//!< 에러 미분값

	long nowOutput;	//!< 현재 출력값
	long pastOutput;	//!< 이전 출력값
	long outputLimit;	//!< 출력 제한값
	
	long underOfPoint;	//!< kP, kI, kD에 공통으로 들어가는 나눗셈 (underOfPoint=1000, kP=1이면 P이득값 = 0.001)
	
	long kP;			//!< P(비례)이득값
	long kI;			//!< I(적분)이득값
	long kD;			//!< D(미분)이득값
}LPID;


/* Part of PID Function */

/** Long 이득값을 사용하는 PID 제어기 함수 */
void PID_Control_Long(LPID* dst, 	//!< 제어 대상 PID 데이터 구조체
		long target, 				//!< 목표 값
		long input					//!< 현재 값
		);

/** Long 이득값을 사용하는 PID 구조체 초기화 함수 */
void PID_Control_Long_Initialize(LPID* dst //!< 제어 대상 PID 데이터 구조체
		);


#endif /* PID_CONTROL_LONG_H_ */
