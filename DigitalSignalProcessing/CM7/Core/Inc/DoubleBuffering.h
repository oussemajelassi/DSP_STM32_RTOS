/*
 * DoubleBuffering.h
 *
 *  Created on: Nov 6, 2023
 *      Author: Flooki
 */

#ifndef INC_DOUBLEBUFFERING_H_
#define INC_DOUBLEBUFFERING_H_


#define ARM_MATH_CM7
#define DSP_FFT_SIZE 	   1024
#define DSP_SAMPLING_TIME  4000

#include <vector>
#include "stdint.h"
#include"FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "main.h"


/*  Function Prototypes Begin  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) ;
/*  Function Prototypes End  */
typedef struct
{
	float32_t DC_Offset ;
	float32_t Fundamental_Freq ;
}DSP_SignalSpecs;

class DoubleBuffer
{
private :
	uint32_t FreeSpace ;
	uint32_t Size ;
	bool CurrentBufferReady = false ;
	std::vector <float32_t> CurrentBuffer ;
	std::vector <float32_t> NextBuffer  ;
	uint32_t SamplingTime ;
public :
	DoubleBuffer ( uint32_t Size , uint32_t SamplingTime ) ;
	~DoubleBuffer ( void ) ;
	void Swap() ;
	void NotifyComputaionTask ( void ) ;
	double ComputeAvg ( void )  ;
	void InsertData ( float32_t Data ) ;
	bool GetCurrentBufferState ( void ) ;
	void SetCurrentBufferState ( bool State ) ;
	DSP_SignalSpecs Compute_SignalSpecs ( void ) ;
};

#endif /* INC_DOUBLEBUFFERING_H_ */
