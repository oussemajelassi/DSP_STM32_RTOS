/*
 * DoubleBuffering.cpp
 *
 *  Created on: Nov 6, 2023
 *      Author: Flooki
 */



#include "DoubleBuffering.h"
extern TaskHandle_t DSP_TaskHandle ;
extern arm_rfft_fast_instance_f32 DSP_FFT_Handle ;
extern ADC_HandleTypeDef hadc3;
extern float32_t FFT_OutputBuffer[ DSP_FFT_SIZE ] ;
extern float32_t FFT_Magnitude [ DSP_FFT_SIZE ] ;
DoubleBuffer::DoubleBuffer ( uint32_t Size , uint32_t SamplingTime )
{
	this->NextBuffer.resize(0) ;
	this->CurrentBuffer.resize(0);
	this->FreeSpace = Size ;
	this->Size = Size ;
	this->SamplingTime = SamplingTime ;
}

DoubleBuffer::~DoubleBuffer ( void )
{
	this->NextBuffer.clear() ;
	this->CurrentBuffer.clear() ;
}
/*
 * Brief : Swap Function Will not Alert The Other Task
 * Maybe I will be adding mechanism to make a choice.
 */
void DoubleBuffer::Swap ( void )
{
	this->CurrentBuffer.swap(this->NextBuffer) ;
	this->NextBuffer.clear() ;
	this->CurrentBuffer.resize(this->Size) ;
	this->FreeSpace = this->Size ;
}

void DoubleBuffer::NotifyComputaionTask ( void )
{
	BaseType_t pxHigherPriorityTaskWoken ;
	xTaskNotifyFromISR(DSP_TaskHandle , 1 , eSetValueWithOverwrite , & pxHigherPriorityTaskWoken ) ;
}

double DoubleBuffer::ComputeAvg( void )
{
	double Avg = 0 ;

	for ( uint32_t cnt = 0 ; cnt < this->Size ; cnt ++  )
	{
		Avg += ( ( (double) this->CurrentBuffer[cnt] ) / (double) this->Size ) ;
	}
	this->CurrentBufferReady = false ;

	return ( Avg ) ;
}

/*
 * Brief : This Function uses CMSIS DSP to Compute FFT and then Takes DC Offset out of it
 */
DSP_SignalSpecs DoubleBuffer::Compute_SignalSpecs( void )
{
	float32_t PeakHz = 0 ;
	uint32_t Index = 0 ;
	DSP_SignalSpecs DSP_CurrentSignalSpecs ;

	arm_rfft_fast_f32(&DSP_FFT_Handle, this->CurrentBuffer.data(), FFT_OutputBuffer , 0 ) ;
	arm_cmplx_mag_f32(FFT_OutputBuffer , FFT_Magnitude , DSP_FFT_SIZE / 2);
	DSP_CurrentSignalSpecs.DC_Offset = FFT_Magnitude [0] ;
	FFT_Magnitude [0] = 0 ;
	arm_max_f32(FFT_Magnitude, DSP_FFT_SIZE / 2, &PeakHz, &Index);
	PeakHz = ( DSP_SAMPLING_TIME * Index ) /  DSP_FFT_SIZE ;
	DSP_CurrentSignalSpecs.Fundamental_Freq = PeakHz ;

	return ( DSP_CurrentSignalSpecs ) ;
}

/*
 * Brief : Insert data mechanism is also where we swap the two buffers in advance
 */

void DoubleBuffer::InsertData( float32_t Data )
{
	if  ( this->FreeSpace )
	{
		this->NextBuffer.push_back(Data) ;
		this->FreeSpace -- ;
		if ( this->FreeSpace == 0 )
		{
			this->Swap() ;
			this->CurrentBufferReady = true ;
		}
	}
}

bool DoubleBuffer::GetCurrentBufferState( void )
{
	bool tmp = false ;
	if ( this->CurrentBufferReady ) { tmp = true ; }
	else { tmp = false ; }
	return ( tmp ) ;
}

void DoubleBuffer::SetCurrentBufferState ( bool State )
{
	this->CurrentBufferReady = State ;
}



