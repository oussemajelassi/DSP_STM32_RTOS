/*
 * DoubleBuffering.cpp
 *
 *  Created on: Nov 6, 2023
 *      Author: Flooki
 */



#include "DoubleBuffering.h"

extern TaskHandle_t DSP_TaskHandle ;
extern arm_rfft_fast_instance_f32 DSP_FFT_Handle ;

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
DSP_SignalSpecs DoubleBuffer::Compute_DCOffset( void )
{
	float32_t FFT_InputBuffer [ this->Size ] ;
	float32_t FFT_OutputBuffer[ this->Size ] ;
	float32_t PeakHz = 0 ;
	DSP_SignalSpecs DSP_CurrentSignalSpecs ;
	float32_t PeakVal = 0 ;
	for ( uint32_t cnt = 0 ; cnt < this->Size ; cnt ++  )
	{
		FFT_InputBuffer [ cnt ] = this->CurrentBuffer.at(cnt);
	}
	arm_rfft_fast_f32(&DSP_FFT_Handle, FFT_InputBuffer, FFT_OutputBuffer, 0 ) ;

	for ( uint32_t freqIndex = 0 ; freqIndex < ( this->Size / 2 ) ; freqIndex ++ )
	{
		PeakVal = \
		sqrtf ( FFT_OutputBuffer [ 2 * freqIndex ] * FFT_OutputBuffer [ 2 * freqIndex ] \
		+ FFT_OutputBuffer [ 2 * freqIndex + 1 ] * FFT_OutputBuffer [ 2 * freqIndex + 1 ] ) ;
		if  ( PeakVal > PeakHz )
		{
			PeakHz = PeakVal ;
		}
	}
	DSP_CurrentSignalSpecs.DC_Offset = FFT_OutputBuffer [0] ;
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
