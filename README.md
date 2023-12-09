![IMG_20231129_190248](https://github.com/oussemajelassi/DSP_STM32_RTOS/assets/100140668/047ff349-5d89-437b-a1df-b4c5d1f41f53)# DSP_STM32_RTOS
This Repo will follow me trying to get a mix up between Real Time techniques and some DSP on the STM32H745ZI ..

**References** :
* [RFFT IN CMSIS](https://elastic-notes.blogspot.com/2019/04/rfft-in-cmsis-dsp-part-1_22.html)
  


## CMSIS_DSP : 

CMSIS are offering a set of DSP funcitons that makes easier implementing DSP algorithms using C language on MCUs,
For instance I will be using them on my H745 running on the CM7 Arm Cortex.
Refer to  [Integrate DSP on CubeIDE](https://community.st.com/t5/stm32-mcus/configuring-dsp-libraries-on-stm32cubeide/ta-p/49637)

## Real Fast Fourier Transform :

RFFTs are widely used in modern DSPs application they are a variant of the well known Fast fourier transform and aims as well to get a signal from time domain to frequency domain.

![ARM RFFT_f32](https://github.com/oussemajelassi/DSP_STM32_RTOS/assets/100140668/fe86019f-3365-4710-a9b9-3d12980fd629)

Using provided Function will take us into an output buffer having both real and imaginary parts.
Every Frequency "Bin" is represented by an amplitude and phase which are the complex number generated by our function.
We should then extract the magnitude of every frequency bin.

First Results are shown below , for a static Sine Wave that i ganerated with C Code ONE Frequenc **Bin** is Shown is CubeMonitor.

![RFFT Running on the STM32](https://github.com/oussemajelassi/DSP_STM32_RTOS/assets/100140668/8483ebe9-b994-4fdb-a531-df69acfb0968)

As A first Application that combines FreeRTOS and DSP I will implement this : 
![image](https://github.com/oussemajelassi/DSP_STM32_RTOS/assets/100140668/fcd71c47-dbad-4073-900a-6dd00fdcd4df)

### Double Buffer : Local Buffer

 #### CPP + CMSIS_DSP troubleshooting : 

I faced a compiling progam when I wanted to use CMSIS_DSP inside a C++ Source Code, 

I followed this github issue to solve the problem :  [CMSIS_DSP C++](https://github.com/ARM-software/CMSIS_5/issues/617) 

There Will be a slight change in the architecture the ADC Task wont be the one directly accessing ADC Instead it will only be filling the Double Buffer since It cannot reach the sampling time with ordianry tasks.
So, I will Use a [Timer Triggered ADC](https://community.st.com/t5/stm32-mcus/using-timers-to-trigger-adc-conversions-periodically/ta-p/49889)., Also this [ST Video](https://www.youtube.com/watch?v=Yt5cHkmtqlA) is very helpful.

### Signal Generation : 

After Computing Signal Specs with Corressponging CMSIS Funcions, We managed to queue Final results into another Task, This Task will generate a Sine Whos Frequency is the sama as INPUT Signal.
This Task will be functionning based on a finite state machine.

![signal jpg](https://github.com/oussemajelassi/DSP_STM32_RTOS/assets/100140668/2ccb5fe7-d5b0-4623-a5c8-4e16b8b912ed)
 
 So INPUT Signal will be a pwm which frequency is 1000HZ : 
 
![IMG_20231129_190248](https://github.com/oussemajelassi/DSP_STM32_RTOS/assets/100140668/1e0e8457-92c8-4cec-9087-fd26e41a7136)

 After All computation and RealTime Pipeline results are there : 
 
 ![IMG_20231205_185031](https://github.com/oussemajelassi/DSP_STM32_RTOS/assets/100140668/077015f9-a41c-409d-95ba-0d85db5791e0)



### NEXT : 
#### Using the DAC to implement an audio player
**Description** : 
The purpose of this demonstration is to provide an audio player solution based on STM32
products to play .WAV files. The approach is optimized to use a minimum number of
external components, and offers to end-users the possibility of using their own .WAV files.
The audio files are stored in a microSD memory card, accessible by the STM32 through the
SPI bus

