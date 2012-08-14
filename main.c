//////////////////////////////////////////////////////////////////////////////
// * File name: main.c
// *              
//
// * fft of audio from codec, modified by Brock Anderson 2012
//
//                                                            
// * Description: This file includes main() and system initialization funcitons.
// *                                                                          
// * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
// *                                                                          
// *                                                                          
// *  Redistribution and use in source and binary forms, with or without      
// *  modification, are permitted provided that the following conditions      
// *  are met:                                                                
// *                                                                          
// *    Redistributions of source code must retain the above copyright        
// *    notice, this list of conditions and the following disclaimer.         
// *                                                                          
// *    Redistributions in binary form must reproduce the above copyright     
// *    notice, this list of conditions and the following disclaimer in the   
// *    documentation and/or other materials provided with the                
// *    distribution.                                                         
// *                                                                          
// *    Neither the name of Texas Instruments Incorporated nor the names of   
// *    its contributors may be used to endorse or promote products derived   
// *    from this software without specific prior written permission.         
// *                                                                          
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   
// *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    
// *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   
// *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        
// *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   
// *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   
// *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     
// *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   
// *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    
// *                                                                          
//////////////////////////////////////////////////////////////////////////////

#include "csl_pll.h"
#include "cslr_sysctrl.h"
#include "cslr_cpu.h"
#include "csl_intc.h"
#include "stdio.h"
#include "hwafft.h"
#include "register_system.h"
#include "register_cpu.h"
#include "register_dma.h"
#include "i2s_bypass1.h"

#define SAMPLE_RATE 44100
#define SAMPLES_PER_UNIT int(0.01f * float(SAMPLE_RATE))
#define BPS_CUTOFF 3
#define BEAT_WINDOW_UNITS 1024 
#define RESOLUTION 4.0f // seconds, this is the longest time it can take for one beat

#define SARCTRL *(ioport volatile unsigned *)0x7012
#define SARDATA *(ioport volatile unsigned *)0x7014
#define SARCLKCTRL *(ioport volatile unsigned *)0x7016
#define SARPINCTRL *(ioport volatile unsigned *)0x7018
#define SARGPOCTRL *(ioport volatile unsigned *)0x701a

#define I2S0_CR_LSW       *(ioport volatile unsigned *)0x2800

#define FFT_FLAG ( 0 ) /* HWAFFT to perform FFT */
#define IFFT_FLAG ( 1 ) /* HWAFFT to perform IFFT */
#define SCALE_FLAG ( 0 ) /* HWAFFT to scale butterfly output */
#define NOSCALE_FLAG ( 1 ) /* HWAFFT not to scale butterfly output */
#define OUT_SEL_DATA ( 0 ) /* Indicates HWAFFT output located in input data vector */
#define OUT_SEL_SCRATCH ( 1 ) /* Indicates HWAFFT output located in scratch vector */

Int32 data_data[1024];
Int32 sound_bytes[16];
Int32 sound_data_a[1024];
Int32 sound_data_b[1024];

unsigned int sub_sample = 0;
unsigned int unit_sample = 0;

unsigned int adc2, adc3;

typedef enum { BANK_A, BANK_B } sound_bank_t;

volatile int led_enabled = 0;
volatile static int global_frame_num;
volatile static int global_beat_offset, global_beat_period;
volatile static int sound_data_ready = 0;
volatile static sound_bank_t sound_bank = BANK_A;

#pragma DATA_SECTION(data_br_data, "data_br_data");
#pragma DATA_ALIGN (data_br_data, 2048);
Int32 data_br_data[1024];  // need to align this!

Int32 scratch_data[1024];
Int32 result_data[1024];

void corellation_mag_1024(Int32 * in, Uint32 * result_mag);
void fft_mag_1024( Int32 * input, Uint32 * result_mag);

void turnOnLED(void);
void turnOffLED(void);

void SYS_GlobalIntEnable(void)
{
	asm(" MOV #0x0059, mmap(@IVPD)");
	asm(" MOV #0x0059, mmap(@IVPH)");
	asm(" MOV #0xfffd, mmap(@IFR0)");
	asm(" MOV #0x7fff, mmap(@IFR1)");
	DMA_IFR = 0xFFFF;
	ST1_55 &= ~CSL_CPU_ST1_55_INTM_MASK;
	asm(" BCLR INTM"); // Enable interrupts
}

void SYS_GlobalIntDisable(void)
{
	asm(" BSET INTM"); // Mask all
}

static unsigned int DMA_z = 0;

interrupt void Default_Isr(void)
{
	int z3;
	z3 = 3;
	z3++;
	return;
}

interrupt void Bus_Isr(void)
{
	int z2;
	z2 = 3;
	z2++;
	return;
}

interrupt void DMA_Isr(void)
{
	static Uint32 square_sum = 0;
	
	// this is the indicator of the energy sample we have collected
	sub_sample++;
	
	
	
	square_sum += (sound_bytes[0] >> 20) * (sound_bytes[0] >> 20); // originally 31 bits of info (sign lost in squaring), shift 20 gives 11, means 22 bits space
	                                                               // then, we may add this 1024 time with means we need 10 more bits for addition space
	
	if (sub_sample >= 441 ) // then save into the sound bank
	{   
        led_enabled = 0;
        if ( global_frame_num == global_beat_offset)
            led_enabled = 1;
     
        global_frame_num++;
        
        if (global_frame_num >= global_beat_period)
            global_frame_num = 0;
         
		sub_sample = 0;
		sound_data_a[unit_sample] = square_sum;
		square_sum = 0;
		unit_sample++;
		if (unit_sample >= BEAT_WINDOW_UNITS)
		{
			unit_sample = 0;
			sound_data_ready = 1;
			// time to process autocorrelation of the beat frame that is BEAT_WINDOW_SECONDS long, should take about 1 ms
		}
	}
	
	DMA_IFR = 0xFFFF;
	asm(" BCLR INTM"); // Enable interrupts
	return;
}

void dma_init(void)
{
    DMA_MSK = 0xFFFF; // dma0 ch 0
    DMA_IFR = 0xFFFF;     // clear interrupt flags
    DMA0_CH0_TC_MSW = 0x0000;
	DMA0_CH0_TC_MSW |= 1 << 5;  // INTEN enabled dma0 channel 0
	DMA0_CH10_EVENT_SRC = 0x02;  // i2so receive event
	DMA0_CH0_SRC_LSW = 0x2828; // pull data from i2s0 rx buffer
	DMA0_CH0_SRC_MSW = 0x0000; //                i2s0 rx buffer
	DMA0_CH0_DST_LSW = ((unsigned long)((unsigned long)((unsigned long)(&sound_bytes[0]) << 1) + (unsigned long)0x10000) & 0xffff);
	DMA0_CH0_DST_MSW = (unsigned long)(((unsigned long)((unsigned long)(&sound_bytes[0]) >> 15) + (unsigned long)(0x10000 >> 16)) & 0xffff);
	DMA0_CH0_TC_LSW  = 0x04; // TC1, copy 1 word
	
    //                 autorld |cnst dst|cnst src
	DMA0_CH0_TC_MSW  = 1 << 12 | 1 << 9 | 1 << 7;
	
	//DMA0_CH0_TC_MSW |= 1 << 2;                    // sync mode enabled;
	// dma should now be initialized
	
}

void i2s0_init()
{
	set_i2s0_slave();
}

extern void AIC3254_init();

void codec_init()
{
    AIC3254_init();
}

//GPAIN0 and GPAIN1 are used, corresponding to channels 2 and 3
// GPAIN0 corresponds to width of base to check
// GPAIN1 corresponds to minimum height that triggers beat
void ADC_init()
{
	SARCLKCTRL = 99; // 100 MHz / (99 + 1) = 1 MHz
	SARPINCTRL = 0x3501; // 0011 0101   0000 0001 
}

unsigned int get_ADC_ch(unsigned int channel)
{
	int j; 
	
	// single conversion, select channel
	SARCTRL = (channel << 12) | 1 << 10;
	
	// start conversion
	SARCTRL |= 1 << 15; 
	
	while (!(SARDATA & 0x8000)); // wait  for end of busy signal
	while (SARDATA & 0x8000); // wait  for end of busy signal
	
	return (SARDATA & 0x3ff);
	
}


void InitSystem(void)
{
	Uint16 i;
    // PLL set up from RTC
    // bypass PLL
    CONFIG_MSW = 0x0;

    PLL_CNTL2 = 0x8000;
    PLL_CNTL4 = 0x0000;
    PLL_CNTL3 = 0x0806;
    PLL_CNTL1 = 0x82FA;

    while ( (PLL_CNTL3 & 0x0008) == 0);
    // Switch to PLL clk
    CONFIG_MSW = 0x1;

// clock gating
// enable all clocks
    IDLE_PCGCR = 0x0;
    IDLE_PCGCR_MSW = 0xFF84;
    

// reset peripherals
    PER_RSTCOUNT = 0x02;
    PER_RESET = 0x00fb;    
    for (i=0; i< 0xFFF; i++);
}


void ConfigPort(void)
{
    Int16 i;
    //  configure ports
    PERIPHSEL0 = 0x6900;        // parallel port: mode 6, serial port1: mode 2 
    

    for (i=0; i< 0xFFF; i++);
}

void turnOnLED(void)
{
    Uint16 temp;
    
    temp = ST1_55;
    if((temp&0x2000) == 0)
    {
        // turn on LED
        temp |= 0x2000;
        ST1_55 =temp;
    }
    
}


void turnOffLED(void)
{
    Uint16 temp;
    
    temp = ST1_55;
    if((temp&0x2000) != 0)
    {
        // turn off LED
        temp &=0xDFFF;
        ST1_55 =temp;
    }
}



void main(void)
{
	Int32 * sound_data_ptr;
	float beat_s;
	int beat_frames;
	int   beat_n;
	int   beat_offset;
	Uint32 beat_mag;
	
	int led_count = 0;
	int i = 0;
	int j = 0;
	
	// initialization of periph / DSP
	InitSystem();
	ConfigPort();
	SYS_GlobalIntEnable(); // appears to immediately cause an interrupt
	
	for ( i = 0; i < 255; i++);
	
	// should do RTC setup here?
	IER0 = 0x0110;      // enable dma, timer int      
    IER1 = 0x0004;      // enable RTC int
	CONFIG_MSW = 0x00;
	
	
    PLL_CNTL2 = 0x8000;
    PLL_CNTL4 = 0x0000;
    PLL_CNTL3 = 0x0806;
    PLL_CNTL1 = 0x82ED;

    while ( (PLL_CNTL3 & 0x0008) == 0);
    // Switch to PLL clk
    CONFIG_MSW = 0x1; // appears to fuck up clock
    
    for (i = 0; i < 255; i++);
	// setup clock to i2c/i2s.  PLL configuration?
	// setup i2c
	// config aic
	// setup i2s w/ DMA copy, possibly w/ interrupt


    PER_RESET &= (~(1 << 0)) & (~(1 << 4)) & (~(1 << 5)); // i2c i2s0 dma enable
    
    IDLE_PCGCR &= ~(1 << 8);    // enable i2c
    IDLE_PCGCR &= ~(1 << 6);   // enable i2s0
    IDLE_PCGCR &= ~(1 << 3);   // enable DMA0
    IDLE_PCGCR &= ~(1);
    
    IDLE_PCGCR_MSW &= ~(1 << 1);
    
    asm(" IDLE");              // let idle register do its work

    SYS_GlobalIntDisable();
    DMA_IFR = 0xffff; // clear dma interrupts
    IFR0 = 0x0000;    // clear cpu interrupts
    IFR1 = 0x0000;    // clear cpu interrupts
    
    IER0 |= 1 << 8; // DMA interrupt enabled

	// here is where we configure DMA for i2s
	
	IFR0 = 0x0000;    // clear cpu interrupts
    IFR1 = 0x0000;    // clear cpu interrupts
	
	
	codec_init();
	dma_init();
	i2s0_init();	
	// enable DMA interrupt
	DMA0_CH0_TC_MSW &= ~0x8004; // disable syncmode / enable
	DMA0_CH0_TC_MSW |= (1 << 15) | (1 << 13) | (1 << 2);         // interrupt enable, enabled
	DMA0_CH0_TC_MSW |= (unsigned int)0x8004;
	
    SYS_GlobalIntEnable();
	
	I2S0_CR_LSW |= 0x8000;// i2s0 enabled; bus error?
	
	// problem seems to occur when both dma and i2s0 enabled, with bus interrupt
	// being caused when the last of the two is enabled
	
	
	// i2s0 should be enabled here
	
	// after configurable DMA, enable global interrupts again
	
	asm(" INTR #8");
	//asm(" INTR #9"); 
	
	for (i = 0; i < 1024; i++)
	{
		data_data[i] = (i % 8 == 0) ? 0x050000 : 0;
	}
	
	fft_mag_1024((Int32 *)data_data, (Uint32 *)result_data);
	
	// search for max between 1/10 s and 5 s
	
	
	
	
	//setup codec via i2c
	//setup i2s codec input
	
	//ADC_init();
	
   while(1)
   {
   	
   	if (led_enabled)
   	   turnOnLED();
   	else
   	   turnOffLED();
   	
   // New audio sample available?
   // read i2s buffer for l & r channel 
   
  
   //   --combine left and right channels
   //   --Filter sample to ~2.5khz via FIR filter
   //   --Decimate to 6 kHz sampling
   //   -- take 1024 point FFT (6 Hz resolution)
   //   -- determine 3 highest magnitude frequencies
   //   -- determine magnitude of peaks
   //   -- record the three peak/mag pairs

   // use previous peak/mag pair data to determine if should send sig
   
      SYS_GlobalIntEnable();
      turnOffLED();
      if (sound_data_ready)
      {
      	
      	// hack this for the first square of sig test
         if (sound_bank == BANK_A)
    	    sound_data_ptr = sound_data_a;
    	 else
    	    sound_data_ptr = sound_data_a;  
      	
      	// process fft of audio signal
      	for (i = 0; i < 1024; i++)
      	{
      		data_data[i] = (sound_data_ptr[i] & 0xfffff000) >> 12;
      	}
        corellation_mag_1024((Int32 *)data_data, (Uint32 *)result_data);
        
        beat_n = 0;
        beat_s = 0.0f;
        beat_mag = 0;
        
        for (i = 10; i <500; i++)
        {
        	if (result_data[i] > beat_mag)
        	{
        		beat_mag = result_data[i];
        	    beat_n = i;
        	}
        }
        
        beat_s = (float)beat_n / 54.00;
        
        // now we know how many frames a period is, find the beat offset
        
        
        
        for ( i = 0; i < beat_n; i++)
        {
        	sound_data_b[i] = 0;
        }
        
        // don't go more than 10 frames back
        beat_frames = 1024 / beat_n;
        if (beat_frames > 10)
           beat_frames = 10;
        
        for ( i = 0; i < (beat_frames); i++)
        {
        	for (j = 0; j < beat_n; j++)
        	{
        		sound_data_b[j] += sound_data_ptr[1024 - beat_n * (i + 1) + j] >> 9;
        	}
        }
        
        
        beat_mag = 0;
        
        for ( i = 0; i < beat_n; i++)
        {
        	if ( sound_data_b[i] > beat_mag)
        	{
        		beat_mag = sound_data_b[i];
        		beat_offset = i;
        	}
        }
        
        // now we know beats in seconds
        
        
        global_beat_offset = beat_offset;
        global_beat_period = beat_n;
        global_frame_num   = 0;
        
      	sound_data_ready = 0;
      }
   
   }
   
   return;
}

// input : array of 32 bit complex number (16 signed real, 16 signed imaginary)
// output: unsigned int32 magnitude of autocorrelation, unscaled
// comprises two 1024 point FFT, one 1024 point complex multipication
void corellation_mag_1024(Int32 * in, Uint32 * result_mag)
{
	Int32 * data = in;
	Int32 * data_br = (Int32 *)data_br_data;
	Int32 * result = (Int32 *)result_data;
	Int16 * complex_res;
	Int16 * complex_conj;
	Int32 * scratch = (Int32 *)scratch_data;
	
	Uint16 out_sel;
	Uint16 fft_flag;
	Uint16 scale_flag;
	
	int i;
	
	//SYS_GlobalIntDisable();
	
	fft_flag = FFT_FLAG;
	scale_flag = SCALE_FLAG;
	
	/* Bit-Reverse 1024-point data, Store into data_br, data_br aligned to
	12-least significant binary zeros */
	hwafft_br(data, data_br, DATA_LEN_1024);
	data = data_br;
	
	/* Compute 1024-point FFT, scaling disabled */
	out_sel = hwafft_1024pts(data, scratch, fft_flag, scale_flag);
	if (out_sel == OUT_SEL_DATA) {
	   result = data;
	} else {
	   result = scratch;
	}
	
	// take magnitude of FFT
    complex_res = (Int16 *)result;
    complex_conj = (Int16 *)result_mag;
    
    // result mag is complex conjugate of fft, result is original
    
	for (i = 0; i < 1024; i++)
    { 
    	complex_conj[i * 2]     = -1 * (complex_res[i * 2] >> 1) * (complex_res[i * 2] >> 1);
    	complex_conj[i * 2 + 1] =  (complex_res[i * 2 + 1] >> 1) * (complex_res[i * 2 + 1] >> 1);
    }
    
    data = in;
    
    for (i = 0; i < 1024; i++)
    {
    	data[i] = ((Uint32 *)result_mag)[i];
    }
    
    hwafft_br(data, data_br, DATA_LEN_1024);
	data = data_br;
    
    // bit reverse this data
    
	/* Compute 1024-point IFFT, scaling disabled */
	out_sel = hwafft_1024pts(data, scratch, IFFT_FLAG, scale_flag);
	if (out_sel == OUT_SEL_DATA) {
	   result = data;
	} else {
	   result = scratch;
	}
    
    complex_res = (Int16 *)result;
    
		for (i = 0; i < 1024; i++)
    { 
    	result_mag[i] = (Int32)complex_res[i * 2] * (Int32)complex_res[i * 2] + 
    	            (Int32)complex_res[i * 2 + 1] * (Int32)complex_res[i * 2 + 1];
    }
}


// 1024 point fft, magnitude squared
// input : array of 32 bit complex number (16 signed real, 16 signed imaginary)
// output: unsigned int32 magnitude of fft, unscaled
void fft_mag_1024( Int32 * in, Uint32 * result_mag)
{
	
	Int32 * data = in;
	Int32 * data_br = (Int32 *)data_br_data;
	Int32 * result = (Int32 *)result_data;
	Int16 * complex_res;
	Int32 * scratch = (Int32 *)scratch_data;
	
	Uint16 out_sel;
	Uint16 fft_flag;
	Uint16 scale_flag;
	
	int i;
	
	//SYS_GlobalIntDisable();
	
	fft_flag = FFT_FLAG;
	scale_flag = SCALE_FLAG;
	
	/* Bit-Reverse 1024-point data, Store into data_br, data_br aligned to
	12-least significant binary zeros */
	hwafft_br(data, data_br, DATA_LEN_1024);
	data = data_br;
	
	/* Compute 1024-point FFT, scaling disabled */
	out_sel = hwafft_1024pts(data, scratch, fft_flag, scale_flag);
	if (out_sel == OUT_SEL_DATA) {
	   result = data;
	} else {
	   result = scratch;
	}
	
	// take magnitude of FFT
    complex_res = (Int16 *)result;
    
	for (i = 0; i < 1024; i++)
    { 
    	result_mag[i] = (Int32)complex_res[i * 2] * (Int32)complex_res[i * 2] + 
    	            (Int32)complex_res[i * 2 + 1] * (Int32)complex_res[i * 2 + 1];
    }
    
    //SYS_GlobalIntEnable();
    
}


