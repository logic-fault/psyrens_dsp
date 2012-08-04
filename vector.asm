;//////////////////////////////////////////////////////////////////////////////
;// * File name: vector.asm
;// *                                                                          
;// * Description:  Code for playback using I2S2 and DMA1 to AIC3254.
;// *                                                                          
;// * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
;// *                                                                          
;// *                                                                          
;// *  Redistribution and use in source and binary forms, with or without      
;// *  modification, are permitted provided that the following conditions      
;// *  are met:                                                                
;// *                                                                          
;// *    Redistributions of source code must retain the above copyright        
;// *    notice, this list of conditions and the following disclaimer.         
;// *                                                                          
;// *    Redistributions in binary form must reproduce the above copyright     
;// *    notice, this list of conditions and the following disclaimer in the   
;// *    documentation and/or other materials provided with the                
;// *    distribution.                                                         
;// *                                                                          
;// *    Neither the name of Texas Instruments Incorporated nor the names of   
;// *    its contributors may be used to endorse or promote products derived   
;// *    from this software without specific prior written permission.         
;// *                                                                          
;// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     
;// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       
;// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   
;// *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    
;// *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   
;// *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        
;// *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   
;// *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   
;// *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     
;// *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   
;// *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    
;// *                                                                          
;//////////////////////////////////////////////////////////////////////////////

         .mmregs
         .include "lpva200.inc"
         
         
         .C54CM_off
         .CPL_off
         .ARMS_off
   
;**********************************************************************************
;        predefined stack operation modes  
;**********************************************************************************
;        USE_RETA     : 2x16-bit fast return mode (RETA used)
;        NO_RETA      : 2x16-bit slow return mode (RETA not used)
;        C54X_STK     : 32-bit fast return mode 

;**********************************************************************************    
         .global _DMA_Isr 
         ;.global _SAR_Isr 
         ;.global _I2S0_TX_Isr
         ;.global _I2S1_RX_Isr
         .global _RTC_Isr
		 ;.global _Timer_isr
         .global _Bus_Isr
         .global _Default_Isr
             
;**********************************************************************************
        .sect "vector"
        .align  256         
;**********************************************************************************

;****************************************************************************
;* Other interrupt vector definitions go here
;****************************************************************************
	.def	_RST
;_RST:		.ivec    reset_isr, USE_RETA; Reset / Software Interrupt #0
_RST:		.ivec    _Default_Isr, USE_RETA ; Reset / Software Interrupt #0

;NMI:		.ivec    reset_isr				; Nonmaskable Interrupt
NMI:		.ivec    _Default_Isr				; Nonmaskable Interrupt

INT0:		.ivec    _Default_Isr				; External User Interrupt #0 

INT1:		.ivec    _Default_Isr				; External User Interrupt #1

;TINT0:		.ivec    _Timer_isr				; Timer #0 / Software Interrupt #4
TINT0:		.ivec    _Default_Isr				; Timer #0 / Software Interrupt #4

;PROG0:		.ivec    _I2S0_TX_Isr			; Programmable 0 Interrupt
PROG0:		.ivec    _Default_Isr			; Programmable 0 Interrupt

UART:		.ivec    _Default_Isr				; IIS #1 Receive Interrupt

PROG1:		.ivec    _Default_Isr				; Programmable 1 Interrupt

;DMA:		.ivec    DMA_Isr_2                ; DMA Interrupt
DMA:		.ivec    _DMA_Isr                ; DMA Interrupt
;DMA:		.ivec    _Default_Isr                ; DMA Interrupt

PROG2:		.ivec    _Default_Isr			; Programmable 2 Interrupt

COPROCFFT:  .ivec    _Default_Isr			; Coprocessor FFT Module Interrupt

;PROG3:		.ivec    _I2S1_RX_Isr			; Programmable 3 Interrupt
PROG3:		.ivec    _Default_Isr			; Programmable 3 Interrupt

LCD:		.ivec	 _Default_Isr				; LCD Interrupt

SARADC:		.ivec    _Default_Isr				; SAR ADC Interrupt

XMIT2:		.ivec	 _Default_Isr		    ; I2S2 Tx Interrupt

RCV2:		.ivec	 _Default_Isr		    ; I2S2 Rx Interrupt

XMIT3:		.ivec	 _Default_Isr		    ; I2S3 Tx Interrupt

RCV3:		.ivec	 _Default_Isr		    ; I2S3 Rx Interrupt

;RTC:		.ivec    _RTC_Isr				; RTC interrupt
RTC:		.ivec    _Default_Isr				; RTC interrupt

SPI:    	.ivec    _Default_Isr				; SPI Receive Interrupt

USB:		.ivec    _Default_Isr				; USB Transmit Interrupt

GPIO:		.ivec    _Default_Isr				; GPIO Interrupt

EMIF:	    .ivec    _Default_Isr			    ; EMIF Interrupt

I2C:		.ivec    _Default_Isr				; IIC interrupt

BERRIV:
IV24:		.ivec	 _Default_Isr   ; Bus error interrupt

;    .ref  _DLOGINT_isr 
DLOGIV:
IV25:		.ivec   _Default_Isr  ; Data log (RTDX) interrupt

;	.ref _RTOSINT_isr
RTOSIV:
IV26:		.ivec _Default_Isr  ; Real-time OS interrupt
IV27:		.ivec _Default_Isr  ; General-purpose software-only interrupt
IV28:		.ivec _Default_Isr  ; General-purpose software-only interrupt
IV29:		.ivec _Default_Isr  ; General-purpose software-only interrupt
IV30:		.ivec _Default_Isr  ; General-purpose software-only interrupt
IV31:		.ivec _Default_Isr  ; General-purpose software-only interrupt


;****************************************************************************
;* Reset 
;****************************************************************************

		.text
		.def reset_isr
		.ref _c_int00
		
        .align 2		
reset_isr:
	;	*port(#0x1C01) = #0x0				; Clear idles
		bit (ST1, #11) = #1					; Disable interrupts
    	@IVPD_L = #(_RST >> 8) || mmap()
    	@IVPH_L = #(_RST >> 8) || mmap()
	   	 bit(ST3,#7) = #0		; Clear bus error interrupts
	   	 bit(ST3,#2) = #1		; shut off clockout port
	   	 bit(ST1,#13) = #0		; shut off XF port

		@#IFR0_L = #0xffff || mmap() ; clear all pending interrupts
		@#IFR1_L = #0xffff || mmap()

		*port(#IDLE_ICR) = #(RESERVED_ICR|IPORT_IDLE|HWA_IDLE|DPORT_IDLE)
		idle

;**********************************************************************************
;     Reset all peripherals
;**********************************************************************************
	*port(#0x1C04) = 0x1
	nop_16
	*port(#0x1C05) = 0x00FB	; Reset all peripherals
	nop_16


;**********************************************************************************
;     Enalbe EMIF
;**********************************************************************************

    *port(IDLE_PCGCR) = #0x0

    ;/* Config EMIF - System Control Regsiter */
    *port(#0x1C33) = #0x0  

    ;// for SRAM in memory card (Async_CE1)
    ;/* Config EMIF - ASYNC Regsiters */
    *port(#0x1004) = #0x0080     
    *port(#0x1005) = #0x00E4     

	;/* Configure as 16-bit data bus */    
    ;// Async4 ==> Async_CE1 (SRAM)	
    *port(#0x101C) = #0x40AD     
    *port(#0x101D) = #0x0020     

    ;// Async3 ==> Async_CE0(Flash)
    *port(#0x1018) = #0xFFFD     
    *port(#0x1019) = #0x3FFF     


    ;// do not assign Async_CE0 and Async_CE1 for NAND
    *port(#0x1060) = #0x0003     
    
    ;// Turn off page mode for all Chip Selects
    *port(#0x1068) = #0xFCFC     
    *port(#0x1069) = #0xFCFC     

	goto	_c_int00
		
********************************************************************************
** Name     : no_isr                                                          **
**                                                                            **
** Purpose  : Spurious interrupt handler                                      **
**                                                                            **
** Author   :                                                                 **
**                                                                            **
********************************************************************************
             .text   
no_isr:      goto no_isr 

DMA_Isr_2:
    return_int

dummy_isr:
    return_int
 
    
    .end