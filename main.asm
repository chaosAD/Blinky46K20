;*******************************************************************************
;                                                                              *
;    Microchip licenses this software to you solely for use with Microchip     *
;    products. The software is owned by Microchip and/or its licensors, and is *
;    protected under applicable copyright laws.  All rights reserved.          *
;                                                                              *
;    This software and any accompanying information is for suggestion only.    *
;    It shall not be deemed to modify Microchip?s standard warranty for its    *
;    products.  It is your responsibility to ensure that this software meets   *
;    your requirements.                                                        *
;                                                                              *
;    SOFTWARE IS PROVIDED "AS IS".  MICROCHIP AND ITS LICENSORS EXPRESSLY      *
;    DISCLAIM ANY WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING  *
;    BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS    *
;    FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL          *
;    MICROCHIP OR ITS LICENSORS BE LIABLE FOR ANY INCIDENTAL, SPECIAL,         *
;    INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO     *
;    YOUR EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR    *
;    SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY   *
;    DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER      *
;    SIMILAR COSTS.                                                            *
;                                                                              *
;    To the fullest extend allowed by law, Microchip and its licensors         *
;    liability shall not exceed the amount of fee, if any, that you have paid  *
;    directly to Microchip to use this software.                               *
;                                                                              *
;    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF    *
;    THESE TERMS.                                                              *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Filename:                                                                 *
;    Date:                                                                     *
;    File Version:                                                             *
;    Author:                                                                   *
;    Company:                                                                  *
;    Description:                                                              *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Notes: In the MPLAB X Help, refer to the MPASM Assembler documentation    *
;    for information on assembly instructions.                                 *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Known Issues: This template is designed for relocatable code.  As such,   *
;    build errors such as "Directive only allowed when generating an object    *
;    file" will result when the 'Build in Absolute Mode' checkbox is selected  *
;    in the project properties.  Designing code in absolute mode is            *
;    antiquated - use relocatable mode.                                        *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Revision History:                                                         *
;                                                                              *
;*******************************************************************************



;*******************************************************************************
; Processor Inclusion
;
; TODO Step #1 Open the task list under Window > Tasks.  Include your
; device .inc file - e.g. #include <device_name>.inc.  Available
; include files are in C:\Program Files\Microchip\MPLABX\mpasmx
; assuming the default installation path for MPLAB X.  You may manually find
; the appropriate include file for your device here and include it, or
; simply copy the include generated by the configuration bits
; generator (see Step #2).
;
;*******************************************************************************
    list   p=p18f46k20, r=dec

; TODO INSERT INCLUDE CODE HERE
#include <p18f46k20.inc>

;*******************************************************************************
;
; TODO Step #2 - Configuration Word Setup
;
; The 'CONFIG' directive is used to embed the configuration word within the
; .asm file. MPLAB X requires users to embed their configuration words
; into source code.  See the device datasheet for additional information
; on configuration word settings.  Device configuration bits descriptions
; are in C:\Program Files\Microchip\MPLABX\mpasmx\P<device_name>.inc
; (may change depending on your MPLAB X installation directory).
;
; MPLAB X has a feature which generates configuration bits source code.  Go to
; Window > PIC Memory Views > Configuration Bits.  Configure each field as
; needed and select 'Generate Source Code to Output'.  The resulting code which
; appears in the 'Output Window' > 'Config Bits Source' tab may be copied
; below.
;
;*******************************************************************************

; TODO INSERT CONFIG HERE
    CONFIG WDTEN=OFF    ; Disable watchdog timer
    CONFIG HFOFST=OFF   ; CPU clocking without waiting for OSC to stabilize
    CONFIG DEBUG=ON     ; Enable debugging mode
    CONFIG MCLRE=ON     ; MCU clear pin on
    CONFIG LVP=OFF      ; Disable low-voltage programming
    CONFIG PBADEN=OFF   ; Port B configured NOT as analog (digital I/O instead)
    CONFIG CCP2MX=PORTBE; CCP2 on RB3
    CONFIG FOSC=INTIO67

#define PWM_PERIOD      (d'43' << 2)
#define PWM_DUTY        d'16'
#define PWM_DUTY_LOW    d'160'
#define TIMER2_CONFIG   0

;*******************************************************************************
;
; TODO Step #3 - Variable Definitions
;
; Refer to datasheet for available data memory (RAM) organization assuming
; relocatible code organization (which is an option in project
; properties > mpasm (Global Options)).  Absolute mode generally should
; be used sparingly.
;
; Example of using GPR Uninitialized Data
;
;   GPR_VAR        UDATA
;   MYVAR1         RES        1      ; User variable linker places
;   MYVAR2         RES        1      ; User variable linker places
;   MYVAR3         RES        1      ; User variable linker places
;
;   ; Example of using Access Uninitialized Data Section (when available)
;   ; The variables for the context saving in the device datasheet may need
;   ; memory reserved here.
;   INT_VAR        UDATA_ACS
;   W_TEMP         RES        1      ; w register for context saving (ACCESS)
;   STATUS_TEMP    RES        1      ; status used for context saving
;   BSR_TEMP       RES        1      ; bank select used for ISR context saving
;
;*******************************************************************************

; TODO PLACE VARIABLE DEFINITIONS GO HERE
INT_VAR        UDATA_ACS
delayCounter   RES        2
dataByte       RES        1

;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    start                   ; go to beginning of program

    CODE    0x0008                  ; interrupt vector
    movlw   PWM_DUTY >> 2
    btfss   dataByte, 0
    movlw   PWM_DUTY_LOW >> 2
    movwf   CCPR2L
    bsf     LATB, RB1
    bcf     LATB, RB1
    rrncf   dataByte
    bcf     PIR1, TMR2IF
    retfie

;*******************************************************************************
; TODO Step #4 - Interrupt Service Routines
;
; There are a few different ways to structure interrupt routines in the 8
; bit device families.  On PIC18's the high priority and low priority
; interrupts are located at 0x0008 and 0x0018, respectively.  On PIC16's and
; lower the interrupt is at 0x0004.  Between device families there is subtle
; variation in the both the hardware supporting the ISR (for restoring
; interrupt context) as well as the software used to restore the context
; (without corrupting the STATUS bits).
;
; General formats are shown below in relocatible format.
;
;------------------------------PIC16's and below--------------------------------
;
; ISR       CODE    0x0004           ; interrupt vector location
;
;     <Search the device datasheet for 'context' and copy interrupt
;     context saving code here.  Older devices need context saving code,
;     but newer devices like the 16F#### don't need context saving code.>
;
;     RETFIE
;
;----------------------------------PIC18's--------------------------------------
;
; ISRHV     CODE    0x0008
;     GOTO    HIGH_ISR
; ISRLV     CODE    0x0018
;     GOTO    LOW_ISR
;
; ISRH      CODE                     ; let linker place high ISR routine
; HIGH_ISR
;     <Insert High Priority ISR Here - no SW context saving>
;     RETFIE  FAST
;
; ISRL      CODE                     ; let linker place low ISR routine
; LOW_ISR
;       <Search the device datasheet for 'context' and copy interrupt
;       context saving code here>
;     RETFIE
;
;*******************************************************************************

; TODO INSERT ISR HERE

;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE                      ; let linker place main program

start
    rcall   setMcuFreqTo64MHz
    rcall   configurePins

    movlw   0x72
    movwf   dataByte
    rcall   configurePWM

    ; TODO Step #5 - Insert Your Program Here
tryAgain
    bsf     LATB, RB0
    rcall   delay
    bcf     LATB, RB0
    rcall   delay
    bra     tryAgain
    ; End of start


setMcuFreqTo64MHz
    rcall   setMcuFreqTo16MHz
    bsf     OSCTUNE, PLLEN         ; Multiply Fosc by 4
    return

setMcuFreqTo16MHz
    movlw       0x70
    iorwf       OSCCON
    return

; Subroutine to configre pins
; LED pin : RB0
; Signal pin: RB1
; PWM pin : RB3 (CCP2)
configurePins
    bcf     TRISB, RB0              ; Make RB0 as output pin
    bcf     TRISB, RB1              ; Make RB1 as output pin
    return

configurePWM
    bsf     TRISB, RB3              ; Disable CCP2 (PWM) [force to input]
    movlw   PWM_PERIOD >> 2
    movwf   PR2
    movlw   ((PWM_DUTY & 3) << 4) | 0xc
    movwf   CCP2CON
    movlw   PWM_DUTY >> 2
    movwf   CCPR2L
    bcf     PIR1, TMR2IF
    movlw   TIMER2_CONFIG
    movwf   T2CON
    bsf     T2CON, TMR2ON
    bsf     INTCON, GIE
    bsf     INTCON, PEIE
pwmWaitTillReady
    btfss   PIR1, TMR2IF
    bra     pwmWaitTillReady
    bcf     TRISB, RB3              ; Enable CCP2 (PWM) pin
    bsf     PIE1, TMR2IE            ; Enable Timer2 interrupt
    return

delay
    movlw   d'80'
    movwf   delayCounter+1
delayOuterRepeat
    movlw   d'255'
    movwf   delayCounter
delayMidRepeat
    movlw   d'255'
delayInnerRepeat
    decfsz  WREG
    bra     delayInnerRepeat
    decfsz  delayCounter
    bra     delayMidRepeat
    decfsz  delayCounter+1
    bra     delayOuterRepeat
    return

    END