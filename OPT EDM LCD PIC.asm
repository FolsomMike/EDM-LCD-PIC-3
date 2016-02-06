;--------------------------------------------------------------------------------------------------
; Project:  OPT EDM Notch Cutter -- LCD PIC software
; Date:     2/29/12
; Revision: 1.0
;
; IMPORTANT: When programming the PIC in the notch cutter, turn the Electrode Current switch to
; Off and the Electrode Motion switch to Setup.
;
; Normally, the programming header for the LCD PIC is not installed on the board.  It can be
; installed in the Main PIC socket, programmed, and then moved to the LCD PIC socket.
;
; Overview:
;
; This program reads serial data sent by the Main PIC and displays it on the LCD. All data is
; first stored in a local buffer which is then repeatedly transmitted to the LCD display. This
; constant refreshing corrects errors which occur in the displayed text due to electrical noise
; from the cutting current causing spikes in the LCD display control lines.
;
; There are two PIC controllers on the board -- the Main PIC and the LCD PIC.  This code is
; for the LCD PIC.  The Main PIC sends data to the LCD PIC via a serial data line for display
; on the LCD.
;
;--------------------------------------------------------------------------------------------------
;
; Revision History:
;
; 1.0   Some code and concepts used from source code disassembled from hex object code version 2.7 
;       from original author.
; 2.0	Major change to methodolgy. The data from the master PIC is now written to a local buffer
;		which is then continuously and repeatedly transmitted to the LCD display. All control codes
; 		received from the master except address change codes are transmitted straight to the display.
;		The constant refreshing of the display serves to correct errors caused by noise from the
;		cutting current. The errors manifested as changed or missing characters in the display.
; 2.1  No changes -- used to verify program downloaded into PIC.
;
;--------------------------------------------------------------------------------------------------
;
;--------------------------------------------------------------------------------------------------
; LCD Notes for the OPT EDM Control Board I
;
; Optrex C-51847NFJ-SLW-ADN 20 characters by 4 lines
;
; The user manual specified for this display is Dmcman_full-user manual.pdf from www.optrex.com
; This manual does not list this exact part number, but seems to be the appropriate manual.
;
; The R/W line is tied permanently low on the board, so only writes to the LCD are allowed.
;
; The E line is used to strobe the read/write operations.
;
; Addressing ---
;
; LCD ADDRESSING NOTE: LCD addressing is screwy - the lines are not in sequential order:
;
; line 1 column 1 = 0x80  	(actually address 0x00)
; line 2 column 1 = 0xc0	(actually address 0x40)
; line 3 column 1 = 0x94	(actually address 0x14)
; line 4 column 1 = 0xd4	(actually address 0x54)
;
; To address the second column in each line, use 81, C1, 95, d5, etc.
;
; The two different columns of values listed above are due to the fact that the address
; is in bits 6:0 and control bit 7 must be set to signal that the byte is an address
; byte.  Thus, 0x00 byte with the control bit set is 0x80.  The 0x80 value is what is
; actually sent to the LCD to set address 0x00.
;
;  Line 3 is actually the continuation in memory at the end of line 1
;    (0x94 - 0x80 = 0x14 which is 20 decimal -- the character width of the display)
;  Line 4 is a similar extension of line 2.
;
; Note that the user manual offered by Optrex shows the line addresses
; for 20 character wide displays at the bottom of page 20.
;
; Cursor and Blinking ---
;
; The display has the capability to display a cursor and/or blink the character at the cursor
; location. This is not used in this program. Since the screen is refreshed by redrawing
; constantly, the cursor or blinking is seen racing across the screen as it follows each character
; written. It was too complicated to turn it off during refresh, then delay long enough for it
; to be seen after turning it back on.
;
; Instead, blinking is handled in the PIC code by replacing the character to be blinked by a
; space when it is transmitted.
;
;--------------------------------------------------------------------------------------------------
; Notes on PCLATH
;
; The program counter (PC) is 13 bits. The lower 8 bits can be read and written as register PCL.
; The upper bits cannot be directly read or written.
;
; When a goto is executed, 11 bits embedded into the goto instruction are loaded into the PC<10:0>
; while bits PCLATH<4:3> are copied to bits PC<12:11>
;
; When a goto is executed, 11 bits embedded into the goto instruction are loaded into the lower
; 11 bits of PC while bits PCLATH<4:3> are copied to bits PC<12:11>
;
; Changing PCLATH does NOT instantly change the PC register. The PCLATH will be used the next time
; a goto is executed (or similar opcode) or the PCL register is written to. Thus, to jump farther
; than the 11 bits (2047 bytes) in the goto opcode will allow, the PCLATH register is adjusted
; first and then the goto executed.
;
;--------------------------------------------------------------------------------------------------
; Serial Data from Main PIC
;
; These notes are nearly identical for "OPT EDM LCD PIC.ASM" and "EDM MAIN PIC.ASM". Changes to
; either should be copied to the other. The "LCD" version uses internal 4Mhz clock for Fosc
; while "MAIN" uses external ???Mhz clock. Thus the timing are different for using Timer0 as
; an interrupt source.
;
; The "Main" PIC sends data to the "LCD" PIC via PortA,0 (RA0). A word is sent for each 
; command/data value:
;
; Data Format --
;
; the first byte is a control byte and specifies either an LCD command or an LCD character
; 	0 = data; 1 = command; other may be a command for a different PIC listening on the same line
;
; the second byte is either an LCD command or an LCD character
;
; PIC16 Timing --
;
; These notes were verified using an oscilloscope.
;
; The "LCD" PIC is configured to use the internal oscillator of 4Mhz (FOSC). The instruction cycle
; frequency is Fosc/4. All instructions except branches take 1 cycle.
;
; Timer0 increment frequency is Fosc/4 without prescaler -- in the "LCD" program the prescaler
; is assigned to the WatchDog, thus Timer0 freq = Fosc/4 (same as the cycle frequency)
;
; Serial Timing --
;
; each serial bit from the Main PIC is 64 uS wide 
;
; a single instruction cycle (a nop) on the "LCD" PIC is 1uS wide
; a goto instruction is 3uS
; a simple decfsz loop can be used for bit to bit timing
;
; Loop Code Timing
;
;	loop1: 	decfsz	counter,F
;    		goto    loop1
;
; if you are using a bsf/bcf pair to create a pulse train for verifying this
; 	measurement, the one cycle of the bcf will be included in the wait time and must be
;	accounted for by subtracting 1 cycle from the resulting delay times; for the board used
;	for this test, one cycle = 1 us
;
; delay for different starting values of counter:
;
;  2 -> 5 uS
;  3 -> 8 uS (each additional count -> 3 uS)
; 21 -> 64 uS  ~ ((64 - 5) / 3) + 2
; 
; in actual use: 19 gives the closest result taking all loop cycles into account
;
; Version 1.0 of the LCD PIC code read the word all at one time and then transmitted to the display
; between words without using an interrupt. The time between words is significantly larger
; allowing for the time for the display update. For version 2.0, which constantly refreshes the
; display from a local buffer, the time required for display access code is more than that allowed
; between incoming serial words. Thus, an interrupt is used to monitor for start bits.
;
; For reasonably accurate detection of a start bit, checking the input every 1/3 bit width (21 uS)
; is desired -- worst case should be detecting the start bit 2/3 after the down transition
; this should be close enough to successfully detect the remaining 8 bits, allowing for 21 uS
; of timing slop...
;
; ...HOWEVER...
;
; ...the interrupt takes up 15 cycles at its quickest and the program would spend all of its
; time in the interrupt routine. So the input will actually be checked every 32 uS (32 cycles)
; or one/half the time between bits. This still leaves only 17 cycles or so for the main thread
; to execute for every call to the interrupt, but the main thread is not time critical.
;
; When the interrupt detects a start bit, the main thread will not run again until an entire
; word is read. This could probably be changed to return to the main thread while waiting for the
; next bit, but that would require another mode flag check, more lost cycles, and a lot more
; complexity. The main thread will get 12 cycle blocks when between words and when the "Main" PIC
; is not sending data.
;
; NOTE: Because the main thread does not execute while the interrupt code is receiving a word,
; it must have time between words to perform its worst case processing so it can retrieve each
; new word before the interrupt overwrites it. Thus the "Main" PIC must have a significant
; delay between words.
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F648a	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

	errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
					 ;	(this is displayed when a RAM address above bank 1 is used -- it is
					 ;	 expected that the lower bits will be used as the lower address bits)


#INCLUDE <P16f648a.inc> 		; Microchip Device Header File

#INCLUDE <STANDARD.MAC>     	; include standard macros

; Specify Device Configuration Bits

	__CONFIG _INTOSC_OSC_CLKOUT & _WDT_OFF & _PWRTE_ON & _MCLRE_OFF & _BOREN_OFF & _LVP_OFF & _CPD_OFF & _CP_OFF

;_INTOSC_OSC_CLKOUT = uses internal oscillator, clock can be output on RA6
;_WDT_OFF = watch dog timer is off
;_PWRTE_ON = device will delay startup after power applied to ensure stable operation
;_MCLRE_OFF = RA5/MCLR/VPP pin function is digital input, MCLR internally to VDD
;_BOREN_ON = Brown Out Reset is on -- low supply voltage will cause device reset
;_LVP_OFF = RB4/PGM is digital I/O, HV on MCLR must be used for programming
;           (device cannot be programmed in system with low voltage)
;_CPD_OFF = data memory is not protected and can be read from the device
;_CP_OFF = code memory is not protected and can be read from the device
;
;for improved reliability, Watch Dog code can be added and the Watch Dog Timer turned on - _WDT_ON
;turn on code protection to keep others from reading the code from the chip - _CP_ON
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Hardware Definitions

LCD_CTRL        EQU     0x05		; PORTA
SERIAL_IN		EQU		0x00		; RA0 - serial data from Main PIC
LCD_E           EQU     0x01		; RA1 - data read/write strobe
LCD_RS			EQU		0x02		; RA2 - instruction/data register select
UNUSED1			EQU		0x03		; RA3 - no used in application -- useful for debugging output
UNUSED2			EQU		0X04		; RA4 - no used in application -- useful for debugging output

LCD_DATA        EQU     0x06		; PORTB

; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Constant Definitions
;

TIMER0_RELOAD_START_BIT_SEARCH	EQU	.255-.38		; interrupt every 32 us (32 cycles)
													; (half of the 64 uS (64 cycles) between serial bits)
													; see note "Serial Data from Main PIC" in this file
													; wasted cycles in interrupt not accounted for -- use
													; where this is not a factor
													; 38 is actual value used to take into account cycles
													; lost in interrupt and due to counter skips after load


TIMER0_RELOAD_START_BIT_SEARCH_Q	EQU	.255-.16
													; interrupt every 32 us (32 cycles)
													; (half of the 64 uS (64 cycles) between serial bits)
													; see note "Serial Data from Main PIC" in this file
													; 16 is actual value used to take into account cycles
													; lost in interrupt and due to counter skips after load

BIT_TO_BIT_LOOP_DELAY			EQU	.19				; used in decfsz loops to delay between serial bits
													; want 64 uS
													; 19 takes into account cycles used by bit read loop

BIT_TO_BIT_LOOP_DELAY_H			EQU	BIT_TO_BIT_LOOP_DELAY/.2
													; used in decfsz loops to delay after start bit
													; half of normal bit width to put timing into center
													; of first data bit

FINAL_BIT_LOOP_DELAY			EQU	.22				; used in decfsz loops to delay after the final bit
													; so it won't be seen as the next start bit -- it
													; is slightly longer than a full bit delay


CURSOR_BLINK_RATE		EQU .20			; controls how fast the character at the cursor location blinks

DISPLAY_ON_OFF_CMD_MASK	EQU 0xf8		; masks lower bits off on/off command to leave only the command type
DISPLAY_ON_OFF_CMD		EQU 0x08		; the upper bits which specify the command type

DISP_ON_CURSOR_OFF_BLINK_OFF_CMD	equ 0x0c	; command to turn display on, cursor off, blink off

BLINK_ON_OFF_CMD_FLAG	EQU	.0



; control bits in flags

UNUSED_FLAG_BIT0		EQU		.0
CHAR_AT_CURSOR_STATE	EQU		.1


CHAR_AT_CURSOR_STATE_XOR_MASK	EQU		0x02	; use to flip flag bit using XOR

; end of control bits in flags


ADDRESS_SET_BIT	EQU		.7		; set in LCD control codes to specify an address change byte

MAX_COLUMN      EQU     .19		; highest column number (20 columns)
PAST_MAX_COLUMN EQU		.20		; one past the highest column number
MAX_LINE		EQU		.3		; highest line number (4 lines)
PAST_MAX_LINE	EQU		.4		; one past the highest line number

; actual bytes to write to LCD to address the different columns
; see "LCD ADDRESSING NOTE" in header notes at top of page for addressing explanation

LCD_COLUMN0_START	EQU		0x80
LCD_COLUMN0_END		EQU		0x93
LCD_COLUMN1_START	EQU		0xc0
LCD_COLUMN1_END		EQU		0xd3
LCD_COLUMN2_START	EQU		0x94
LCD_COLUMN2_END		EQU		0xa7
LCD_COLUMN3_START	EQU		0xd4
LCD_COLUMN3_END		EQU		0xe7

LCD_BUFFER_SIZE		EQU		.80

; LCD Display Commands

CLEAR_SCREEN_CMD	EQU		0x01

; LCD Display On/Off Command bits

;  bit 3: specifies that this is a display on/off command if 1
;  bit 2: 0 = display off, 1 = display on
;  bit 1: 0 = cursor off, 1 = cursor on
;  bit 0: 0 = character blink off, 1 = blink on

DISPLAY_ONOFF_CMD_FLAG	EQU		0x08
DISPLAY_ON_FLAG			EQU		0x04
CURSOR_ON_FLAG			EQU		0x02
BLINK_ON_FLAG			EQU		0x01

; end of Constant Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in RAM
;
; Note that you cannot use a lot of the data definition directives for RAM space (such as DB)
; unless you are compiling object files and using a linker command file.  The cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
; 

; Assign variables in RAM - Bank 0 - must set RP0:RP1 to 0:0 to access
; Bank 0 has 80 bytes of free space

 cblock 0x20                ; starting address

    flags                   ; bit 0: unused
                            ; bit 1: 0 = char at cursor is off : 1 = char at cursor is on
                            ; bit 2:
                            ; bit 3:
                            ; bit 4:
                            ; bit 5:
							; bit 6:
							; bit 7:

	controlByte				; the first byte of each serial data byte pair is stored here
	lcdData					; stores data byte to be written to the LCD
	
	currentCursorLocation		; the current cursor location on the display; this is the
								; code which is sent to the display to set that location

	currentCursorBufferPosition ; the current buffer position of the last cursor location
								; specified by the "Main" PIC

	currentLCDOnOffState	; on/off state of the LCD along with cursor on/off and
							; blink on/off; this is code for the display to set those

	charBlinkRate			; delay to control blink rate of cursor at character location

	smallDelayCnt			; used to count down for small delay
	bigDelayCnt				; used to count down for big delay
	
	scratch0				; scratch pad variable
	scratch1				; scratch pad variable
	scratch2				; scratch pad variable

	; next variables ONLY written to by interrupt code

	intScratch0				; scratch pad variable for exclusive use by interrupt code
	bitCount				; used to count number of bits received
	newSerialByte			; each serial data byte is stored here upon being received
	newControlByte			; new control bytes stored here by interrupt routine
							; (this one is reset by main thread)
	newLCDData				; new data bytes stored here by interrupt routine

	; end of variables ONLY written to by interrupt code

	
    eepromAddress		    ; use to specify address to read or write from EEprom
    eepromCount	        	; use to specify number of bytes to read or write from EEprom

	debugCounter			; counts number of bytes saved to debug buffer
	debugPointer			; points to next address in debug buffer

	db0
	db1
	db2
	db3
	db4
	db5
	db6
	db7
	db8
	db9
	db10
	db11
	db12
	db13
	db14
	db15
	db16
	db17
	db18
	db20
	db21
	db22
	db23
	db24
	db25
	db26
	db27
	db28
	db29

 endc

;-----------------

; Assign variables in RAM - Bank 1 - must set RP0:RP1 to 0:1 to access
; Bank 1 has 80 bytes of free space

 cblock 0xa0                ; starting address

    lcdFlags                ; bit 0: 0 = not used, 1 = not used
                            ; bit 1: 
                            ; bit 2:
                            ; bit 3:
                            ; bit 4:

    lcdScratch0             ; scratch pad variables
    lcdScratch1

	lcdOutLine				; current line being written to the display
	lcdOutColumn			; current column to be written to the display
    lcdBufOutPtr			; read from buffer to write to LCD pointer


	lcdInColumn				; current column being written to in the buffer
	lcdBufInPtr				; write to buffer from master PIC pointer

 endc

;-----------------

; Assign LCD character buffer in RAM - Bank 2 - RP0:RP1 to 1:0 to access
; Bank 2 has 80 bytes of free space

; To access this bank (Bank 2) indirectly, STATUS:IRP must be set to 1

; This buffer is often accessed indirectly (using the FSR/INDF registers)
; The INDF register provides the lower 8 bits of the indirect address while the
; IRP bit in the STATUS register provides the 9th bit to allow access of any
; RAM location in any bank.

; LCD character buffer -- 4 lines x 20 characters each
; see "LCD ADDRESSING NOTE" in header notes at top of page for addressing explanation

; WARNING -- the buffer entirely fills Bank 2 -- do not add any variables to this bank
	
 cblock 0x120		; starting address

	; line 1

	lcd0			; LCD address 0x00 (send 0x80 to LCD with address control bit 7 set)
	lcd1
	lcd2
	lcd3
	lcd4
	lcd5
	lcd6
	lcd7
	lcd8
	lcd9
	lcd10
	lcd11
	lcd12
	lcd13
	lcd14
	lcd15
	lcd16
	lcd17
	lcd18
	lcd19

	; line 2

	lcd20				; LCD address 0x40 (send 0xc0 to LCD with address control bit 7 set)
	lcd21
	lcd22
	lcd23
	lcd24
	lcd25
	lcd26
	lcd27
	lcd28
	lcd29
	lcd30
	lcd31
	lcd32
	lcd33
	lcd34
	lcd35
	lcd36
	lcd37
	lcd38
	lcd39

	; line 3

	lcd40			; LCD address 0x14 (send 0x94 to LCD with address control bit 7 set)
	lcd41
	lcd42
	lcd43
	lcd44
	lcd45
	lcd46
	lcd47
	lcd48
	lcd49
	lcd50
	lcd51
	lcd52
	lcd53
	lcd54
	lcd55
	lcd56
	lcd57
	lcd58
	lcd59

	; line 4

	lcd60		; LCD address 0x54 (send 0xd4 to LCD with address control bit 7 set)
	lcd61
	lcd62
	lcd63
	lcd64
	lcd65
	lcd66
	lcd67
	lcd68
	lcd69
	lcd70
	lcd71
	lcd72
	lcd73
	lcd74
	lcd75
	lcd76
	lcd77
	lcd78
	lcd79

 endc

; WARNING -- the buffer entirely fills Bank 2 -- do not add any variables to this bank

;-----------------
 
; Define variables in the memory which is mirrored in all 4 RAM banks.  This area is usually used
; by the interrupt routine for saving register states because there is no need to worry about
; which bank is current when the interrupt is invoked.
; On the PIC16F628A, 0x70 thru 0x7f is mirrored in all 4 RAM banks.

; NOTE:
; This block cannot be used in ANY bank other than by the interrupt routine.
; The mirrored sections:
;
;	Bank 0		Bank 1		Bank 2		Bank3
;	70h-7fh		f0h-ffh		170h-17fh	1f0h-1ffh
;

 cblock	0x70
    W_TEMP
    FSR_TEMP
    STATUS_TEMP
    PCLATH_TEMP	
 endc

;-----------------

; end of Variables in RAM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in EEprom
;
; Assign variables in EEprom
;

 cblock 	0x0      	; Variables start in RAM at 0x0
	
	eeScratch0
    eeScratch1
	eeScratch2

 endc

; end of Variables in EEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Power On and Reset Vectors
;
; Note: handleInterrupt not used:
;
;  In most programs, handleInterrupt is called to determine the type of interrupt. The timing is
;  so tight in this program that the function is not used to save time in the interrupt when there
;  is no data to process. Since only the Timer0 interrupt is used, it is assumed that any
;  interrupt is a Timer0 interrupt. The serial input bit is checked and handleTimer0Interrupt is
;  called directly if it is low (a start bit).
;

	org 0x00                ; Start of Program Memory

	goto start              ; jump to main code section
	nop			            ; Pad out so interrupt
	nop			            ; service routine gets
	nop			            ; put at address 0x0004.

; interrupt vector at 0x0004
; NOTE: You must save values (PUSH_MACRO) and clear PCLATH before jumping to the interrupt
; routine - if PCLATH has bits set it will cause a jump into an unexpected program memory
; bank.

	PUSH_MACRO              ; MACRO that saves required context registers
	clrf	STATUS          ; set to known state
    clrf    PCLATH          ; set to bank 0 where the ISR is located

	movlw	TIMER0_RELOAD_START_BIT_SEARCH_Q
	movwf	TMR0
	bcf 	INTCON,T0IF     ; clear the Timer0 overflow interrupt flag

	; data bank 0 already selected by clearing STATUS above

	btfss	PORTA,SERIAL_IN
	goto	handleTimer0Interrupt

	POP_MACRO               		; MACRO that restores required registers

	retfie                  		; return and enable interrupts

	; handleInterrupt is skipped in this program -- see note above "Note: handleInterrupt not used."
    ;goto 	handleInterrupt	; points to interrupt service routine

; end of Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;

setup:

	; make sure both bank selection bits are set to Bank 0

	bcf	    STATUS,RP0	    ; back to Bank 0
    bcf     STATUS,RP1	    ; back to Bank 0

    clrf    INTCON          ; disable all interrupts

	movlw	0x07			; turn off comparator, PortA pins set for I/O                              
 	movwf	CMCON           ; 7 -> Comparator Control Register to disable comparator inputs

    movlw   0xff
    movwf   PORTA           ; 0xff -> Port A

 	movlw	0x00			                                
 	movwf	PORTB			; set Port B outputs low

 	bsf 	STATUS,RP0		; select bank 1

	movlw 	0x01                              
 	movwf 	TRISA			; 0x01 -> TRISA = PortA I/O 0000 0001 b (1=input, 0=output)
    						;	RA0 - Input : receives serial input data
							;	RA1 - Output: E strobe to initiate LCD R/W
							;	RA2 - Output: Register Select for LCD
							;	RA3 - Output: unused (pulled high for some reason)
							;	RA4 - Output: unused (pulled high for some reason)
							;	RA5 - Vpp for PIC programming, unused otherwise
							;	RA6 - unused (unconnected)
							;	RA7 - unused (unconnected)
 	movlw 	0x00
 	movwf	TRISB			; 0x00 -> TRISB = PortB I/O 0000 0000 b (1=input, 0=output)
							;	port B outputs data to the LCD display
							; 	RB6 is also used for programming the PIC
							; 	RB7 is also used for programming the PIC

	movlw	0x58			; set options 0101 1000 b
	movwf	OPTION_REG		; bit 7 = 0: PORTB pull-ups are enabled by individual port latch values
     						; bit 6 = 1: RBO/INT interrupt on rising edge
							; bit 5 = 0: TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
							; bit 4 = 1: TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
							; bit 3 = 1: PSA ~ Prescaler assigned to WatchDog; Timer 0 will be 1:1 with Fosc/4
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    000 = 1:2 scaling for Timer0 (if assigned to Timer0)
                            ; bit 0 = 0 :

 	bcf 	STATUS,RP0		; select bank 0                          
	
	movlw	TIMER0_RELOAD_START_BIT_SEARCH_Q
	movwf	TMR0

 	bcf		PORTA,LCD_E     ; set LCD E strobe low (inactive)
	bcf		PORTA,LCD_RS	; set LCD Register Select low (chooses instruction register)
	bsf		PORTA,UNUSED1	; set high to match pullup (unused)
	bcf		PORTA,UNUSED2   ; set high to match pullup (unused)

; enable the interrupts


	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enabe TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

	; make sure both bank selection bits are set to Bank 0

	bcf	    STATUS,RP0	    ; back to Bank 0
    bcf     STATUS,RP1	    ; back to Bank 0

	movlw	.0
	movwf	flags
	
	movlw	CURSOR_BLINK_RATE		; preset blink rate timer
	movwf	charBlinkRate

	movlw	LCD_COLUMN0_START
	movwf	currentCursorLocation

	movlw	DISP_ON_CURSOR_OFF_BLINK_OFF_CMD
	movwf	currentLCDOnOffState

	return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Main Code
;
; Sets up the PIC, the LCD, displays a greeting, then monitors the serial data input line from
; the main PIC for data and instructions to be passed on to the LCD display.
;

start:

	call	setup			; set up main variables and hardware

   	call    bigDelay		; should wait more than 15ms after Vcc = 4.5V
    
	call    initLCD

	bcf 	STATUS,RP0		; select bank 0

	call	setUpLCDCharacterBuffer

	call	clearLCDLocalBuffer

 	bcf 	STATUS,RP0		; select bank 0

	call	displayGreeting

	bcf 	STATUS,RP0		; select bank 0

	movlw	0xff			; 0xff in newControlByte means no new serial data
	movwf	newControlByte	; it is changed when a new value is received

	movlw	.30				; save 255 values
	movwf	debugCounter

	movlw	db0
	movwf	debugPointer


; begin monitoring the serial data input line from the main PIC for data and instructions
; to be passed on to the LCD display

; in between each check for incoming data on the serial line, write one character from the local
; LCD buffer to the LCD display

mainLoop:

	bcf		STATUS,RP0			; select bank 0

	movf	newControlByte,W	; if newControlByte is 0xff, then no new data to process
 	sublw	0xff
 	btfss	STATUS,Z
	call	handleDataFromMainPIC

	bcf		STATUS,RP0			; select bank 0

	call	writeNextCharInBufferToLCD ;write one character in the buffer to the LCD

    goto    mainLoop

; end of Main Code
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleDataFromMainPIC
;
; Processes the values in controlByte and lcdData.
;
; NOTE: This function must be called faster than the interrupt routine is reading consecutive
; words. The words have a delay between them which allows for more time between calls to this
; function.
;
; Control codes other than address changes and clear screen commands are written to the LCD
; display immediately. Address changes and data values are used to address and store in the local
; LCD character buffer.
;
; Data bank 0 should be selected on entry.
;

handleDataFromMainPIC:
	
	;copy the values to working variables so interrupt routine can use new variables
	
	movf	newControlByte,W
	movwf	controlByte
	movf	newLCDData,W
	movwf	lcdData

	movlw	0xff					; signal that the new data has been collected
	movwf	newControlByte

	; call to debug will cause communication to be corrupted with Main PIC when it comes time to
	; write to the eeprom as that disables interrupts for a while
	;call	debug	;debug mks -- store the controlByte and lcdData in eeprom

	movf	controlByte,W			; if the control byte is 0, then the second byte is data for the LCD
 	sublw	0
 	btfsc	STATUS,Z
    goto    writeToLCDBuffer		; store byte in the local LCD character buffer

	bcf		STATUS,RP0				; select bank 0
	movf	controlByte,W			; if the control byte is 1, then the second byte is an instruction for the LCD
 	sublw	0x1
 	btfsc	STATUS,Z
    goto    handleLCDInstruction

	return

; end of handleDataFromMainPIC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleLCDInstruction
;
; Handles LCD instruction codes received from the master PIC. If the control code is an address
; change or clear screen code, the command is directed to the local LCD character buffer. The LCD
; display itself is not changed -- that is handled by the code which transmits the buffer contents
; to the display.
;
; All other control codes are transmitted directly to the LCD display.
;
; Data bank 0 should be selected on entry.
;

handleLCDInstruction:

	; catch clear screen command

	movf	lcdData,W
 	sublw	CLEAR_SCREEN_CMD
 	btfss	STATUS,Z
	goto	notClearScreenCmd

	goto	clearLCDLocalBuffer

notClearScreenCmd:

	; check for address change instruction

    btfss   lcdData,ADDRESS_SET_BIT	
	goto	notAddressChangeCmd
	
	goto	setLCDBufferWriteAddress

notAddressChangeCmd:

	; check for display on/off instruction

	; mask lower bits; leave only type selection bits to compare with commmand

	movlw	DISPLAY_ON_OFF_CMD_MASK
	andwf	lcdData,W
	sublw	DISPLAY_ON_OFF_CMD
 	btfss	STATUS,Z
	goto	notDisplayCursorBlinkCmd

	goto	setLCDOnOffAndCursorAndBlink

notDisplayCursorBlinkCmd:

	; transmit all other control codes straight to the display

	goto    writeLCDInstruction

; end of handleLCDInstruction
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeNextCharInBufferToLCD
;
; Writes the next character in the current line to the LCD display. If the end of the line is
; reached, the line pointer is incremented to the next line.
;
; Bank selection not important on entry.
; STATUS:IRP is modified
;

writeNextCharInBufferToLCD:

	bsf     STATUS,RP0      ; select data bank 1 to access LCD buffer variables
    bsf     STATUS,IRP      ; use upper half of memory for indirect addressing of LCD buffer

    movf    lcdBufOutPtr,W  ; get pointer to next character to be written to LCD
    movwf   FSR             ; point indirect register FSR at the character    

	movf	INDF,W			; load the character

	bcf		STATUS,RP0		; select bank 0
	movwf	lcdData         ; store for use by writeLCDData function

	; check if cursor is blinking and is in the "off" state

	btfsc	currentLCDOnOffState,BLINK_ON_OFF_CMD_FLAG
	btfsc	flags,CHAR_AT_CURSOR_STATE
	goto	noHideCharacter

	; character should be off

	bsf		STATUS,RP0		; select bank 1
	movf	lcdBufOutPtr,W
	bcf		STATUS,RP0		; select bank 0
	subwf	currentCursorBufferPosition,W
	btfss	STATUS,Z
	goto	noHideCharacter

	movlw	' '
	movwf	lcdData         ; store for use by writeLCDData function	

noHideCharacter:
	
    call    writeLCDData

	bsf     STATUS,RP0      ; back to bank 1 
	call	incrementLCDOutBufferPointers

	return

; end of writeNextCharInBufferToLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; incrementLCDOutBufferPointers
;
; Increments the pointers used to track the next character to be written to the LCD from the
; character buffer -- the buffer location, line number, and column number are all incremented.
;
; When the last column is reached, the line number is incremented while the column rolls back to 0;
; when the last line is reached the line number rolls back to 0.
;
; Data bank 1 should be selected on entry.
;

incrementLCDOutBufferPointers:

	incf	lcdBufOutPtr,F	; point to next character in buffer

	incf	lcdOutColumn,F	; track column number
	movf	lcdOutColumn,W	; check if highest column number reached
 	sublw	PAST_MAX_COLUMN
 	btfss	STATUS,Z
    goto	noRollOver

	clrf	lcdOutColumn	; start over at column 0

	incf	lcdOutLine,F	; track line number
	movf	lcdOutLine,W	; check if highest line number reached
 	sublw	PAST_MAX_LINE
 	btfss	STATUS,Z
	goto	setLCDVariablesPerLineNumber    ; highest not reached

	; highest line number reached -- handle end of refresh tasks

	call	handleEndOfRefreshTasks

	bsf     STATUS,RP0      				; back to bank 1 
	clrf	lcdOutLine						; start over at line 0
	call	setLCDVariablesPerLineNumber

noRollOver:

	return

; end of incrementLCDOutBufferPointers
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleEndOfRefreshTasks
;
; Performs tasks required at the end of a refresh such as setting cursor and blink on/off states.
; 
; Note: The cursor and blink functions of the display are not used -- the PIC code handles those
; functions. See notes "Cursor and Blinking" in this file.
;
; Bank selection not important on entry.
;

handleEndOfRefreshTasks:

	bcf		STATUS,RP0			; select bank 0

	; refreshing the buffer moves the cursor location with each character sent
	; set the cursor location to the last location specified by the "Main" PIC

	movf	currentCursorLocation,W
	movwf	lcdData         	; store for use by writeLCDData function
    call    writeLCDInstruction

	; the cursor and blink are turned off at the start of each buffer refresh to eliminate
	; flicker as they follow each character sent; at the end of each refresh, set them
	; to the last state specified by the "Main" PIC

	; check if "Main" PIC has turned blinking on

	btfss	currentLCDOnOffState,BLINK_ON_OFF_CMD_FLAG
	goto	blinkIsOffHERT

	; blinking is on -- check if blink rate counter has timed out

	decfsz	charBlinkRate,F
	goto	blinkIsOffHERT

	movlw	CURSOR_BLINK_RATE		; reset blink rate timer
	movwf	charBlinkRate

	; timed out -- switch on/off states of character at cursor location

	movf	flags,W					; flip the character at cursor on/off state flag
	xorlw	CHAR_AT_CURSOR_STATE_XOR_MASK
	movwf	flags

blinkIsOffHERT:

	return

; end of handleEndOfRefreshTasks
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLCDVariablesPerLineNumber
;
; Sets the lcdBufOutPtr and the write address currently stored in the LCD display appropriate to
; the current buffer line number being written.
;
; Data bank 1 should be selected on entry.
;

setLCDVariablesPerLineNumber:

	movf	lcdOutLine,W	; handle line 0
 	sublw	0
 	btfss	STATUS,Z
    goto	notLine0

	movlw   lcd0			; start of line 0
    movwf   lcdBufOutPtr

	movlw	LCD_COLUMN0_START
	goto	writeLCDInstructionAndExit

notLine0:

	movf	lcdOutLine,W	; handle line 1
 	sublw	1
 	btfss	STATUS,Z
    goto	notLine1

	movlw   lcd20			; start of line 1
    movwf   lcdBufOutPtr

	movlw	LCD_COLUMN1_START
	goto	writeLCDInstructionAndExit

notLine1:

	movf	lcdOutLine,W	; handle line 2
 	sublw	2
 	btfss	STATUS,Z
    goto	notLine2

	movlw   lcd40			; start of line 2
    movwf   lcdBufOutPtr

	movlw	LCD_COLUMN2_START
	goto	writeLCDInstructionAndExit

notLine2:

	; don't check if line 3 -- any number not caught above is either 3 or illegal; if illegal then default
	; to line 3 to get things back on track

	movlw   lcd60			; start of line 3
    movwf   lcdBufOutPtr

	movlw	LCD_COLUMN3_START

writeLCDInstructionAndExit:

	bcf		STATUS,RP0				; select bank 0
	movwf	lcdData					; save set address instruction code for writing
    call    writeLCDInstruction		

	return

; end of setLCDVariablesPerLineNumber
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearLCDLocalBuffer
;
; Sets all data in the local LCD character buffer to spaces. The LCD display will be cleared
; when the local buffer is next transmitted to the display.
;
; Bank selection not important on entry.
; STATUS:IRP is modified     bsf     STATUS,IRP      ; use upper half of memory for indirect addressing of LCD buffer
;

clearLCDLocalBuffer:

   	bsf     STATUS,RP0      ; select data bank 1 to access LCD buffer variables
    bsf     STATUS,IRP      ; use upper half of memory for indirect addressing of LCD buffer

	movlw	LCD_BUFFER_SIZE	; set up loop counter
	movwf	lcdScratch0

	movlw	lcd0			; point indirect register FSR at buffer start
    movwf   FSR             

	movlw	' '				; fill with spaces

clearLCDLoop:
	
	movwf	INDF			; store to each buffer location
	incf	FSR,F
	decfsz	lcdScratch0,F
	goto	clearLCDLoop
	
	call	setUpLCDCharacterBuffer

	return

; end of clearLCDLocalBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setUpLCDCharacterBuffer
;
; Prepares the LCD character buffer for use.
;
; Bank selection not important on entry.
;

setUpLCDCharacterBuffer:

   	bsf     STATUS,RP0      ; select data bank 1 to access LCD buffer variables

	movlw   lcd0
	movwf	lcdBufInPtr		; set write to buffer pointer from master PIC to line 0 column 0

    clrf    lcdOutLine    	; start at line 0 for writing buffer to LCD
	clrf	lcdOutColumn	; start a column 0 for writing buffer to LCD

	call	setLCDVariablesPerLineNumber	; set up buffer out to LCD variables

	return

; end of setUpLCDCharacterBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLCDBufferWriteAddress
;
; Sets the LCD buffer write pointer according to the address in lcdData. This value is the
; control code that would be written to the LCD display to set an address. 
;
; see "LCD ADDRESSING NOTE" in header notes at top of page for addressing explanation
;
; Bank 0 should be selected on entry.
;

setLCDBufferWriteAddress:

	movf	lcdData,W		; load address control code from bank 0

	movwf	currentCursorLocation	; store as the cursor location for later use when the
									; display is refreshed

   	bsf     STATUS,RP0      ; select data bank 1 to access LCD buffer variables

	movwf	lcdScratch0		; store address control code in bank 1 for easy access

	call	getLCDLineContainingAddress	; find which line contains the specified address

	movf	lcdBufInPtr,W
 	bcf 	STATUS,RP0					; select bank 0
	movwf	currentCursorBufferPosition	; store as the cursor location for later use in making
										; the character at that location blink
	return

; end of setLCDBufferWriteAddress
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLCDOnOffAndCursorAndBlink
;
; Stores the state of the display on/off, cursor on/off and blink on/off. This command is not
; immediately transmitted to the display -- it will be applied the next time the display is
; refreshed.
;
; The "Main" PIC sets the cursor location and then turns blink on when it wants to
; highlight a character. This causes problems because the latest "LCD" PIC code refreshes
; the display by redrawing the entire screen -- when blink is activated glitches can be seen
; zipping across the screen as each character tries to blink as it is written.
;
; To solve this, the last cursor location and blink status from the "Main" PIC are stored.
; Blink is turned off during a refresh; at the end of each refresh cycle, the cursor is
; briefly positioned at the last location specified by the "Main" PIC and if blink has been
; set to "on" then it is turned back on to briefly highlight the selected location before
; the next refresh.
;
; Bank 0 should be selected on entry.
;

setLCDOnOffAndCursorAndBlink:

	movf	lcdData,W		; load address control code from bank 0

	movwf	currentLCDOnOffState	; store as the display on/off, cursor on/off, blink on/off
									; for use next time the display is refreshed

	return

; end of setLCDOnOffAndCursorAndBlink
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeToLCDBuffer
;
; Writes the byte in lcdData to the local LCD character buffer at memory location stored in
; lcdBufInPtr. Pointer lcdBufInPtr is then incremented.
;
; The number of characters written to each line is tracked via lcdInColumn. If the maximum
; number of characters has been stored for a line, all further attempts to write will be ignored
; until the address is reset.
;
; Bank 0 should be selected on entry.
; STATUS:IRP is modified
;

writeToLCDBuffer:

	movf	lcdData,W		; get the byte to be stored

   	bsf     STATUS,RP0      ; select data bank 1 to access LCD buffer variables
    bsf     STATUS,IRP      ; use upper half of memory for indirect addressing of LCD buffer

	movwf	lcdScratch0		; store byte in bank 1 for easy access

	movf	lcdInColumn,W	; bail out if already one past the max column number
 	sublw	PAST_MAX_COLUMN	
 	btfsc	STATUS,Z	
	return

	incf	lcdInColumn,f	; track number of bytes written to the line
	
    movf    lcdBufInPtr,W  	; get pointer to next memory location to be used
    movwf   FSR             ; point FSR at the character    

	movf	lcdScratch0,W	; retrieve the byte and store it in the buffer
    movwf	INDF

    incf   lcdBufInPtr,F	; increment the pointer

	return

; end of writeToLCDBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; getLCDLineContainingAddress
;
; Returns in the W register the line containing the address specified by the control code in
; lcdScratch0. The control byte is the value which would be sent to the LCD display to set the
; address.
;
; The lcdBufInPtr will be set to the proper memory location for storing at the specified address.
;
; An illegal address outside the range of any line defaults to line 3.
;
; see "LCD ADDRESSING NOTE" in header notes at top of page for addressing explanation
;
; Bank 1 should be selected on entry.
;
; REMEMBER: Borrow flag is inverse: 0 = borrow, 1 = no borrow
;

getLCDLineContainingAddress:

	; check for address any where on line 0 (between *_START and *_END

	movlw	LCD_COLUMN0_START	; compare address with *_START
    subwf	lcdScratch0,W		; address >= *_START?
    btfss   STATUS,C			; c = 0 = borrow = address<*_START
    goto	notLine0_GL

	movf	lcdScratch0,W		; compare address	
	sublw	LCD_COLUMN0_END		; address <= *_END?
    btfss   STATUS,C			; c = 0 = borrow = address>*_END
    goto	notLine0_GL

	movlw	LCD_COLUMN0_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd0				; add column to the line start's memory location
	movwf	lcdBufInPtr			; to get the address's memory location

	movlw	0					; the address is in line 0
	return

notLine0_GL:

	movlw	LCD_COLUMN1_START	; compare address with *_START
    subwf	lcdScratch0,W		; address >= *_START?
    btfss   STATUS,C			; c = 0 = borrow = address<*_START
    goto	notLine1_GL

	movf	lcdScratch0,W		; compare address	
	sublw	LCD_COLUMN1_END		; address <= *_END?
    btfss   STATUS,C			; c = 0 = borrow = address>*_END
    goto	notLine1_GL

	movlw	LCD_COLUMN1_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd20				; add column to the line start's memory location
	movwf	lcdBufInPtr			; to get the address's memory location

	movlw	1					; the address is in line 1
	return

notLine1_GL:

	movlw	LCD_COLUMN2_START	; compare address with *_START
    subwf	lcdScratch0,W		; address >= *_START?
    btfss   STATUS,C			; c = 0 = borrow = address<*_START
    goto	notLine2_GL

	movf	lcdScratch0,W		; compare address	
	sublw	LCD_COLUMN2_END		; address <= *_END?
    btfss   STATUS,C			; c = 0 = borrow = address>*_END
    goto	notLine2_GL

	movlw	LCD_COLUMN2_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd40				; add column to the line start's memory location
	movwf	lcdBufInPtr			; to get the address's memory location

	movlw	2					; the address is in line 2
	return

notLine2_GL:

	; all addresses not caught so far returned as line 3
	; illegal addresses end up here as well and default to line 3

	movlw	LCD_COLUMN3_START	; calculate the buffer index for the address
	subwf	lcdScratch0,W		; by finding the column number first by
								; subtracting the line's start address
	movwf	lcdInColumn			; store the column
	addlw	lcd60				; add column to the line start's memory location
	movwf	lcdBufInPtr			; to get the address's memory location

	movlw	3					; the address is in line 3
	return

; end of getLCDLineContainingAddress
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayGreeting
;
; Displays a greeting string, version info, etc.
;
; The text is written to the local LCD character buffer so it will be transmitted to the display.
;
; Bank 0 should be selected on entry.
;
; wip mks -- convert this to the write string method used in "OPT EDM Main PIC.asm"
;

displayGreeting:

	movlw	0x80			; move cursor to line 1 column 1 (address 0x00 / code 0x80)
	movwf	lcdData         ;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'O'				; display "OPT EDM" on the first line
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'P'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'T'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	' '
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'E'                             
	movwf 	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'D'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'M'                     
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	0xc1			; move cursor to line 2 column 2 (address 41h)
	movwf	lcdData         ;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'N'				; display "Notcher" on the second line
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'o'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	't'                            
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'c'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'h'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'e'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'r'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	0x96			; move cursor to line 3 column 7 (address 16h)
	movwf	lcdData			;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'b'				; display "by CMP" on the third line
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'y'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	' '  
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'M'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'K'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'S'
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	0xd7			; move cursor to line 4 column 8 (address 57h)
	movwf	lcdData			;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'R'				; display "Rev 2.7" on the fourth line
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'e'                            
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'v'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	' '                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'2'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

 	movlw	'.'                             
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	movlw	'1'
	movwf	lcdData                             
    call    writeToLCDBuffer
 	bcf 	STATUS,RP0		; select bank 0

	return

; end of displayGreeting
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; initLCD
;
; Initialize the LCD display.
;
; See Dmcman_full-user manual.pdf from www.optrex.com for details.
;

initLCD:

	bcf		LCD_CTRL,LCD_E		; LCD E strobe low                          
	bcf		LCD_CTRL, LCD_RS    ; LCD RS low (instruction register selected)                       
    call    smallDelay			; wait a bit

	movlw	0x30				; 1st send of Function Set Command: (8-Bit interface)(BF cannot be checked before this command.)
	movwf	LCD_DATA			; prepare to write                   
    call    strobeE				; write to LCD
    call    bigDelay			; should wait more than 4.1ms

	movlw	0x30				; 2nd send of Function Set Command: (8-Bit interface)(BF cannot be checked before this command.)
	movwf	LCD_DATA			; prepare to write                   
    call    strobeE				; write to LCD
    call    smallDelay			; should wait more than 100us

	movlw	0x30				; 3rd send of Function Set Command: (8-Bit interface)(BF can be checked after this command)
	movwf	LCD_DATA			; prepare to write	(BF busy flag cannot be checked on this board because R/W line is tied low)
    call    strobeE				; write to LCD
    call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board

	movlw	0x38				; write 0011 1000 Function Set Command ~ multi line display with 5x7 dot font
	movwf	LCD_DATA			;  0011 in upper nibble specifies Function Set Command
    call    strobeE				;  bit 3: 0 = 1 line display, 1 = multi-line display
								;  bit 2: 0 = 5x7 dot font, 1 = 5 x 10 dot font
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board

	movlw	0x0c				; write 0000 1000 ~ Display Off
	movwf	LCD_DATA			;  bit 3: specifies display on/off command
    call    strobeE				;  bit 2: 0 = display off, 1 = display on
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board
								; NOTE: LCD user manual instructs to turn off display here with 0x08
								;  but this did NOT work. Unknown why.

	movlw	0x01				; write 0000 0001 ~ Clear Display
	movwf	LCD_DATA
    call    strobeE
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board
								; NOTE: clear display added by MKS to match suggested setup in LCD user manual

	movlw	0x06				; write 0000 0110 ~ Entry Mode Set, increment mode, no display shift
	movwf	LCD_DATA			; bits 3:2 = 0:1 : specifies Entry Mode Set
    call    strobeE				; bit 1: 0 = no increment, 1 = increment mode; bit 0: 0 = no shift, 1 = shift display
	call    smallDelay			; wait for a bit because BF (busy flag) cannot be checked on this board
								; NOTE: Entry Mode Set was being done after display on -- moved by MKS to match
								;		suggested setup in LCD user manual.
								; NOTE2: See above regarding not working when display turned off before this --
								; does display need to be on when this command given regardless of LCD manual
								; suggestions?

	movlw	0x0c				; write 0000 1100 ~ Display On, cursor off, blink off
	movwf	LCD_DATA			;  bit 3: specifies display on/off command
    call    strobeE				;  bit 2: 0 = display off, 1 = display on
								;  bit 1: 0 = cursor off, 1 = cursor on
								;  bit 0: 0 = blink off, 1 = blink on

; Note: BF should be checked before each of the instructions starting with Display OFF.
;   Since this board design does not allow BF (LCD busy flag) to be checked, a delay is inserted after each
;	instruction to allow time for completion.

    call    bigDelay

	return

; end of initLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeLCDData
;
; Writes a data byte from variable lcdData to the LCD.  The data is a character to be displayed.
;
; Data bank 0 should be selected on entry.
;
                                 
writeLCDData:

	bcf		LCD_CTRL,LCD_E			; init E to low
	bsf		LCD_CTRL,LCD_RS			; select data register in LCD

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	movf	lcdData,W				; place data on output port
	movwf	LCD_DATA                          
    call    strobeE					; write the data

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	return

; end of writeLCDData
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeLCDInstruction
;
; Writes an instruction byte from variable lcdData to the LCD.  An instruction is one of several
; codes recognized by the LCD display for clearing the screen, moving the cursor, etc.
;
; Data bank 0 should be selected on entry.
;
                                 
writeLCDInstruction:

	bcf 	LCD_CTRL,LCD_E			; init E to low  
	bcf 	LCD_CTRL,LCD_RS			; select instruction register in LCD

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	movf 	lcdData,W				; place instruction on output port
	movwf	LCD_DATA                         
    call    strobeE					; write the instruction
	bsf		LCD_CTRL,LCD_RS			; set the instruction/data register select back to data register

;debug mks
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	return

; end of writeLCDInstruction
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; smallDelay
;
; Creates a small delay.
;
                                 
smallDelay:
	movlw	0x2a                             
	movwf	smallDelayCnt
                             
L8b:
	decfsz	smallDelayCnt,F                         
    goto    L8b
	return

; end of smallDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; bigDelay
;
; Creates a big delay.
;
                                 
bigDelay:

	movlw	0x28                             
	movwf	bigDelayCnt                             
    call	smallDelay

L9b:
	decfsz	bigDelayCnt,F                         
    goto    L9b
	return

; end of bigDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; strobeE
;
; Strobes LCD E line to read/write data to the LCD and then delays for a bit.
;

strobeE:

	bsf		LCD_CTRL,LCD_E
	nop                                    
	nop
	nop
	nop
	nop
	bcf		LCD_CTRL,LCD_E

;debug mks    
;    call    smallDelay
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	return

; end of strobeE
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readFromEEprom
; 
; Read block of bytes from EEprom.
;
; Address in EEprom from which to read first byte should be in eepromAddress.
; Number of bytes to read should be in eepromCount.
; Indirect register (FSR) should point to first byte in RAM to be written into.
; The block of bytes will be copied from EEprom to RAM.
;
; NOTE: This function must be modified for use with PIC16F876 - look for
;  notes in the function referring to PIC16F876 for details
; 
; STATUS:IRP is modified
;

readFromEEprom:

loopRFE1:

	movf	eepromAddress,W	; load EEprom address from which to read
	incf	eepromAddress,F	; move to next address in EEprom	
	    
    bcf     STATUS,IRP      ; use lower half of memory for indirect addressing of debug buffer

    bsf	    STATUS,RP0		; select bank 1 [for PIC16F648]
    ;bsf    STATUS,RP1		; select bank 2 [for PIC16F876]
	
    movwf	EEADR			; place in EEprom read address register

	;bsf    STATUS,RP0		; select bank 3 [for PIC16F876]
	;bcf    EECON1,EEPGD	; read from EEprom (as opposed to Flash) [for PIC16F876]
	bsf	    EECON1,RD		; perform EEProm read

	;bcf	STATUS,RP0	    ; select bank 2 [for PIC16F876]
	movf	EEDATA,W		; move data read into w
	movwf   INDF			; write to RAM
	incf	FSR,F			; move to next address in RAM

	bcf	STATUS,RP0		    ; select bank 0
	bcf	STATUS,RP1

	decfsz	eepromCount,F	; count down number of bytes transferred
	goto	loopRFE1		; not zero yet - read more bytes

	return

; end of readFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeToEeprom
;
; Writes block of bytes to EEprom.
;
; Address in EEprom to store first byte should be in eepromAddress.
; Number of bytes to write should be in eepromCount.
; Indirect register (FSR) should point to first byte in RAM to be written.
; The block of bytes will be copied from RAM to EEprom.
;
; STATUS:IRP is modified
;

writeToEEprom:

loopWTE1:	
    
    bcf     STATUS,IRP      ; use lower half of memory for indirect addressing of debug buffer

    movf	eepromAddress,W	; load EEprom address at which to store
	incf	eepromAddress,F	; move to next address in EEprom	

    bsf	    STATUS,RP0		; select bank 1 [for PIC16F648]
    ;bsf    STATUS,RP1		; select bank 2 [for PIC16F876]
	
	movwf	EEADR			; place in EEprom write address register

	movf	INDF,W			; get first byte of block from RAM
	incf	FSR,F			; move to next byte in RAM
	movwf	EEDATA			; store in EEprom write data register

	;bsf	STATUS,RP0		; select bank 3 [for PIC16F876]
    ;bcf	EECON1,EEPGD	; write to EEprom (as opposed to Flash) [for PIC16F876]
	bsf	    EECON1,WREN		; enable EEprom write
	bcf	    INTCON,GIE		; disable all interrupts
	
	movlw	0x55
	movwf	EECON2			; put 0x55 into EECON2
	movlw	0xaa
	movwf	EECON2			; put 0xaa into EECON2
	bsf	    EECON1,WR	  	; begin the write process

waitWTE1:	
    
    btfsc	EECON1,WR		; loop until WR bit goes low (write finished)
	goto	waitWTE1

	bcf	EECON1,WREN		    ; disable writes
	bsf	INTCON,GIE		    ; re-enable interrupts

	bcf	STATUS,RP0		    ; select bank 0
	bcf	STATUS,RP1

	decfsz	eepromCount,F	; count down number of bytes transferred
	goto	loopWTE1        ; not zero yet - write more bytes

	return

; end of writeToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; debug
;
; Stores several bytes (number specified by debugCounter) in eeprom for later viewing.
;
; Specified number of bytes in debugCounter are first stored in memory then copied to eeprom
; all at once. This eliminates the lengthy time delay for writing to eeprom being inserted into
; program flow -- the delays all happen at the end of the capture when the block is written to
; eeprom.
;
; STATUS:IRP is modified
;
                                 
debug:
	
    bcf     STATUS,IRP      ; use lower half of memory for indirect addressing of debug buffer

	bcf		STATUS,RP0				; select bank 0

	movf	debugCounter,W			; if counter is zero, do not store any more
 	sublw	0
 	btfsc	STATUS,Z
	goto	storeDebugDataInEEprom

	decf	debugCounter,F

    movf    debugPointer,W
    movwf   FSR           

	movf	controlByte,W
	movwf	INDF
	incf	FSR,F

	movf	lcdData,W
	movwf	INDF
	incf	FSR,F

	movf	FSR,W
	movwf	debugPointer

	return

storeDebugDataInEEprom:

	movf	debugPointer,W			; if pointer has been set to zero, already stored to eeprom so skip
 	sublw	0
 	btfsc	STATUS,Z
	return

	movlw	.0
	movwf	debugPointer

;eepromAddress set to zero by setup code -- will get incremented by each
;call to writeToEEprom

	movlw	eeScratch0		; beginning of storage buffer in eeprom
	movwf	eepromAddress

    movlw   db0        		; address in RAM
    movwf   FSR

    movlw   .30
    movwf   eepromCount     ; write 1 byte

    call    writeToEEprom

	return

debugFreeze:

	goto	debugFreeze

; end of debug
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInterrupt
;
; NOT USED -- handleInterrupt is skipped in this program -- see note in this file:
;	 "Note: handleInterrupt not used."
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;
; Data bank 0 should be selected on entry.
;

handleInterrupt:

	btfsc 	INTCON,T0IF     		; Timer0 overflow interrupt?
	goto 	handleTimer0Interrupt	; YES, so process Timer0
           
; Not used at this time to make interrupt handler as small as possible.
;	btfsc 	INTCON, RBIF      		; NO, Change on PORTB interrupt?
;	goto 	portB_interrupt       	; YES, Do PortB Change thing

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

endISR:

	POP_MACRO               		; MACRO that restores required registers

	retfie                  		; Return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Interrupt
;
; This function is called when the Timer0 register overflows.
;
; Data bank 0 should be selected on entry.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;
; NOTE: Because the main thread does not execute while the interrupt code is receiving a word,
; it must have time between words to perform its worst case processing so it can retrieve each
; new word before the interrupt overwrites it. Thus the "Main" PIC must have a significant
; delay between words.
;

handleTimer0Interrupt:

; if start bit was caught at the very leading edge, need to delay a bit
; to catch first data bit a little after its leading edge; if the
; start bit was caught a bit late, this shouldn't be enough to push
; the timing too late 

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

; read the control byte
                                 
	movlw	0x8					; preset bit counter to 8 bits to make a byte
	movwf	bitCount

readControlByteLoop:

	movlw	BIT_TO_BIT_LOOP_DELAY	; delay between bits
	movwf	intScratch0
bitLoop1: 	
	decfsz	intScratch0,F
    goto    bitLoop1
	
	movf	PORTA,W				; get Port A to get bit 0 (the serial data input)
	movwf	intScratch0			; save it so rrf can be performed
	rrf		intScratch0,F		; rotate bit 0 into the Carry bit                            
	rlf		newSerialByte,F		; rotate the Carry bit into the new data byte being constructed
      
 	decfsz	bitCount,F			; loop for 8 bits                         
    goto    readControlByteLoop

	movf	newSerialByte,W		; store in newControlByte where main thread will detect that it has
	movwf	newControlByte		; been changed from 0xff
								; (since the control byte and the data byte are read in the
								;  interrupt before returning, no worry about main thread seeing
								;  control byte change before data byte read)

; delay to get past the final bit

	movlw	FINAL_BIT_LOOP_DELAY	; delay a bit extra to get past last bit so it isn't detected
	movwf	intScratch0				; as the next start bit
finalBitLoop1: 	
	decfsz	intScratch0,F
    goto    finalBitLoop1

waitForStartBitLoop1:
 	btfsc	PORTA,SERIAL_IN			; loop until next start bit detected
    goto    waitForStartBitLoop1

; wait 1/2 bit width after start bit to put timing into center of first data bit

	movlw	BIT_TO_BIT_LOOP_DELAY_H	; 1/2 delay between bits
	movwf	intScratch0
bitLoop2: 	
	decfsz	intScratch0,F
    goto    bitLoop2

; read the data byte

	movlw	0x8					; preset bit counter to 8 bits to make a byte
	movwf	bitCount

readDataByteLoop:

	movlw	BIT_TO_BIT_LOOP_DELAY	; delay between bits
	movwf	intScratch0
bitLoop3: 	
	decfsz	intScratch0,F
    goto    bitLoop3
	
	movf	PORTA,W				; get Port A to get bit 0 (the serial data input)
	movwf	intScratch0			; save it so rrf can be performed
	rrf		intScratch0,F		; rotate bit 0 into the Carry bit                            
	rlf		newSerialByte,F		; rotate the Carry bit into the new data byte being constructed
      
 	decfsz	bitCount,F			; loop for 8 bits                         
    goto    readDataByteLoop

	movf	newSerialByte,W		; store in newLCDData where main thread will use it after
	movwf	newLCDData			; detecting that newControlByte has been set

; no need to delay for final bit -- won't be looked at until next interrupt

; reset the timer to a full count to avoid the time wasted in the interrupt

	movlw	TIMER0_RELOAD_START_BIT_SEARCH
	movwf	TMR0
	bcf 	INTCON,T0IF     	; clear the Timer0 overflow interrupt flag

	POP_MACRO               	; MACRO that restores required registers

	retfie                  	; Return and enable interrupts

; end of handleTimer0Interrupt
;--------------------------------------------------------------------------------------------------
 
    END
