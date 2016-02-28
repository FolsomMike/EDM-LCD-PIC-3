;--------------------------------------------------------------------------------------------------
; Project:  OPT EDM Notch Cutter -- LCD PIC software Model 3
; Date:     2/06/16
; Revision: 1.0
;
; This code is for the LCD controller PIC on the EDM Notch Cutter User Interface Board.
;
; Overview:
;
; This program reads serial data sent by the Main PIC and displays it on the LCD. All data is
; first stored in a local buffer which is then repeatedly transmitted to the LCD display. This
; constant refreshing corrects errors which occur in the displayed text due to electrical noise
; from the cutting current causing spikes in the LCD display control lines.
;
;--------------------------------------------------------------------------------------------------
;
; Revision History:
;
; 1.0   Code base copied from "OPT EDM LCD PIC" code.
;
;--------------------------------------------------------------------------------------------------
;
;--------------------------------------------------------------------------------------------------
; LCD Notes for the  EDM Notch Cutter User Interface Board
;
; Optrex C-51847NFJ-SLW-ADN 20 characters by 4 lines
;
; The user manual specified for this display is Dmcman_full-user manual.pdf from www.optrex.com
; This manual does not list this exact part number, but seems to be the appropriate manual.
;
; The R/W line on pin RA4 is driven low to write to the LCD, high to read.
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
; The LCD Data Buffer in the PIC
;
; The LCD's buffer is mirrored in a buffer in the PIC. This allows the LCD to be constantly
; refreshed from the PIC buffer to correct errors. The PIC LCD buffer is a contiguous block of
; memory, unlike the LCD's which has a chopped up address spacing (see above for details).
; Function(s) are included in this program for finding the PIC buffer position which corresponds
; to a specified LCD screen address.
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
; Serial Data Timing
;
; Each transmission is a packet starting with two header bytes (0xaa,0x55), a packet length byte
; (length of all data bytes plus checksum byte...the packet command byte is the first data byte).
;
; Upon detecting a complete packet, the main loop saves a copy of the length byte and immediately
; releases the receive buffer so a new packet can be received by the interrupt routine. If a new
; packet begins arriving immediately, it can overwrite the two header bytes with no conflict as
; they are always the same anyway. It can overwrite the length byte so long as the main code has
; already saved the copy. It can overwrite the command byte if the main code has already parsed it.
;
; In this manner, the interrupt code can begin overwriting the buffer with a new packet before the
; main code is completely finished with the previous one.
;
; The main code must execute handleSerialPacket before the buffer is released for the next packet.
; The time allowed for that is the delay between packets plus the transmission time for the first
; header byte:
;
;   delay + 10 bits * 17 uS = delay + 170 uS or delay + 340 * 4 = delay + 680 opcode cycles
;        for Fosc = 16Mhz
;
; If a new packet begins to arrive before the main code can execute handleSerialPacket, the new
; packet will be ignored.
;
; At 57,600 baud, the transmission time for the two header bytes is:
;   2 bytes * 10 bits * 17 uS = 340 uS or 340 * 4 = 1360 opcode cycles for Fosc = 16Mhz
;
; Thus the main code has 1360 instruction cycles to at least retrieve the length byte. From there
; it just needs to process the data in the packet faster than the interrupt can overwrite it.
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F1459	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

	errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
					 ;	(this is displayed when a RAM address above bank 1 is used -- it is
					 ;	 expected that the lower bits will be used as the lower address bits)


#INCLUDE <p16f1459.inc> 		; Microchip Device Header File

; Specify Device Configuration Bits

; CONFIG1
; __config 0xF9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_DISABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_INTOSC -> internal oscillator, I/O function on CLKIN pin
; _WDTE_OFF -> watch dog timer disabled
; _PWRTE_OFF -> Power Up Timer disabled
; _MCLRE_OFF -> MCLR/VPP pin is digital input
; _CP_OFF -> Flash Program Memory Code Protection off
; _BOREN_OFF -> Power Brown-out Reset off
; _CLKOUTEN_OFF -> CLKOUT function off, I/O or oscillator function on CLKOUT pin
; _IESO_OFF -> Internal/External Oscillator Switchover off
;   (not used for this application since there is no external clock)
; _FCMEN_OFF -> Fail-Safe Clock Monitor off
;   (not used for this application since there is no external clock)
; _WRT_ALL -> Flash Memory Self-Write Protection on -- no writing to flash
;
; _CPUDIV_NOCLKDIV -> CPU clock not divided
; _USBLSCLK_48MHz -> only used for USB operation
; _PLLMULT_4x -> sets PLL (if enabled) multiplier -- 4x allows software override
; _PLLEN_DISABLED -> the clock frequency multiplier is not used
;
; _STVREN_ON -> Stack Overflow/Underflow Reset on
; _BORV_LO -> Brown-out Reset Voltage Selection -- low trip point
; _LPBOR_OFF -> Low-Power Brown-out Reset Off
; _LVP_OFF -> Low Voltage Programming off
;
;for improved reliability, Watch Dog code can be added and the Watch Dog Timer turned on - _WDT_ON
;turn on code protection to keep others from reading the code from the chip - _CP_ON
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Defines
;

; COMMENT OUT "#define debug 1" line before using code in system.
; Defining debug will insert code which simplifies simulation by skipping code which waits on
; stimulus and performing various other actions which make the simulation run properly.
; Search for "ifdef debug" to find all examples of such code.

;#define debug 1     ; set debug testing "on"

; end of Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions
;

;  bits in flags variable

UNUSED_FLAG_BIT0		EQU		.0
CHAR_AT_CURSOR_STATE	EQU		.1

; bits in flags2 variable

HEADER_BYTE_1_RCVD  EQU 0
HEADER_BYTE_2_RCVD  EQU 1
LENGTH_BYTE_VALID   EQU 2
SERIAL_PACKET_READY EQU 3

; bits in statusFlags variable

SERIAL_COM_ERROR    EQU 0
I2c_COM_ERROR       EQU 1

SERIAL_RCV_BUF_LEN  EQU .64

SERIAL_XMT_BUF_LEN  EQU .10

; Serial Port Packet Commands

NO_ACTION_CMD               EQU .0
ACK_CMD                     EQU .1
SET_OUTPUTS_CMD             EQU .2
SWITCH_STATES_CMD           EQU .3
LCD_DATA_CMD                EQU .4
LCD_INSTRUCTION_CMD         EQU .5
LCD_BLOCK_CMD               EQU .6

; end of Software Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RA0   I/*,IOC,USB-D+                  ~ In ~ unused, pulled high on board version 1.1 and up
; RA1   I/*,IOC,USB-D-                  ~ In ~ unused, pulled high
; RA2   not implemented in PIC16f1459   ~ 
; RA3   I/*,IOC,T1G,MSSP-SS,Vpp,MCLR    ~ In ~ Vpp
; RA4   I/O,IOC,T1G,CLKOUT,CLKR, AN3    ~ Out ~ LCD R/W
; RA5   I/O,IOC,T1CKI,CLKIN             ~ Out ~ LCD RS (register select)
; RA6   not implemented in PIC16f1459
; RA7   not implemented in PIC16f1459
;
; On version 1.0, RA0 is connected to Serial_Data_To_Local_PICs and RB5 is connected to the
; LCD E Strobe. Those boards are modified with jumpers so that the EUSART RX on RB5 can be used to
; read serial data. The E Strobe cannot be switched to RA0 since that pin can only be an input on
; the PIC16F1459.  Those boards are modified with jumpers to connect RA0 and RB5 to feed the serial
; input into RB5, disconnect RB5 from the E Strobe, and connect RB6 to the E Strobe. RB6 is the 
; I2CSCL line, but it could still be used by setting by setting the Port C 
;
; Port B        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RB0   not implemented in PIC16f1459
; RB1   not implemented in PIC16f1459
; RB2   not implemented in PIC16f1459
; RB3   not implemented in PIC16f1459
; RB4   I/O,IOC,MSSP-SDA/SDI,AN10       ~ I ~ I2CSDA, I2C bus data line
; RB5   I/O,IOC,EUSART-RX/DX,AN11       ~ I ~ EUSART-RX, serial port data in
; RB6   I/O,IOC,MSSP-SCL/SCK            ~ Out ~ I2CSCL, I2C bus clock line ~ LCD E strobe
; RB7   I/O,IOC,EUSART-TX/CK            ~ Out ~ EUSART-TX, serial port data out
;
; Port C        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RC0   I/O,AN4,C1/2IN+,ICSPDAT,Vref    ~ In/Out ~ ICSPDAT ~ LCD D0
; RC1   I/O,AN5,C1/2IN1-,ICSPCLK,INT    ~ In/Out ~ ICSPCLK ~ LCD D1
; RC2   I/O,AN6,C1/2IN2-,DACOUT1        ~ In/Out ~ LCD D2
; RC3   I/O,AN7,C1/2IN3-,DACOUT2,CLKR   ~ In/Out ~ LCD D3
; RC4   I/O,C1/2OUT                     ~ In/Out ~ LCD D4
; RC5   I/O,T0CKI,PWM1                  ~ In/Out ~ LCD D5
; RC6   I/O,AN8,PWM2,MSSP-SS            ~ In/Out ~ LCD D6
; RC7   I/O,AN9,MSSP-SDO                ~ In/Out ~ LCD D7
;
;end of Hardware Control Description
;--------------------------------------------------------------------------------------------------
               
;--------------------------------------------------------------------------------------------------
; Hardware Definitions
;
; The ports are defined with suffix _P while the latches are defined with _L.
; Read from the ports (movf, btfss, btfsc, etc.).
; Write to the latches (movw, bcf, bsf, etc.)
;
; * Note: The LCD E pin is on RB6, the I2CSCL line. If the I2C bus is to be used to communicate
; with the Switch PIC, then during communication PORTC should be set as input and the RW line set
; high so when the LCD sees pulses on the E line due to I2CSCL clocking, data will be read from the
; LCD instead of random stuff being written. The LCD PIC must be the I2C master for this to work.
; The Switch PIC will ignore clocking on the I2CSCL line caused by strobing of the E line as the
; I2CSDA line must also be flipped to initiate an I2C communication.
;

LCD_RW_RS_L     EQU     LATA
LCD_E_L         EQU     LATB
LCD_DATA_OUT_L  EQU     LATC
LCD_DATA_IN_P   EQU     PORTC

LCD_RW          EQU     RA4         ; data read/write mode control
LCD_RS          EQU     RA5         ; instruction/data register select
SERIAL_IN       EQU     RB5         ; EUSART receive pin
LCD_E           EQU     RB6         ; data strobe to LCD *see note in header above
SERIAL_OUT      EQU     RB7         ; EUSART transmit pin

; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Constant Definitions
;

; Timer0 reload values shown here assume that the Timer0 prescaler is configured with a 4:1 ratio
; for 16Mhz Fosc.

; .255-.38 for 4Mhz
TIMER0_RELOAD_START_BIT_SEARCH	EQU	.255-.60		; interrupt every 32 us (32 cycles)
													; (half of the 64 uS (64 cycles) between serial bits)
													; see note "Serial Data from Main PIC" in this file
													; wasted cycles in interrupt not accounted for -- use
													; where this is not a factor
													; 38 is actual value used to take into account cycles
													; lost in interrupt and due to counter skips after load

; .255-.16 for 4Mhz
TIMER0_RELOAD_START_BIT_SEARCH_Q	EQU	.255-.29    ; interrupt every 32 us (32 cycles)
													; (half of the 64 uS (64 cycles) between serial bits)
													; see note "Serial Data from Main PIC" in this file
													; 16 is actual value used to take into account cycles
													; lost in interrupt and due to counter skips after load


FOSC_IN_MHZ                     EQU .16             ; Set this to match the Fosc frequency

INSTRUCTION_RATE                EQU FOSC_IN_MHZ/.4

BIT_TO_BIT_LOOP_DELAY			EQU	.21*INSTRUCTION_RATE	; used in decfsz loops to delay between serial bits
                                                            ; want 64 uS
                                                            ; 19 takes into account cycles used by bit read loop
                                                            ; 19 is value when Fosc = 4Mhz

BIT_TO_BIT_LOOP_DELAY_H			EQU	BIT_TO_BIT_LOOP_DELAY/.2
													; used in decfsz loops to delay after start bit
													; half of normal bit width to put timing into center
													; of first data bit

FINAL_BIT_LOOP_DELAY			EQU	.22*INSTRUCTION_RATE	; used in decfsz loops to delay after the final bit
                                                            ; so it won't be seen as the next start bit -- it
                                                            ; is slightly longer than a full bit delay
                                                            ; 22 is value when Fosc = 4MHz  


CURSOR_BLINK_RATE		EQU .15*INSTRUCTION_RATE    ; controls how fast the character at the cursor location blinks

DISPLAY_ON_OFF_CMD_MASK	EQU 0xf8		; masks lower bits off on/off command to leave only the command type
DISPLAY_ON_OFF_CMD		EQU 0x08		; the upper bits which specify the command type

DISP_ON_CURSOR_OFF_BLINK_OFF_CMD	equ 0x0c	; command to turn display on, cursor off, blink off

BLINK_ON_OFF_CMD_FLAG	EQU	.0


CHAR_AT_CURSOR_STATE_XOR_MASK	EQU		0x02	; use to flip flag bit using XOR

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

; Assign variables in RAM - Bank 0
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

    flags2                  ; bit 0: 1 = first serial port header byte received
                            ; bit 1: 1 = second serial port header byte received
                            ; bit 2: 1 = serial port packet length byte received and validated
                            ; bit 3: 1 = data packet ready for processing
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =

    statusFlags             ; bit 0: 0 = one or more com errors from serial have occurred
                            ; bit 1: 0 = one or more com errors from I2C have occurred
                            ; bit 2: 0 =
                            ; bit 3: 0 =
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =

	controlByte				; the first byte of each serial data byte pair is stored here
	lcdData					; stores data byte to be written to the LCD

	currentCursorLocation	; the current cursor location on the display; this is the
							; code which is sent to the display to set that location

	currentCursorBufPosH    ; the current buffer position of the last cursor location
    currentCursorBufPosL	;     specified by the "Main" PIC

	currentLCDOnOffState	; on/off state of the LCD along with cursor on/off and
							; blink on/off; this is code for the display to set those

	charBlinkRate			; delay to control blink rate of cursor at character location

	smallDelayCnt			; used to count down for small delay
	bigDelayCnt				; used to count down for big delay
    msDelayCnt              ; used to count milliseconds

	scratch0				; scratch pad variable
	scratch1				; scratch pad variable
	scratch2				; scratch pad variable

	; next variables ONLY written to by interrupt code

	intScratch0				; scratch pad variable for exclusive use by interrupt code

	; end of variables ONLY written to by interrupt code

    serialPortErrorCnt      ; number of com errors from Rabbit via serial port
    slaveI2CErrorCnt        ; number of com errors from Slave PICs via I2C bus

    serialRcvPktLenMain     ; used by main to process completed packets
    serialRcvPktCntMain     ; used by main to process completed packets

    usartScratch0
    usartScratch1
    serialIntScratch0
    
    ; used by serial receive interrupt
    
    serialRcvPktLen
    serialRcvPktCnt
    serialRcvBufPtrH
    serialRcvBufPtrL
    serialRcvBufLen

    ; used by serial transmit interrupt
    
    serialXmtBufNumBytes
    serialXmtBufPtrH
    serialXmtBufPtrL
    serialXmtBufLen

 endc

;-----------------

; Assign variables in RAM - Bank 1
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
    lcdBufOutPtrH			; read-from buffer pointer for transfer to LCD
    lcdBufOutPtrL

	lcdInColumn				; current column being written to in the buffer
	lcdBufInPtrH			; write-to buffer from master PIC pointer
    lcdBufInPtrL

 endc

;-----------------

; Assign LCD character buffer in RAM - Bank 2
; Bank 2 has 80 bytes of free space

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

; Assign variables in RAM - Bank 3
; Bank has 80 bytes of free space
 
; WARNING: These buffers may be large enough to overrun the following banks. Linear indirect
; addressing is used to access them. They may be moved to a higher bank if necessary to make room
; for variables in this bank.

    cblock 0x1a0                        ; starting address
 
    serialRcvBuf:SERIAL_RCV_BUF_LEN    
    
    serialXmtBuf:SERIAL_XMT_BUF_LEN    
    
    endc

; Compute address of serialRcvBuf in linear data memory for use as a large buffer
RCV_BUF_OFFSET EQU (serialRcvBuf & 0x7f) - 0x20
SERIAL_RCV_BUF_LINEAR_ADDRESS   EQU ((serialRcvBuf/.128)*.80)+0x2000+RCV_BUF_OFFSET
SERIAL_RCV_BUF_LINEAR_LOC_H     EQU high SERIAL_RCV_BUF_LINEAR_ADDRESS
SERIAL_RCV_BUF_LINEAR_LOC_L     EQU low SERIAL_RCV_BUF_LINEAR_ADDRESS
    
; Compute address of serialXmtBuf in linear data memory for use as a large buffer
XMT_BUF_OFFSET EQU (serialXmtBuf & 0x7f) - 0x20
SERIAL_XMT_BUF_LINEAR_ADDRESS   EQU ((serialXmtBuf/.128)*.80)+0x2000+XMT_BUF_OFFSET
SERIAL_XMT_BUF_LINEAR_LOC_H     EQU high SERIAL_XMT_BUF_LINEAR_ADDRESS
SERIAL_XMT_BUF_LINEAR_LOC_L     EQU low SERIAL_XMT_BUF_LINEAR_ADDRESS
          
;-----------------

; Define variables in the memory which is mirrored in all RAM banks.
;
; On older PICs, this section was used to store context registers during an interrupt as the
; current bank was unknown upon entering the interrupt. Now, the section can be used for any
; purpose as the more powerful PICs automatically save the context on interrupt.
;
;	Bank 0		Bank 1		Bank 2		Bank3
;	70h-7fh		f0h-ffh		170h-17fh	1f0h-1ffh
;

 cblock	0x70

 
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
; NOTE: You must set PCLATH before jumping to the interrupt routine - if PCLATH is wrong the
; jump will fail.

    movlp   high handleInterrupt
    goto    handleInterrupt	; points to interrupt service routine

; end of Power On and Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;

setup:

    clrf    INTCON          ; disable all interrupts

    call    setupClock      ; set system clock source and frequency

    call    setupPortA

    banksel LCD_RW_RS_L
    bcf		LCD_RW_RS_L,LCD_RW  ; set LCD R/W low to select write to LCD mode
    bcf		LCD_RW_RS_L,LCD_RS	; set LCD Register Select low (chooses instruction register)

    call    setupPortB

    banksel LCD_E_L
 	bcf		LCD_E_L,LCD_E       ; set LCD E strobe low (inactive)

    call    setupPortC

    movlp   high setupSerialPort
    call    setupSerialPort
    movlp   high setup
    
    banksel OPTION_REG

    movlw   0x58
    movwf   OPTION_REG      ; Option Register = 0x58   0101 1000 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 0 : TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
                            ; bit 4 = 1 : TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
                            ; bit 3 = 1 : PSA ~ Prescaler disabled; Timer0 will be 1:1 with Fosc/4
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    000 = 1:2 scaling for Timer0 if enabled
                            ; bit 0 = 0 :

    banksel TMR0

	movlw	TIMER0_RELOAD_START_BIT_SEARCH_Q
	movwf	TMR0

; enable the interrupts

    banksel INTCON

	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enabe TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

    banksel flags

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
; setupClock
;
; Sets up the system clock source and frequency (Fosc).
;
; Instruction cycle rate is Fosc/4.
;
; Assumes clock related configuration bits are set as follows:
;
;   _FOSC_INTOSC,  _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_DISABLED
;
; Assumes all programmable clock related options are at Reset default values.
;
; NOTE: Adjust I2C baud rate generator value when Fosc is changed.
;       Adjust FOSC_IN_MHZ also.
;
; 16 Mhz -> IRCF<3:0> = 1111
;  4 Mhz ->  IRCF<3:0> = 1101
;

setupClock:

    ; choose internal clock frequency of 16 Mhz ~ IRCF<3:0> = 1111

    banksel OSCCON

    bsf     OSCCON, IRCF3
    bsf     OSCCON, IRCF2
    bsf     OSCCON, IRCF1
    bsf     OSCCON, IRCF0

    return

; end of setupClock
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortA
;
; Sets up Port A for I/O operation.
;
; NOTE: Writing to PORTA is same as writing to LATA for PIC16f1459. The code example from the
; data manual writes to both on initialization -- probably to be compatible with other PIC chips.
;
; NOTE: RA0, RA1 and RA3 can only be inputs on the PIC16f1459 device.
;       RA2, RA6, RA7 are not implemented.
;

setupPortA:

    banksel WPUA
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUA

    banksel PORTA
    clrf    PORTA                       ; init port value

    banksel LATA                        ; init port data latch
    clrf    LATA

    banksel ANSELA
    clrf    ANSELA                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISA
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISA

    ; set direction for each pin used

    bcf     TRISA, LCD_RW               ; output
    bcf     TRISA, LCD_RS               ; output
    bsf     TRISA, RA1                  ; input - unused - pulled up via external resistor

    return

; end of setupPortA
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortB
;
; Sets up Port B for I/O operation.
;
; NOTE: Writing to PORTB is same as writing to LATB for PIC16f1459. The code example from the
; data manual writes to both on initialization -- probably to be compatible with other PIC chips.
;
; NOTE: RB0, RB1, RB2, RB3 are not implemented on the PIC16f1459 device.
;

setupPortB:

    banksel WPUB
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUB

    banksel PORTB
    clrf    PORTB                       ; init port value

    banksel LATB                        ; init port data latch
    clrf    LATB

    banksel ANSELB
    clrf    ANSELB                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISB
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISB

    ; set direction for each pin used

    bcf     TRISB, LCD_E                ; output

    return

; end of setupPortB
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortC
;
; Sets up Port C for I/O operation.
;
; NOTE: Writing to PORTC is same as writing to LATC for PIC16f1459. The code example from the
; data manual writes to both on initialization -- probably to be compatible with other PIC chips.
;
; For this program, PortC is the LCD data bus.
;

setupPortC:

    ; Port C does not have a weak pull-up register

    banksel PORTC
    clrf    PORTC                       ; init port value

    banksel LATC
    clrf    LATC                        ; init port data latch

    banksel ANSELC
    clrf    ANSELC                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISC
    movlw   b'00000000'                 ; set all to outputs
    movwf   TRISC

    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------

;debug mks
;    banksel SERIAL_OUT_L
;    bcf     SERIAL_OUT_L,SERIAL_OUT
;    bsf     SERIAL_OUT_L,SERIAL_OUT
;    bcf     SERIAL_OUT_L,SERIAL_OUT
;debug mks end

;--------------------------------------------------------------------------------------------------
; Main Code
;
; Sets up the PIC, the LCD, displays a greeting, then monitors the serial data input line from
; the main PIC for data and instructions to be passed on to the LCD display.
;

start:

	call	setup			; set up main variables and hardware

	call    initLCD

	call	setUpLCDCharacterBuffer

	call	clearLCDLocalBuffer

	call	displayGreeting


; begin monitoring the serial data from the main PIC for data and instructions to be passed on to
; the LCD display

; in between each check for incoming data on the serial line, write one character from the local
; LCD buffer to the LCD display

mainLoop:

    call    handleReceivedDataIfPresent ; process received packet if available

	call	writeNextCharInBufferToLCD  ;write one character in the local buffer to the LCD

    goto    mainLoop

; end of Main Code
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleLCDDataPacket
;
; Handles an LCD data byte packet by writing the byte to the local LCD buffer from which it will
; be written to the LCD on the next scan.
;
; Control codes other than address changes and clear screen commands are written to the LCD
; display immediately. Address changes and data values are used to address and store in the local
; LCD character buffer.
;
; The first byte is the packet command byte, followed by the LCD data byte.
;
; On Entry:
;
;   FSR1 points to serialRcvBuf
; 

handleLCDDataPacket:

    moviw   1[FSR1]                     ; get the switch state value from the packet

    banksel lcdData                     ; store data byte
    movwf   lcdData

    call    writeToLCDBuffer            ; store byte in the local LCD character buffer

    return

; end of handleLCDDataPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleLCDInstructionPacket
;
; Control codes other than address changes and clear screen commands are written to the LCD
; display immediately. Address changes are used to set the address in the local LCD buffer.
;
; The first byte is the packet command byte, followed by the LCD instruction byte.
;
; On Entry:
;
;   FSR0 points to serialRcvBuf
; 

handleLCDInstructionPacket:

    moviw   1[FSR1]                     ; get the switch state value from the packet

    banksel lcdData                     ; store data byte
    movwf   lcdData

    call    handleLCDInstruction        ; process the instruction code

    return

; end of handleLCDInstructionPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleLCDBlockPacket
;
; Handles a block packet containing one or more LCD instructions and LCD data.   
;
; The first byte is the packet command byte, followed by the LCD instructions/data bytes.
; Each instruction or data set consists of a two byte sequence -- if first byte is 0x00 then the
; following byte is a data byte; if first byte is 0x01 then the following byte is an instruction
; byte.
;
; On Entry:
;
;   FSR1 points to serialRcvBuf
; 

handleLCDBlockPacket:

    addfsr  FSR1,1                      ; skip the packet command byte

    movf    serialRcvPktLenMain,W       ; load counter
    movwf   serialRcvPktCntMain
    decf    serialRcvPktCntMain,F       ; account for packet command byte
    decf    serialRcvPktCntMain,F       ; account for checksum byte
    
hLBPLoop1:
    
    moviw   1[FSR1]                     ; get and store the instruction/data byte
    banksel lcdData
    movwf   lcdData

    moviw   FSR1++                      ; get instruction/data tag    
    addfsr  FSR1,1                      ; skip the instruction/data byte
    
    btfss   STATUS,Z
    goto    hLBPDoInstr

    call    writeToLCDBuffer             ; process the data byte    
    goto    hLBPNext
    
hLBPDoInstr:
    
    call    handleLCDInstruction        ; process the instruction code

hLBPNext:    
    
    banksel serialRcvPktCntMain
    decf    serialRcvPktCntMain,F       ; count the tag byte    
    decfsz  serialRcvPktCntMain,F       ; count the instruction/data byte and check for end
    goto    hLBPLoop1   
    
    return

; end of handleLCDBlockPacket
;--------------------------------------------------------------------------------------------------
    
;--------------------------------------------------------------------------------------------------
; handleLCDInstruction
;
; Handles the LCD instruction byte in lcdData.
;
; Handles LCD instruction codes. If the control code is an address change or clear screen code,
; the command is directed to the local LCD character buffer. The LCD display itself is not changed
; -- that is handled by the code which transmits the buffer contents to the display.
;
; All other control codes are transmitted directly to the LCD display.
;

handleLCDInstruction:

    banksel flags

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

writeNextCharInBufferToLCD:

    banksel lcdBufOutPtrH

    movf    lcdBufOutPtrH,W  ; get pointer to next character to be written to LCD
    movwf   FSR0H
    movf    lcdBufOutPtrL,W  ; get pointer to next character to be written to LCD
    movwf   FSR0L

	movf	INDF0,W			; load the character

    banksel lcdData
	movwf	lcdData         ; store for use by writeLCDData function

	; check if cursor is blinking and is in the "off" state

	btfsc	currentLCDOnOffState,BLINK_ON_OFF_CMD_FLAG
	btfsc	flags,CHAR_AT_CURSOR_STATE
	goto	noHideCharacter

	; character should be off

    banksel lcdBufOutPtrL
	movf	lcdBufOutPtrL,W  ; only uses LSB -- buffer should not cross 256 byte boundary
    banksel currentCursorBufPosL
	subwf	currentCursorBufPosL,W
	btfss	STATUS,Z
	goto	noHideCharacter

	movlw	' '
	movwf	lcdData         ; store for use by writeLCDData function

noHideCharacter:

    call    writeLCDData

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

incrementLCDOutBufferPointers:

    banksel lcdBufOutPtrH

	incf	lcdBufOutPtrL,F	; point to next character in buffer
    btfsc   STATUS,Z
    incf    lcdBufOutPtrH,F

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

    banksel lcdOutLine
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

handleEndOfRefreshTasks:

    banksel flags

	; refreshing the buffer moves the cursor location with each character sent
	; set the cursor location to the last location specified by the "Main" PIC

	movf	currentCursorLocation,W
	movwf	lcdData         	; store for use by writeLCDData function
    call    writeLCDInstruction

	; the cursor and blink are turned off at the start of each buffer refresh to eliminate
	; flicker as they follow each character sent; at the end of each refresh, set them
	; to the last state specified by the "Main" PIC

	; check if "Main" PIC has turned blinking on

    banksel flags

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

setLCDVariablesPerLineNumber:

    banksel lcdOutLine

	movf	lcdOutLine,W	; handle line 0
 	sublw	0
 	btfss	STATUS,Z
    goto	notLine0

	movlw   high lcd0		; start of line 0
    movwf   lcdBufOutPtrH
	movlw   low lcd0
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN0_START
	goto	writeLCDInstructionAndExit

notLine0:

	movf	lcdOutLine,W	; handle line 1
 	sublw	1
 	btfss	STATUS,Z
    goto	notLine1

	movlw   high lcd20		; start of line 1
    movwf   lcdBufOutPtrH
	movlw   low lcd20
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN1_START
	goto	writeLCDInstructionAndExit

notLine1:

	movf	lcdOutLine,W	; handle line 2
 	sublw	2
 	btfss	STATUS,Z
    goto	notLine2

	movlw   high lcd40		; start of line 2
    movwf   lcdBufOutPtrH
	movlw   low lcd40
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN2_START
	goto	writeLCDInstructionAndExit

notLine2:

	; don't check if line 3 -- any number not caught above is either 3 or illegal; if illegal then default
	; to line 3 to get things back on track

	movlw   high lcd60		; start of line 3
    movwf   lcdBufOutPtrH
	movlw   low lcd60
    movwf   lcdBufOutPtrL

	movlw	LCD_COLUMN3_START

writeLCDInstructionAndExit:

    banksel lcdData
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

clearLCDLocalBuffer:

    banksel lcdScratch0      ; select data bank 1 to access LCD buffer variables

	movlw	LCD_BUFFER_SIZE	; set up loop counter
	movwf	lcdScratch0

	movlw	high lcd0		; point indirect register FSR at buffer start
    movwf   FSR0H
	movlw	low lcd0
    movwf   FSR0L

	movlw	' '				; fill with spaces

clearLCDLoop:

	movwi	FSR0++			; store to each buffer location
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

setUpLCDCharacterBuffer:

   	banksel lcdBufInPtrH     ; select data bank 1 to access LCD buffer variables

	movlw   high lcd0		; set write to buffer pointer from master PIC to line 0 column 0
    movwf   lcdBufInPtrH
	movlw   low lcd0
    movwf   lcdBufInPtrL

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

setLCDBufferWriteAddress:

    banksel flags

	movf	lcdData,W		; load address control code from bank 0

	movwf	currentCursorLocation	; store as the cursor location for later use when the
									; display is refreshed

   	banksel lcdScratch0     ; select data bank 1 to access LCD buffer variables

	movwf	lcdScratch0		; store address control code in bank 1 for easy access

	call	getLCDLineContainingAddress	; find which line contains the specified address

    banksel lcdBufInPtrH

	movf	lcdBufInPtrH,W          ; store as the cursor location for later use in making
    banksel currentCursorBufPosH    ; the character at that location blink
	movwf	currentCursorBufPosH
   	banksel lcdScratch0
    movf	lcdBufInPtrL,W
    banksel currentCursorBufPosL
	movwf	currentCursorBufPosL

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

setLCDOnOffAndCursorAndBlink:
    
    banksel lcdData

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

writeToLCDBuffer:

    banksel lcdData

	movf	lcdData,W		; get the byte to be stored

   	banksel lcdScratch0     ; select data bank 1 to access LCD buffer variables
	movwf	lcdScratch0		; store byte in bank 1 for easy access

	movf	lcdInColumn,W	; bail out if already one past the max column number
 	sublw	PAST_MAX_COLUMN
 	btfsc	STATUS,Z
	return

	incf	lcdInColumn,f	; track number of bytes written to the line

    movf    lcdBufInPtrH,W  ; get pointer to next memory location to be used
    movwf   FSR0H           ; point FSR at the character
    movf    lcdBufInPtrL,W  ; get pointer to next memory location to be used
    movwf   FSR0L           ; point FSR at the character

	movf	lcdScratch0,W	; retrieve the byte and store it in the buffer
    movwf	INDF0

    incf    lcdBufInPtrL,F	; increment the pointer
    btfsc   STATUS,Z
    incf    lcdBufInPtrH,F

	return

; end of writeToLCDBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; getLCDLineContainingAddress
;
; Returns in the W register the line containing the address specified by the control code in
; lcdScratch0. The control code is the value which would be sent to the LCD display to set the
; address.
;
; The lcdBufInPtr will be set to the proper buffer location for storing at the spot which mirrors
; the specified address in the LCD buffer. The LCD buffer has a non-contiguous addressing while
; the buffer in the PIC is contiguous, so some translation is required.
;
; NOTE: this function only manipulates the lower byte of the lcdBufInPtr. It is assumed that the
; buffer does not cross a 256 byte boundary and the pointer high byte does not change.
;
; An illegal address outside the range of any line defaults to line 3.
;
; see "LCD ADDRESSING NOTE" in header notes at top of page for addressing explanation
;
; REMEMBER: Borrow flag is inverse: 0 = borrow, 1 = no borrow
;

getLCDLineContainingAddress:

    banksel lcdScratch0

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
	movwf	lcdBufInPtrL		; to get the address's memory location

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
	movwf	lcdBufInPtrL		; to get the address's memory location

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
	movwf	lcdBufInPtrL		; to get the address's memory location

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
	movwf	lcdBufInPtrL		; to get the address's memory location

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
; wip mks -- convert this to the write string method used in "OPT EDM Main PIC.asm"
;

displayGreeting:

    banksel flags

	movlw	0x80			; move cursor to line 1 column 1 (address 0x00 / code 0x80)
	movwf	lcdData         ;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'O'				; display "OPT EDM" on the first line
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'P'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'T'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	' '
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'E'
	movwf 	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'D'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'M'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	0xc1			; move cursor to line 2 column 2 (address 41h)
	movwf	lcdData         ;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'N'				; display "Notcher" on the second line
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'o'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	't'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'c'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'h'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'e'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'r'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	0x96			; move cursor to line 3 column 7 (address 16h)
	movwf	lcdData			;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'b'				; display "by CMP" on the third line
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'y'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	' '
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'M'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'K'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'S'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	0xd7			; move cursor to line 4 column 8 (address 57h)
	movwf	lcdData			;   (bit 7 = 1 specifies address set command, bits 6:0 are the address)
    call	setLCDBufferWriteAddress
    banksel flags

	movlw	'R'				; display "Rev 2.7" on the fourth line
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'e'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'v'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	' '
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'3'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

 	movlw	'.'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

	movlw	'1'
	movwf	lcdData
    call    writeToLCDBuffer
    banksel flags

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

    banksel LCD_E_L
	bcf		LCD_E_L,LCD_E		; LCD E strobe low
    banksel LCD_RW_RS_L
	bcf		LCD_RW_RS_L,LCD_RS  ; LCD RS low (instruction register selected)
    bcf		LCD_RW_RS_L,LCD_RW  ; set LCD R/W low to select write to LCD mode

    movlw   .200                ; delay at least 15 ms after Vcc = 4.5V
    call    msDelay             ;  delay plenty to allow power to stabilize

	movlw	0x30				; 1st send of Function Set Command: (8-Bit interface)
                                ; (BF cannot be checked before this command.)
    banksel	LCD_DATA_OUT_L
	movwf	LCD_DATA_OUT_L		; prepare to write
    call    strobeE				; write to LCD

    movlw   .6                  ; delay at least 100uS
    call    msDelay

	movlw	0x30				; 2nd send of Function Set Command: (8-Bit interface)
                                ; (BF cannot be checked before this command.)
	banksel	LCD_DATA_OUT_L
    movwf	LCD_DATA_OUT_L		; prepare to write
    call    strobeE				; write to LCD

    movlw   .6                  ; delay at least 100uS
    call    msDelay

	movlw	0x30				; 3rd send of Function Set Command: (8-Bit interface)
    banksel	LCD_DATA_OUT_L      ; (BF can be checked after this command)
	movwf	LCD_DATA_OUT_L		; prepare to write
    call    strobeE				; write to LCD

    call    waitWhileLCDBusy

	movlw	0x38				; write 0011 1000 Function Set Command
                                ;       multi line display with 5x7 dot font
    banksel	LCD_DATA_OUT_L
	movwf	LCD_DATA_OUT_L		;  0011 in upper nibble specifies Function Set Command
    call    strobeE				;  bit 3: 0 = 1 line display, 1 = multi-line display
								;  bit 2: 0 = 5x7 dot font, 1 = 5 x 10 dot font

    call    waitWhileLCDBusy

	movlw	0x0c				; manual says to use 0x08 here (display off) but nothing works
    banksel	LCD_DATA_OUT_L      ;       if that is done -- not sure why? 0x0c = display on
	movwf	LCD_DATA_OUT_L		;  bit 3: specifies display on/off command
    call    strobeE				;  bit 2: 0 = display off, 1 = display on

    call    waitWhileLCDBusy

	movlw	0x01				; write 0000 0001 ~ Clear Display
    banksel	LCD_DATA_OUT_L
	movwf	LCD_DATA_OUT_L
    call    strobeE

    call    waitWhileLCDBusy

	movlw	0x06				; write 0000 0110 ~ Entry Mode Set, increment mode, no display shift
    banksel	LCD_DATA_OUT_L
	movwf	LCD_DATA_OUT_L		; bits 3:2 = 0:1 : specifies Entry Mode Set
    call    strobeE				; bit 1: 0 = no increment, 1 = increment mode; 
                                ; bit 0: 0 = no shift, 1 = shift display

    call    waitWhileLCDBusy

	movlw	0x0c				; write 0000 1100 ~ Display On, cursor off, blink off
    banksel	LCD_DATA_OUT_L
	movwf	LCD_DATA_OUT_L      ;  bit 3: specifies display on/off command
    call    strobeE				;  bit 2: 0 = display off, 1 = display on
								;  bit 1: 0 = cursor off, 1 = cursor on
								;  bit 0: 0 = blink off, 1 = blink on

; Note: BF should be checked before each of the instructions starting with Display OFF.

	return

; end of initLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeLCDData
;
; Writes a data byte from variable lcdData to the LCD.  The data is a character to be displayed.
;
; Waits until LCD ready before writing.
;

writeLCDData:

    call    waitWhileLCDBusy        ; wait until LCD ready

    banksel LCD_RW_RS_L
	bsf		LCD_RW_RS_L,LCD_RS		; set RS high to select data register in LCD
    bcf		LCD_RW_RS_L,LCD_RW      ; set R/W low to write

    banksel lcdData                 ; place data on output port
	movf	lcdData,W
    banksel LCD_DATA_OUT_L
	movwf	LCD_DATA_OUT_L

    goto    strobeE					; write the data

; end of writeLCDData
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeLCDInstruction
;
; Writes an instruction byte from variable lcdData to the LCD.  An instruction is one of several
; codes recognized by the LCD display for clearing the screen, moving the cursor, etc.
;
; Waits until LCD ready before writing.
;

writeLCDInstruction:

    call    waitWhileLCDBusy        ; wait until LCD ready

    banksel LCD_RW_RS_L
	bcf		LCD_RW_RS_L,LCD_RS		; set RS low to select instruction register in LCD
    bcf		LCD_RW_RS_L,LCD_RW      ; set R/W low to write

    banksel lcdData                 ; place data on output port
	movf	lcdData,W				
    banksel LCD_DATA_OUT_L
	movwf	LCD_DATA_OUT_L

    goto    strobeE					; write the data

; end of writeLCDInstruction
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitWhileLCDBusy
;
; Loops until the LCD busy flag returns as not-busy.
;
; The busy flag is returned in bit 7 when the LCD instruction register is read.
;

waitWhileLCDBusy:

    banksel TRISC
    movlw   b'11111111'             ; set all to inputs so data can be read
    movwf   TRISC                   ; do this before RW is taken high

    banksel LCD_RW_RS_L
	bcf		LCD_RW_RS_L,LCD_RS		; set RS low to select instruction register in LCD
    bsf		LCD_RW_RS_L,LCD_RW      ; set R/W high to read

wWLBLoop1:

    call    strobeE					; read from LCD

    btfsc   WREG,7                  ; exit if busy flag = 0
    goto    wWLBLoop1               ; loop if busy flag = 1

    banksel LCD_RW_RS_L
    bcf		LCD_RW_RS_L,LCD_RW      ; set R/W low to write

    banksel TRISC
    movlw   b'00000000'             ; set all to outputs
    movwf   TRISC                   ; do this after RW is taken low

    return

; end of waitWhileLCDBusy
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; strobeE
;
; Strobes LCD E line to write or read data to the LCD.
;
; Control lines RS and WR should have been set at least 2 instructions prior to calling this
; function to ensure proper setup time. Generally, the call opcode will satisfy this requirement.
; The third instruction required for delay is provided by the banksel in this function.
;
; The LCD data port must already be in the proper configuration: input for reading and output
; for writing.
;
; Data from the port is returned in W. If reading data, this will be the byte read from the LCD.
; If writing data, the byte returned is an unpredictable value and should not be used.
;
; Data is strobed into the LCD on the falling edge of E.
; Data can be read from LCD just before rising edge of E.
;
; Program is set up to access LCD when PIC is running at fastest speed (48 MHz Fosc) so it will
; work for all possible speeds.
;
; Instruction cycle time at 48 Mhz Fosc (fastest speed option): 0.0833 uS = 83 nS
;
; Number of nops at 48 Mhz Fosc required to meet timing shown in parentheses below. Each value is
; rounded up to the next integer and then increased by one to ensure timing is met.
;
; Write to LCD:
;
; Minimum time RS,RW to E rising edge: 140 ns (3 instructions)
; Minimum E high pulse width: 450 ns (7 instructions)
; Minimum time Data stable to E falling edge: 195 ns (4 instructions)
;
; Read from LCD:
; 
; Max time Data stable after E rising edge: 320 ns (5 instructions)
;

strobeE:

    banksel LCD_E_L
	bsf		LCD_E_L,LCD_E

    nop                         ;see timing in notes above
    nop                         ;E high for minimum 7 instructions
    nop
    nop

    banksel LCD_DATA_IN_P       ;read data from LCD port - valid only if reading
    movf    LCD_DATA_IN_P,W

    banksel LCD_E_L
    bcf		LCD_E_L,LCD_E

	return

; end of strobeE
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; smallDelay
;
; Creates a small delay.
;

smallDelay:

    banksel smallDelayCnt

    ifdef debug       ; if debugging, don't delay
    return
    endif

	movlw	0x2a
	movwf	smallDelayCnt

L8b:
	decfsz	smallDelayCnt,F
    goto    L8b
	return

; end of smallDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; msDelay
;
; Creates a delay of x milliseconds where x is specified in the W register.
;
; On Entry:
;
; W contains number of milliseconds to delay.
;
; Values to achieve 1 millisecond for various Fosc:
; 
; for 4Mhz  Fosc -> bigDelayCnt = .2, smallDelayCnt = .166
; for 16Mhz Fosc -> bigDelayCnt = .6, smallDelayCnt = .222
;
; Note: these values do not take into account interrupts processing which will increase the delay.
;

msDelay:

    banksel msDelayCnt

    ifdef debug                 ; if debugging, don't delay
    return
    endif

    movwf   msDelayCnt          ; number of milliseconds

msD1Loop1:

	movlw	.6                  ; smallDelayCnt * bigDelayCnt give delay of 1 millisecond
	movwf	bigDelayCnt

msD1Loop2:

	movlw	.222                
	movwf	smallDelayCnt

msD1Loop3:

	decfsz	smallDelayCnt,F
    goto    msD1Loop3

	decfsz	bigDelayCnt,F
    goto    msD1Loop2

	decfsz	msDelayCnt,F
    goto    msD1Loop1

	return

; end of msDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleReceivedDataIfPresent
;
; Processes data in the serial receive buffer if a packet has been received.
;
; 

handleReceivedDataIfPresent:

    banksel flags2                          ; handle packet in serial receive buffer if ready
    btfsc   flags2, SERIAL_PACKET_READY
    goto    handleSerialPacket

    return

; end of handleReceivedDataIfPresent
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPacket
;
; Processes a packet in the serial receive buffer.
;

handleSerialPacket:

    banksel serialRcvPktLen

    movf    serialRcvPktLen,W           ; store the packet length variable so the receive interrupt
    movwf   serialRcvPktLenMain         ; can overwrite it if a new packet arrives
        
    call    resetSerialPortRcvBuf       ; allow the serial receive interrupt to start a new packet
                                        ; see "Serial Data Timing" notes at the top of this page
    
    ;verify the checksum

    banksel serialRcvPktLenMain
  
    movf    serialRcvPktLenMain, W      ; copy number of bytes to variable for counting
    movwf   serialRcvPktCntMain

    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H ; point FSR0 at start of receive buffer
    movwf   FSR0H
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   FSR0L

    clrw                                ; preload W with zero

hspSumLoop:

    addwf   INDF0, W                    ; sum each data byte and the checksum byte at the end
    incf    FSR0L, F
    decfsz  serialRcvPktCntMain, F
    goto    hspSumLoop

    movf    WREG, F                         ; test for zero
    btfsc   STATUS, Z                       ; error if not zero
    goto    parseCommandFromSerialPacket    ; checksum good so handle command

hspError:

    incf    serialPortErrorCnt, F           ; track errors
    bsf     statusFlags,SERIAL_COM_ERROR

    return

; end of handleSerialPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; parseCommandFromSerialPacket
;
; Parses the command byte in a serial packet and performs the appropriate action.
;
; On Entry:
;
; On Exit:
;
; FSR1 points to the start of the serial port receive buffer
;

parseCommandFromSerialPacket:

    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H ; point FSR0 at start of receive buffer
    movwf   FSR1H
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   FSR1L

; parse the command byte by comparing with each command

    movf    INDF1,W
    sublw   LCD_BLOCK_CMD
    btfsc   STATUS,Z
    goto    handleLCDBlockPacket

    movf    INDF1,W
    sublw   LCD_DATA_CMD
    btfsc   STATUS,Z
    goto    handleLCDDataPacket

    movf    INDF1,W
    sublw   LCD_INSTRUCTION_CMD
    btfsc   STATUS,Z
    goto    handleLCDInstructionPacket
    
    return

; end of parseCommandFromSerialPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------
;   EUSART Serial Port Core Functions
;
; Copy this block of code for all basic functions required for serial transmit. Build code, then
; copy all variables and defines which are shown to be missing.
;
;--------------------------------------------------------------------------------------------------
; setUpSerialXmtBuf
;
; Adds the header bytes, length byte, command byte, and various values from this Master PIC to the
; start of the serial port transmit buffer and sets serialXmtBufPtrH:L ready to add data bytes.
;
; Notes on packet length:
;
;   Example with 1 data bytes...
;
;   2 bytes (command byte + data byte)
;   ---
;   2 total (value passed to calcAndStoreCheckSumSerPrtXmtBuf; number bytes checksummed)
;
;   ADD (to determine length byte to insert into packet)
;
;   +1 checksum byte for the overall packet
;   3 total (value passed to setUpSerialXmtBuffer (this function) for packet length)
;
;   ADD (to determine actual number of bytes to send)
;
;   +2 header bytes
;   +1 length byte
;   ---
;   6 total (value passed to startSerialPortTransmit)
;
; On Entry:
;
; usartScratch0 should contain the number of data bytes plus one for the checksum byte in the packet
; usartScratch1 should contain the command byte
;
; On Exit:
;
; FSR0 and serialXmtBufPtrH:serialXmtBufPtrL will point to the location for the next data byte
; serialXmtBufNumBytes will be zeroed
;

setUpSerialXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    banksel usartScratch0

    movlw   0xaa
    movwi   FSR0++                          ; store first header byte

    movlw   0x55
    movwi   FSR0++                          ; store first header byte

    movf    usartScratch0,W                 ; store length byte
    movwi   FSR0++

    movf    usartScratch1,W                 ; store command byte
    movwi   FSR0++

    banksel serialXmtBufPtrH                ; point serialXmtBufPtrH:L at next buffer position
    movf    FSR0H,W
    movwf   serialXmtBufPtrH
    movf    FSR0L,W
    movwf   serialXmtBufPtrL

    clrf   serialXmtBufNumBytes             ; tracks number of bytes added -- must be adjusted
                                            ; later to include the header bytes, length, command

    return

; end of setUpSerialXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; startSerialPortTransmit
;
; Initiates sending of the bytes in the transmit buffer. The transmission will be performed by an
; interrupt routine.
;
; The command byte and all following data bytes are used to compute the checksum which is inserted
; at the end.
;
; On Entry:
;
; serialXmtBufNumBytes should contain the number of bytes to send.
; The bytes to be sent should be in the serial port transmit buffer serialXmtBuf.
;

startSerialPortTransmit:

    ; get number of bytes stored in buffer, add one for the command byte, calculate checksum

    banksel serialXmtBufNumBytes
    movf    serialXmtBufNumBytes,W
    addlw   .1
    movwf   serialXmtBufNumBytes
    movwf   usartScratch0

    call    calcAndStoreCheckSumSerPrtXmtBuf

    banksel serialXmtBufPtrH                ; set FSR0 and pointer to start of transmit buffer
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H
    movwf   serialXmtBufPtrH
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   serialXmtBufPtrL
    movwf   FSR0L

    ; add 1 to length to account for checksum byte, store in packet length byte
    ;  packet length = command byte + data bytes + checksum

    banksel serialXmtBufNumBytes
    movf    serialXmtBufNumBytes,W
    addlw   .1
    movwi   2[FSR0]

    ; add 3 to length to account for two header bytes and length byte, store for xmt routine
    ; this is the total number of bytes to transmit

    addlw   .3
    movwf   serialXmtBufNumBytes

    banksel PIE1                            ; enable transmit interrupts
    bsf     PIE1, TXIE                      ; interrupt will trigger when transmit buffers empty

    return

; end of startSerialPortTransmit
;--------------------------------------------------------------------------------------------------
 
;--------------------------------------------------------------------------------------------------
; clearSerialPortXmtBuf
;
; Sets all bytes up to 255 in the Serial Port transmit buffer to zero. If the buffer is larger
; than 255 bytes, only the first 255 will be zeroed.
;

clearSerialPortXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    banksel usartScratch0                       ; get buffer size to count number of bytes zeroed
    movlw   SERIAL_XMT_BUF_LEN
    movwf   usartScratch0

    movlw   0x00

cSPXBLoop:

    movwi   FSR0++
    decfsz  usartScratch0,F
    goto    cSPXBLoop

    return

; end of clearSerialPortXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupSerialPort
;
; Sets up the serial port for communication.
; Also prepares the receive and transmit buffers for use.
;

setupSerialPort:

    banksel serialRcvBufLen     ;store buffer length constants in variables for easier maths

    movlw   SERIAL_RCV_BUF_LEN
    movwf   serialRcvBufLen
    movlw   SERIAL_XMT_BUF_LEN
    movwf   serialXmtBufLen

    clrf    serialPortErrorCnt
    bcf     statusFlags,SERIAL_COM_ERROR

    ;set the baud rate to 57,600 (will actually be 57.97K with 0.64% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 68

    banksel TXSTA
    bsf     TXSTA, BRGH
    banksel BAUDCON
    bsf     BAUDCON, BRG16
    banksel SPBRGH
    clrf    SPBRGH
    banksel SPBRGL
    movlw   .68
    movwf   SPBRGL

    ;set UART mode and enable receiver and transmitter

    banksel ANSELB          ; RB5/RB7 digital I/O for use as RX/TX
    bcf     ANSELB,RB5
    bcf     ANSELB,RB7

    banksel TRISB
    bsf     TRISB, TRISB5   ; set RB5/RX to input
    bcf     TRISB, TRISB7   ; set RB7/TX to output

    banksel TXSTA
    bcf     TXSTA, SYNC     ; clear bit for asynchronous mode
    bsf     TXSTA, TXEN     ; enable the transmitter
    bsf     RCSTA, CREN     ; enable the receiver
    bsf     RCSTA, SPEN     ; enable EUSART, configure TX/CK I/O pin as an output

    call    resetSerialPortRcvBuf
    call    resetSerialPortXmtBuf

    ; enable the receive interrupt; the transmit interrupt (PIE1/TXIE) is not enabled until data is
    ; ready to be sent
    ; for interrupts to occur, INTCON/PEIE and INTCON/GIE must be enabled also

    banksel PIE1
    bsf     PIE1, RCIE      ; enable receive interrupts
    bcf     PIE1, TXIE      ; disable transmit interrupts (re-enabled when data is ready to xmt)

    return

; end of setupSerialPort
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForTXIFHigh
;
; Waits in a loop for TXIF bit in register PIR1 to go high. This signals that the EUSART serial
; port transmit buffer is empty and a new byte can be sent.
;

waitForTXIFHigh:

    ifdef debug_on    ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    return            ; simulated by the IDE
    endif

    banksel PIR1

wfth1:
    btfss   PIR1, TXIF
    goto    wfth1

    return

; end of waitForTXIFHigh
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetSerialPortRcvBuf
;
; Resets all flags and variables associated with the serial port receive buffer.
;

resetSerialPortRcvBuf:

    banksel flags2

    bcf     flags2, HEADER_BYTE_1_RCVD
    bcf     flags2, HEADER_BYTE_2_RCVD
    bcf     flags2, LENGTH_BYTE_VALID
    bcf     flags2, SERIAL_PACKET_READY

    clrf    serialRcvPktLen
    clrf    serialRcvPktCnt
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_H
    movwf   serialRcvBufPtrH
    movlw   SERIAL_RCV_BUF_LINEAR_LOC_L
    movwf   serialRcvBufPtrL

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    RSPRBnoOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN
        
RSPRBnoOERRError:

    banksel RCREG           ; clear any pending interrupt by clearing both bytes of the buffer
    movf    RCREG, W
    movf    RCREG, W
        
    return

; end of resetSerialPortRcvBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetSerialPortXmtBuf
;
; Resets all flags and variables associated with the serial port transmit buffer.
;

resetSerialPortXmtBuf:

    banksel flags2

    clrf    serialXmtBufNumBytes
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H
    movwf   serialXmtBufPtrH
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   serialXmtBufPtrL

    return

; end of resetSerialPortXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calcAndStoreCheckSumSerPrtXmtBuf
;
; Calculates the checksum for a series of bytes in the serial port transmit buffer. The two
; header bytes and the length byte are not included in the checksum.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series, not including the 2 header bytes and 1 length
; byte
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calcAndStoreCheckSumSerPrtXmtBuf:

    movlw   SERIAL_XMT_BUF_LINEAR_LOC_H                   ; set FSR0 to start of transmit buffer
    movwf   FSR0H
    movlw   SERIAL_XMT_BUF_LINEAR_LOC_L
    movwf   FSR0L

    addfsr  FSR0,.3                             ; skip 2 header bytes and 1 length byte
                                                ; command byte is part of checksum

    goto    calculateAndStoreCheckSum

; end calcAndStoreCheckSumSerPrtXmtBuf
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; calculateAndStoreCheckSum
;
; Calculates the checksum for a series of bytes.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series
; FSR0 points to first byte in series.
;
; On Exit:
;
; The checksum will be stored at the end of the series.
; FSR0 points to the location after the checksum.
;

calculateAndStoreCheckSum:

    call    sumSeries                       ; add all bytes in the buffer

    comf    WREG,W                          ; use two's complement to get checksum value
    addlw   .1

    movwi   FSR0++                          ; store the checksum at the end of the summed series

    return

; end calculateAndStoreCheckSum
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sumSeries
;
; Calculates the sum of a series of bytes. Only the least significant byte of the sum is retained.
;
; On Entry:
;
; usartScratch0 contains number of bytes in series.
; FSR0 points to first byte in series.
;
; On Exit:
;
; The least significant byte of the sum will be returned in WREG.
; Z flag will be set if the LSB of the sum is zero.
; FSR0 points to the location after the last byte summed.
;

sumSeries:

    banksel usartScratch0

    clrf    WREG

sumSLoop:                       ; sum the series

    addwf   INDF0,W
    addfsr  INDF0,1

    decfsz  usartScratch0,F
    goto    sumSLoop

    return

; end sumSeries
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInterrupt
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleInterrupt:

                                    ; INTCON is a core register, no need to banksel
	btfsc 	INTCON, T0IF     		; Timer0 overflow interrupt?
	call 	handleTimer0Int         ; call so the serial port interrupts will get checked
                                    ;  if not, the timer interrupt can block them totally

    banksel PIR1
    btfsc   PIR1, RCIF              ; serial port receive interrupt
    goto    handleSerialPortReceiveInt

    banksel PIE1                    ; only handle UART xmt interrupt if enabled
    btfss   PIE1, TXIE              ;  the TXIF flag is always set whenever the buffer is empty
    retfie                          ;  and should be ignored unless the interrupt is enabled
    
    banksel PIR1
    btfsc   PIR1, TXIF              ; serial port transmit interrupt
    goto    handleSerialPortTransmitInt


; Not used at this time to make interrupt handler as small as possible.
;	btfsc 	INTCON, RBIF      		; NO, Change on PORTB interrupt?
;	goto 	portB_interrupt       	; YES, Do PortB Change thing

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

endISR:

	retfie                  	; Return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Int
;
; This function is called when the Timer0 register overflows.
;
; TMR0 is never reloaded -- thus it wraps around and does a full count for each interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Int:

	bcf 	INTCON,T0IF     ; clear the Timer0 overflow interrupt flag

    ; do stuff here
    
    return

; end of handleTimer0Int
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortReceiveInt
;
; This function is called when a byte(s) has been received by the serial port. The byte(s) will be
; checked to see if it is a header byte, a packet length byte, or a data byte. Data bytes will be
; stored in a buffer. If an error occurs in receiving a packet, the function will ignore data
; received before the error and begin watching for the next packet signature. Upon receiving a
; complete packet, a flag will be set to notify the main loop.
;
; The receive register is a two byte fifo, so two bytes could be ready. This function will process
; all bytes available.
;
; The RCIF flag is cleared by reading all data from the two byte receive FIFO.
;
; This code check each byte sequence to see if it starts with a header prefix (0xaa,0x55) followed
; by a valid length byte. If these are found, the bytes after the length byte are stored in a
; buffer. If the sequence is not matched or the supposed length byte is larger than the buffer,
; all flags are reset and the search for the first header byte starts over.
;
; Packet format:
;   0xaa, 0x55, length, data1, data2, data3,...checksum.
;
; This interrupt function does not verify the checksum; the main loop should do that if required.
; Once a packet has been received, a flag is set to alert the main loop that it is ready for
; processing. All further data will be ignored until the main loop clears that flag. If an error
; occurs, the data received to that point will be discarded and the search for the next packet
; begun anew.
;
; The packet length byte is the number of data bytes plus one for the checksum byte. It does not
; include the two header bytes or the length byte itself. If the length byte value is 0 or is
; greater than the buffer size, the packet will be ignored. If the length byte value is greater
; than the actual number of bytes sent (but still less than the buffer size), the current packet
; AND the next packet(s) will be discarded as the interrupt routine will wait until enough bytes
; are received from subsequent packets to equal the erroneously large length byte value.
;
; Thus, only one packet at a time can be handled. The processing required is typically minimal, so
; the main loop should be able to process each packet before another is received. Some care should
; be taken by the receiver to not flood the line with packets.
;
; The main loop does all the actual processing in order to minimize the overhead of the interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleSerialPortReceiveInt:

    ; if the packet ready flag is set, ignore all data until main loop clears it

    banksel flags2
    btfss   flags2, SERIAL_PACKET_READY
    goto    readSerialLoop

    ; packet ready flag set means last packet still being processed, read byte to clear interrupt
    ; or it will result in an endless interrupt loop, byte is tossed and a resync will occur

    banksel RCREG
    movf    RCREG, W
    goto    rslExit

    ;RCREG is a two byte FIFO and may contain two bytes; read until RCIF flag is clear

readSerialLoop:

    banksel RCREG
    movf    RCREG, W        ; get byte from receive fifo

    banksel flags2

    btfsc   flags2, HEADER_BYTE_1_RCVD      ; header byte 1 already received?
    goto    rsl1                            ; if so, check for header byte 2

    bsf     flags2, HEADER_BYTE_1_RCVD      ; preset the flag, will be cleared on fail

    sublw   0xaa                            ; check for first header byte of 0xaa
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 1, reset all to restart search

rsl1:
    btfsc   flags2, HEADER_BYTE_2_RCVD      ; header byte 2 already received?
    goto    rsl2                            ; if so, check for length byte

    bsf     flags2, HEADER_BYTE_2_RCVD      ; preset the flag, will be cleared on fail

    sublw   0x55                            ; check for second header byte of 0x55
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 2, reset all to restart search

rsl2:
    btfsc   flags2, LENGTH_BYTE_VALID       ; packet length byte already received and validated?
    goto    rsl3                            ; if so, jump to store data byte

    movwf   serialRcvPktLen                 ; store the packet length
    movwf   serialRcvPktCnt                 ; store it again to count down number of bytes stored

    bsf     flags2, LENGTH_BYTE_VALID       ; preset the flag, will be cleared on fail

    movf    serialRcvPktLen, F              ; check for invalid packet size of 0
    btfsc   STATUS, Z
    goto    rslError

    subwf   serialRcvBufLen, W              ; check if packet length < buffer length
    btfsc   STATUS, C                       ; carry cleared if borrow was required
    goto    rsllp                           ; continue on, leaving flag set
                                            ; if invalid length, reset all to restart search

rslError:

    incf    serialPortErrorCnt, F           ; track errors
    bsf     statusFlags,SERIAL_COM_ERROR
    call    resetSerialPortRcvBuf    
    goto    rsllp

rsl3:

    movwf   serialIntScratch0               ; store the new character

    movf    serialRcvBufPtrH, W             ; load FSR0 with buffer pointer
    movwf   FSR0H
    movf    serialRcvBufPtrL, W
    movwf   FSR0L

    movf    serialIntScratch0, W            ; retrieve the new character
    movwi   INDF0++                         ; store in buffer

    movf    FSR0H, W                        ; save adjusted pointer
    movwf   serialRcvBufPtrH
    movf    FSR0L, W
    movwf   serialRcvBufPtrL

    decfsz  serialRcvPktCnt, F              ; count down number of bytes stored
    goto    rsllp                           ; continue collecting until counter reaches 0

rsl4:

    bsf     flags2, SERIAL_PACKET_READY     ; flag main loop that a data packet is ready
    goto    rslExit

rsllp:

    banksel PIR1                            ; loop until receive fifo is empty
    btfsc   PIR1, RCIF
    goto    readSerialLoop

rslExit:

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    noOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN

noOERRError:

    goto    endISR

; end of handleSerialPortReceiveInt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortTransmitInt
;
; This function is called when a byte is to be transmitted to the host via serial port. After
; data is placed in the transmit buffer, the TXIE flag is enabled so this routine gets called
; as an interrupt whenever the transmit buffer is empty. After all bytes in the buffer have been
; transmitted, this routine clears the TXIE flag to disable further interrupts.
;
; Before the TXIE flag is set to start the process, serialXmtBufNumBytes should be set to value
; > 0, i.e. the number of valid bytes in the transmit buffer.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;
; The TXIF flag is cleared in the second instruction cycle after writing data to TXREG.
;

handleSerialPortTransmitInt:

    banksel serialXmtBufPtrH                ; load FSR0 with buffer pointer
    movf    serialXmtBufPtrH, W
    movwf   FSR0H
    movf    serialXmtBufPtrL, W
    movwf   FSR0L

    moviw   FSR0++                          ; send next byte in buffer
    banksel TXREG
    movwf   TXREG

    banksel serialXmtBufPtrH                ; store updated FSR0 in buffer pointer
    movf    FSR0H, W
    movwf   serialXmtBufPtrH
    movf    FSR0L, W
    movwf   serialXmtBufPtrL

    decfsz  serialXmtBufNumBytes, F
    goto    endISR                          ; more data to send, exit with interrupt still enabled

    banksel PIE1                            ; no more data, disable further transmit interrupts
    bcf     PIE1, TXIE

    goto    endISR

; end of handleSerialPortTransmitInt
;--------------------------------------------------------------------------------------------------
;
;   End of EUSART Serial Port Core Functions
;
;--------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------

    END
