;---------------------------------------------------------
; LASERPROJECT.ASM
; DESCRIPTION : A project for Computer Systems
; AUTHOR : Andrew Siemer
; VERSION : 3.6.20
;---------------------------------------------------------
; Registers:
; R16 - LCD buffer
; R17 - general use
; R18 - shift flag
; R19 - general use
; R20 - mode flag
;---------------------------------------------------------

.org 0x00	; Write next command at 0x00 (Reset)
jmp START	; Wakeup/reset
.org 0x02	; Write next command at 0x02 (INT0)
jmp INT0ROUTINE	; Call interrupt service routine for INT0

.org 0x300	; go to 0x300 in program memory
data1:.DB 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'

;---------------------------------------------------------
START:	; start of program

;---------------------------------------------------------
; STACK_CONFIG : Establishes stack variables
;---------------------------------------------------------
STACK_CONFIG:
	LDI		R17, HIGH(RAMEND)
	OUT		SPH, R17			; set up stack
	LDI		R21, LOW(RAMEND)
	OUT		SPL, R17

;---------------------------------------------------------
; INT_CONFIG : Establishes interrupt variables
;---------------------------------------------------------
INT_CONFIG:
	LDI		R19, 0x0A	; preload binary 00001010 into R19
	LDI		R17, 0x00
	STS		EICRA, R19	; set eicra to 00001010 (both interrupts trigger on active low)
	LDI		R19, 0x03	; preload binary 00000011 into R19
	OUT		EIMSK, R19	; set eimsk to 00000011 (enable both interrupts)
	LDI		R19, 0x00	; preload binary 00000000 into R19
	OUT		DDRD, R19	; set ddrd to 00000000 (all pins of portd are input pins, note you only need pins 2 and 3 for the interrupts)
	LDI		R19, 0x0C	; preload binary 00001100 into R19
	OUT		PORTD, R19	; set portd to 00001100 (portd pins 2 and 3 are internally hooked to pull up resistors)
	SEI					; set enable interrupts

;---------------------------------------------------------
; KEYPAD_CONFIG : Establishes keypad variables
;---------------------------------------------------------
KEYPAD_CONFIG:
	.EQU	KPD_PRT = PORTC	; keypad data port
	.EQU	KPD_DDR = DDRC	; keypad ddr
	.EQU	KPD_PIN = PINC	; keypad data pin
	LDI		R17, 0x00
	OUT		KPD_DDR, R17
	OUT		KPD_PRT, R17

;---------------------------------------------------------
; LCD_CONFIG : Establishes LCD variables
;---------------------------------------------------------
LCD_CONFIG:
	.EQU	LCD_PRT = PORTB	; LCD data port
	.EQU	LCD_DDR = DDRB	; LCD ddr
	.EQU	LCD_PIN = PINB	; LCD data pin
	.EQU	LCD_RS = 0		; LCD RS
	.EQU	LCD_RW = 1		; LCD RW
	.EQU	LCD_EN = 2		; LCD EN
	.EQU	LCD_BUF1 = 0x200	; LCD char buffer 1
	.EQU	LCD_BUF2 = 0x208	; LCD char buffer 2
	.EQU	LCD_BUF3 = 0x210	; LCD char buffer 3
	.EQU	LCD_BUF4 = 0x218	; LCD char buffer 4

;---------------------------------------------------------
; KEYPAD_INIT : Initializes the keypad periphrials
;---------------------------------------------------------
KEYPAD_INIT:
	LDI		R20, '0'	; set mode flag to 0

;---------------------------------------------------------
; LCD_INIT : Initializes the LCD periphrials
;---------------------------------------------------------
LCD_INIT:
	LDI		R17, 0xFF
	OUT		LCD_DDR, R17	; LCD data port is output
	OUT		LCD_DDR, R17	; LCD command port is output

	LDI		R16, 0x33		; init. LCD for 4-bit data
	CALL	LCD_CMD			; call command function
	CALL	DELAY_2ms		; init. hold
	LDI		R16, 0x32		; init. LCD for 4-bit data
	CALL	LCD_CMD			; call command function
	CALL	DELAY_2ms		; init. hold
	LDI		R16, 0x28		; init. LCD 2 lines, 5x7 matrix
	CALL	LCD_CMD			; call command function
	CALL	DELAY_2ms		; init. hold
	LDI		R16, 0x0E		; display on, cursor on
	CALL	LCD_CMD			; call command function
	LDI		R16, 0x01		; clear LCD
	CALL	LCD_CMD			; call command function
	CALL	DELAY_2ms		; delay 2 ms for clear LCD
	LDI		R16, 0x06		; shift cursor right
	CALL	LCD_CMD			; call command function
	LDI		R18, 0			; set current state of shift to lower
	LDI		R17, ' '
	STS		LCD_BUF1, R17	; init. buffer with spaces
	STS		LCD_BUF2, R17	; init. buffer with spaces
	STS		LCD_BUF3, R17	; init. buffer with spaces
	STS		LCD_BUF4, R17	; init. buffer with spaces
	CALL	LCD_UPDATE

RJMP MAIN	; setup complete wait for interrupt
;---------------------------------------------------------

;---------------------------------------------------------
; MAIN : Wait in loop when idle
;---------------------------------------------------------
MAIN:
	RJMP	MAIN	; stay here

;---------------------------------------------------------
; INT0ROUTINE : Action on button press
;---------------------------------------------------------
INT0ROUTINE:
	CLI						; clear interrupt		
	CALL	KEYPAD_LOAD		; load in keypad buffer
	CPI		R16, 0x12		; look for shift key
	BREQ	SHIFT_JMP
	CPI		R16, 0x13		; look for mode key
	BREQ	MODE_JMP
	CALL	LOADZREGISTER1	; load Z Register with database value
	CALL	KEYBUFF_LOAD	; load LCD character buffer
	INT_BREAK:
	CALL	LCD_UPDATE		; write diplay buffer to LCD
	SEI						; set enable interrupts
RETI

;---------------------------------------------------------
; LOADZREGISTER1 : Load Z register with database values
;---------------------------------------------------------
LOADZREGISTER1:
	LDI		ZL,	LOW(2*data1)
	LDI		ZH,	HIGH(2*data1)
	LDI		R21, 0
	LDI		R22, 18
	CPSE	R18, R21
	ADD		R16, R22
	ADD		ZL, R16
	LPM		R16, Z
RET

;---------------------------------------------------------
; KEYPAD_LOAD : Load R16 with keypad buffer
;---------------------------------------------------------
KEYPAD_LOAD:
	LDI		R16, 0x00
	OUT		KPD_DDR, R16	; set keypad port as output
	
	LDI		R17, 10
	LOOP:
		CALL	DELAY_2ms
		DEC		R17
	BRNE	LOOP

	IN		R16, KPD_PIN	; read in buffer from keypad
	ANDI	R16, 0x1F		; keypad buffer mask
RET

;---------------------------------------------------------
; SHIFT_JMP : JMP to SHIFT since its outside of relative range
;---------------------------------------------------------
SHIFT_JMP:
	JMP SHIFT

;---------------------------------------------------------
;  MODE_JMP : JMP to MODE since its outside of relative range
;---------------------------------------------------------
MODE_JMP:
	JMP MODE

;---------------------------------------------------------
; KEYBUFF_LOAD : Load new keypad value into buffer
;---------------------------------------------------------
KEYBUFF_LOAD:
	LDS		R17, LCD_BUF3	; load buffer value into R17
	STS		LCD_BUF4, R17	; write R17 value to buffer memory
	LDS		R17, LCD_BUF2	; load buffer value into R17
	STS		LCD_BUF3, R17	; write R17 value to buffer memory
	LDS		R17, LCD_BUF1	; load buffer value into R17
	STS		LCD_BUF2, R17	; write R17 value to buffer memory
	STS		LCD_BUF1, R16	; write R17 value to buffer memory
RET

;---------------------------------------------------------
; LCD_UPDATE : Updates LCD with new char buffer
;---------------------------------------------------------
LCD_UPDATE:
	CALL	LCD_CLR	

	LDI		R16, 'M'
	CALL	LCD_WRT			; write R16 to LCD
	LDI		R16, 'o'
	CALL	LCD_WRT			; write R16 to LCD
	LDI		R16, 'd'
	CALL	LCD_WRT			; write R16 to LCD
	LDI		R16, 'e'
	CALL	LCD_WRT			; write R16 to LCD
	LDI		R16, ':'
	CALL	LCD_WRT			; write R16 to LCD
	LDI		R16, ' '
	CALL	LCD_WRT			; write R16 to LCD
	MOV		R16, R20
	CALL	LCD_WRT			; write R16 to LCD

	LDI		R16, 0x14		; shift cursor right

	LDI		R17, 10
	MOVE_CURSOR:
		CALL	LCD_CMD
		DEC		R17
	BRNE	MOVE_CURSOR

	LDS		R16, LCD_BUF4	; load buffer value into R16
	CALL	LCD_WRT			; write R16 to LCD
	LDS		R16, LCD_BUF3	; load buffer value into R16
	CALL	LCD_WRT			; write R16 to LCD
	LDS		R16, LCD_BUF2	; load buffer value into R16
	CALL	LCD_WRT			; write R16 to LCD
	LDS		R16, LCD_BUF1	; load buffer value into R16
	CALL	LCD_WRT			; write R16 to LCD
RET

;---------------------------------------------------------
; SHIFT : Toggle uppercase/lowercase
;---------------------------------------------------------
Shift:
	CPI		R18, 1		; compare shift flag
	BREQ	LOWERCASE	; if uppercase call lowercase
	BRNE	UPPERCASE	; if lowercase call uppercase

;---------------------------------------------------------
; LOWERCASE : Toggle on lowercase
;---------------------------------------------------------
LOWERCASE:
	LDI		R16, 0x0E	; set LCD cursor to not blinking
	CALL	LCD_CMD		; write command to LCD
	LDI		R18, 0		; set shift flag = 0
	RJMP	INT_BREAK

;---------------------------------------------------------
; LOWERCASE : Toggle on uppercase
;---------------------------------------------------------
UPPERCASE:
	LDI		R16, 0x0F	; set LCD cursor to blinking
	CALL	LCD_CMD		; write command to LCD
	LDI		R18, 1		; set shift flag = 1
	RJMP	INT_BREAK

;---------------------------------------------------------
; MODE : Toggle mode state
;---------------------------------------------------------
MODE:
	CPI		R20, '0'
	BREQ	MODE_1
	CPI		R20, '1'
	BREQ	MODE_2
	CPI		R20, '2'
	BREQ	MODE_3
	CPI		R20, '3'
	BREQ	MODE_0

	MODE_0:
		LDI		R20, '0'
		RJMP	INT_BREAK
	MODE_1:
		LDI		R20, '1'
		RJMP	INT_BREAK
	MODE_2:
		LDI		R20, '2'
		CALL	LASER_WRT
		RJMP	INT_BREAK
	MODE_3:
		LDI		R20, '3'
		RJMP	INT_BREAK	

;---------------------------------------------------------
; LCD_CLR : Clear LCD Screen
;---------------------------------------------------------
LCD_CLR:
	LDI		R16, 0x01	; LCD clear command
	CALL	LCD_CMD		; LCD command write
	CALL	DELAY_2ms
RET

;---------------------------------------------------------
; LCD_CMD : Write data in R16 to LCD as command
;---------------------------------------------------------
LCD_CMD:
	MOV		R27, R16
	ANDI	R27, 0xF0
	IN		R26, LCD_PRT
	ANDI	R26, 0x0F
	OR		R26, R27
	OUT		Lcd_PRT, R26		; LCD data port = R16
	CBI		LCD_PRT, LCD_RS		; RS = 0 for command
	CBI		LCD_PRT, LCD_RW		; RW = 0 for write
	SBI		LCD_PRT, LCD_EN		; EN = 1 for high pulse
	CALL	SDELAY				; make a wide EN pulse
	CBI		LCD_PRT, LCD_EN		; EN = 0 for H-to-L pulse

	CALL	DELAY_100us			; make a wide EN pulse

	MOV		R27, R16
	SWAP	R27
	ANDI	R27, 0xF0
	IN		R26, LCD_PRT
	ANDI	R26, 0x0F
	OR		R26, R27
	OUT		LCD_PRT, R26		; LCD data port = R16
	SBI		LCD_PRT, LCD_EN		; EN = 1 for high pulse
	CALL	SDELAY				; make a wide EN pulse
	CBI		LCD_PRT, LCD_EN		; EN = 0 for H-to-L pulse

	CALL	DELAY_100us			; wait 100 us
RET

;---------------------------------------------------------
; LCD_WRT : Write data in R16 to LCD
;---------------------------------------------------------
LCD_WRT:
	MOV		R27, R16
	ANDI	R27, 0xF0
	IN		R26, LCD_PRT
	ANDI	R26, 0x0F
	OR		R26, R27
	OUT		LCD_PRT, R26		; LCD data port = R16
	SBI		LCD_PRT, LCD_RS		; RS = 1 for data
	CBI		LCD_PRT, LCD_RW		; RW = 0 for write
	SBI		LCD_PRT, LCD_EN		; EN = 1 for high pulse
	CALL	SDELAY				; make a wide EN pulse
	CBI		LCD_PRT, LCD_EN		; EN = 0 for H-to-L pulse

	MOV		R27, R16
	SWAP	R27
	ANDI	R27, 0xF0
	IN		R26, LCD_PRT
	ANDI	R26, 0x0F
	OR		R26, R27
	OUT		LCD_PRT, R26		; LCD data port = R16
	SBI		LCD_PRT, LCD_EN		; EN = 1 for high pulse
	CALL	SDELAY				; make a wide EN pulse
	CBI		LCD_PRT, LCD_EN		; EN = 0 for H-to-L pulse

	CALL	DELAY_100us			; wait 100 us
RET

;---------------------------------------------------------
; SDELAY : Pulse delay for LCD
;---------------------------------------------------------
SDELAY:
		NOP
		NOP
RET

;---------------------------------------------------------
; DELAY_100us : 100 microsecond delay
;---------------------------------------------------------
DELAY_100us:
		PUSH	R17
		LDI		R17, 60
DR0:	CALL	SDELAY
		DEC		R17
		BRNE	DR0
		POP		R17
RET

;---------------------------------------------------------
; DELAY_2ms : 2 millisecond delay
;---------------------------------------------------------
DELAY_2ms:
		PUSH	R17
		LDI		R17, 20
LDR0:	CALL	DELAY_100us
		DEC		R17
		BRNE	LDR0
		POP		R17
RET

;---------------------------------------------------------

LASER_WRT:
		LDI R21, 0xFF
		OUT DDRC, R21
		LDI R21, 0x00		;Initialization, Port is set to all 0's
		OUT PortC, R21
		LDI R21, 0xFF		;Set PortB as output
		OUT DDRB, R21
		LDI R21, 0b00000011 ;Set X and Y -Buffer
		OUT PortC, R21
		LDI R21, 0x00		;Clear X and Y -Buffer
		OUT PortB, R21
		LDI R21, 0b00000000	;Finish setting X and Y
		OUT PortC, R21
	LOP:	
		LDI R16, 0x00
		CALL XBUFF_SET
		CALL LASER_UPDATE
		LDI R16, 0x00
		CALL YBUFF_SET
		CALL LASER_UPDATE

		CALL DELAY_100us
		CALL DELAY_100us
		CALL DELAY_100us
		CALL DELAY_100us

		LDI R16, 0x00
		CALL XBUFF_SET
		CALL LASER_UPDATE
		LDI R16, 0xFF
		CALL YBUFF_SET
		CALL LASER_UPDATE

		CALL DELAY_100us
		CALL DELAY_100us
		CALL DELAY_100us
		CALL DELAY_100us
	RJMP LOP

	XBUFF_SET:
		LDI R21, 0b00000001
		OUT PortC, R21
		CALL DELAY_100us
	RET
	
	YBUFF_SET:
		LDI R21, 0b00000010
		OUT PortC, R21
		CALL DELAY_100us
	RET
	
	LASER_UPDATE:					;Sets the position of the one of the buffers
		OUT PortB, R16
		CALL DELAY_100us
		CALL DELAY_100us
	RET
