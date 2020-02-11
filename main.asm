;---------------------------------------------------------
; LASERPROJECT.ASM
; DESCRIPTION : A project for Computer Systems
; AUTHOR : Andrew Siemer
; VERSION : 2.11.20
;---------------------------------------------------------
; Registers:
; R16 - LCD buffer
; R17 - placeholder
; R18 - shift flag
;---------------------------------------------------------

.org 0x00		; Write next command at 0x00 (Reset)
jmp START		; Wakeup/reset
.org 0x02		; Write next command at 0x02 (INT0)
jmp INT0ROUTINE	; Call interrupt service routine for INT0

.org 0x300	; go to 0x300 in program memory
data1:.DB 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'

;---------------------------------------------------------
; INT0ROUTINE : Action on button press
;---------------------------------------------------------
INT0ROUTINE:
	CLI						; clear interrupt
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms		
	CALL	LOAD_KEYPAD		; load in keypad buffer
	LDI		R17, 0xFF
	CPSE	R16, R17		; check if button press was not a character
	CALL	LOADZREGISTER1	; load Z Register with database value
	CPSE	R16, R17		; check if button press was not a character
	CALL	DATAWRT			; Write diplay buffer to LCD
	SEI						; set enable interrupts
RJMP MAIN

;---------------------------------------------------------
; MAIN : Wait in loop when idle
;---------------------------------------------------------
MAIN:
	JMP		MAIN	; stay here

;---------------------------------------------------------
; CLEAR_SCREEN : Clear LCD Screen
;---------------------------------------------------------
CLEAR_SCREEN:
	LDI		R16, 0x01	; LCD clear command
	CALL	CMnDWRT		; LCD command write
	CALL	DELAY_2ms
RET

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
; LOAD_KEYPAD : Load R16 with keypad buffer
;---------------------------------------------------------
LOAD_KEYPAD:
	LDI		R16, 0x00
	OUT		KPD_DDR, R16	; set keypad port as output
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	CALL	DELAY_2ms
	IN		R16, KPD_PIN	; read in buffer from keypad
	ANDI	R16, 0x1F		; keypad buffer mask
	CPI		R16, 0x12		; look for shift key
	BREQ	Shift
RET

;---------------------------------------------------------
; SHIFT : Toggle uppercase/lowercase
;---------------------------------------------------------
Shift:
	CPI		R18, 1
	BREQ	LOWERCASE	; if uppercase call lowercase
	BRNE	UPPERCASE	; if lowercase call uppercase
RET

;---------------------------------------------------------
; LOWERCASE : Toggle on lowercase
;---------------------------------------------------------
LOWERCASE:
	LDI		R16, 0x0E	; set LCD cursor to not blinking
	CALL	CMNDWRT		; write command to LCD
	LDI		R18, 0		; set shift flag = 0
	LDI		R16, 0xFF	; fill R16 with 1 for LCD ignore
RET

;---------------------------------------------------------
; LOWERCASE : Toggle on uppercase
;---------------------------------------------------------
UPPERCASE:
	LDI		R16, 0x0F	; set LCD cursor to blinking
	CALL	CMNDWRT		; write command to LCD
	LDI		R18, 1		; set shift flag = 1
	LDI		R16, 0xFF	; fill R16 with 1 for LCD ignore
RET

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
; INTERRUPT_CONFIG : Establishes interrupt variables
;---------------------------------------------------------
INTERRUPT_CONFIG:
	LDI		R31, 0x0A	; preload binary 00001010 into r31
	LDI		R17, 0x00
	STS		EICRA, R31	; set eicra to 00001010 (both interrupts trigger on active low)
	LDI		R31, 0x03	; preload binary 00000011 into r31
	OUT		EIMSK, R31	; set eimsk to 00000011 (enable both interrupts)
	LDI		R31, 0x00	; preload binary 00000000 into r31
	OUT		DDRD, R31	; set ddrd to 00000000 (all pins of portd are input pins, note you only need pins 2 and 3 for the interrupts)
	LDI		R31, 0x0C	; preload binary 00001100 into r31
	OUT		PORTD, R31	; set portd to 00001100 (portd pins 2 and 3 are internally hooked to pull up resistors)
SEI						; set enable interrupts

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

;---------------------------------------------------------
; LCD_INIT : Initializes the LCD periphrials
;---------------------------------------------------------
LCD_INIT:
	LDI		R17, 0xFF
	OUT		LCD_DDR, R17	; LCD data port is output
	OUT		LCD_DDR, R17	; LCD command port is output

	LDI		R16, 0x33		; init. LCD for 4-bit data
	CALL	CMNDWRT			; call command function
	CALL	DELAY_2ms		; init. hold
	LDI		R16, 0x32		; init. LCD for 4-bit data
	CALL	CMNDWRT			; call command function
	CALL	DELAY_2ms		; init. hold
	LDI		R16, 0x28		; init. LCD 2 lines, 5x7 matrix
	CALL	CMNDWRT			; call command function
	CALL	DELAY_2ms		; init. hold
	LDI		R16, 0x0E		; display on, cursor on
	CALL	CMNDWRT			; call command function
	LDI		R16, 0x01		; clear LCD
	CALL	CMNDWRT			; call command function
	CALL	DELAY_2ms		; delay 2 ms for clear LCD
	LDI		R16, 0x06		; shift cursor right
	CALL	CMNDWRT			; call command function

	LDI R18, 0				; set current state of shift to lower

JMP MAIN	; setup complete wait for interrupt
;---------------------------------------------------------

;---------------------------------------------------------
; CMnDWRT : Write data in R16 to LCD as command
;---------------------------------------------------------
CMnDWRT:
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
; DATAWRT : Write data in R16 to LCD
;---------------------------------------------------------
DATAWRT:
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
; DELAY_2ms : 100 millisecond delay
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
