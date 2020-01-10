; LaserProject.asm
;
; Created: 1/7/20 9:54:30 AM
; Author : andrewsiemer
;

.INCLUDE "M328PDEF.INC"
.EQU	LCD_DPRT = PORTD	;LCD DATA PORT
.EQU	LCD_DDDR = DDRD		;LCD DATA DDR
.EQU	LCD_DPIN = PIND		;LCD DATA PIN
.EQU	LCD_CPRT = PORTB	;LCD COMMANDS PORT
.EQU	LCD_CDDR = DDRB		;LCD COMMAND DDR
.EQU	LCD_CPIN = PINB		;LCD COMMANDS PIN
.EQU	LCD_RS = 0			;LCD RS
.EQU	LCD_RW = 1			;LCD RW
.EQU	LCD_EN = 2			;LCD EN

		LDI		R21, HIGH (RAMEND)
		OUT		SPH, R21
		LDI		R21, LOW (RAMEND)
		OUT		SPL, R21

		LDI		R21,0xFF;
		OUT		LCD_DDDR, R21	;LCD data port is output
		OUT		LCD_CDDR, R21	;LCD command port is output
		CBI		LCD_CPRT, LCD_EN;LCD_EN = 0
		CALL	DELAY_2ms		;wait for power on
		LDI		R16, 0x38		;init LCD 2 lines, 5x7 matrix
		CALL	CMNDWRT			;call command function
		CALL	DELAY_2ms		;wait 2 ms
		LDI		R16, 0x0E		;display on, cursor on
		CALL	CMNDWRT			;call command function
		LDI		R16, 0x01		;clear LCD
		CALL	CMNDWRT			;call command function
		CALL	DELAY_2ms		;wait 2 ms
		LDI		R16, 0x06		;shift cursor right
		CALL	CMNDWRT			;call command function
		LDI		R16, 'H'		;display letter 'H'
		CALL	DATAWRT			;call data write function
		LDI		R16, 'i'		;display letter 'i'
		CALL	DATAWRT			;call data write function
HERE:	JMP HERE				;stay here
;---------------------------------------------------------
CMNDWRT:
		OUT		LCD_DPRT, R16		;LCD data port = R16
		CBI		LCD_CPRT, LCD_RS	;RS = 0 for command
		CBI		LCD_CPRT, LCD_RW	;RW = 0 for write
		SBI		LCD_CPRT,LCD_EN		;EN = 1
		CALL	SDELAY				;make a wide EN pulse
		CBI		LCD_CPRT, LCD_EN	;EN = 0 for H-to-L pulse
		CALL	DELAY_100us			;wait 100 us
		RET

DATAWRT:
		OUT		LCD_DPRT, R16		;LCD data port = R16
		SBI		LCD_CPRT, LCD_RS	;RS = 1 for data
		CBI		LCD_CPRT, LCD_RW	;RW = 0 for write
		SBI		LCD_CPRT, LCD_EN	;EN = 1
		CALL	SDELAY				;make a wide EN pulse
		CBI		LCD_CPRT, LCD_EN	;EN = 0 for H-to-L pulse
		CALL	DELAY_100us			;wait 100 us
		RET
;---------------------------------------------------------
SDELAY:	NOP
		NOP
		RET
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
DELAY_2ms:
		PUSH	R17
		LDI		R17, 20
LDR0:	CALL	DELAY_100us
		DEC		R17
		BRNE	LDR0
		POP		R17
		RET
