; LaserProject.asm
; @description A project for Computer Systems
; @author Andrew Siemer
; @version 1.14.20

.INCLUDE "M328PDEF.INC"

.EQU	LCD_PRT = PORTB		;LCD DATA PORT
.EQU	LCD_DDR = DDRB		;LCD DATA DDR
.EQU	LCD_PIN = PINB		;LCD DATA PIN
.EQU	LCD_RS = 0			;LCD RS
.EQU	LCD_RW = 1			;LCD RW
.EQU	LCD_EN = 2			;LCD EN

		LDI		R21,HIGH(RAMEND)
		OUT		SPH,R21			;set up stack
		LDI		R21,LOW(RAMEND)
		OUT		SPL,R21

		LDI		R21,0xFF
		OUT		LCD_DDR,R21		;LCD data port is output
		OUT		LCD_DDR,R21		;LCD command port is output

		LDI		R16,0x33		;init. LCD for 4-bit data
		CALL	CMNDWRT			;call command function
		CALL	DELAY_2ms		;init. hold
		LDI		R16,0x32		;init. LCD for 4-bit data
		CALL	CMNDWRT			;call command function
		CALL	DELAY_2ms		;init. hold
		LDI		R16,0x28		;init. LCD 2 lines,5x7 matrix
		CALL	CMNDWRT			;call command function
		CALL	DELAY_2ms		;init. hold
		LDI		R16,0x0E		;display on, cursor on
		CALL	CMNDWRT			;call command function
		LDI		R16,0x01		;clear LCD
		CALL	CMNDWRT			;call command function
		CALL	DELAY_2ms		;delay 2 ms for clear LCD
		LDI		R16,0x06		;shift cursor right
		CALL	CMNDWRT			;call command function
		
		LDI		R16,'H'			;display letter 'H'
		CALL	DATAWRT			;call data write function
		LDI		R16,'i'			;display letter 'i'
		CALL	DATAWRT			;call data write function

HERE:
		JMP HERE				;stay here
;---------------------------------------------------------
CMnDWRT:
		MOV		R27,R16
		ANDI	R27,0xF0
		IN		R26,LCD_PRT
		ANDI	R26,0x0F
		OR		R26,R27
		OUT		Lcd_PRT,R26			;LCD data port = R16
		CBI		LCD_PRT,LCD_RS		;RS = 0 for command
		CBI		LCD_PRT,LCD_RW		;RW = 0 for write
		SBI		LCD_PRT,LCD_EN		;EN = 1 for high pulse
		CALL	SDELAY				;make a wide EN pulse
		CBI		LCD_PRT,LCD_EN		;EN = 0 for H-to-L pulse

		CALL	DELAY_100us			;make a wide EN pulse

		MOV		R27,R16
		SWAP	R27
		ANDI	R27,0xF0
		IN		R26,LCD_PRT
		ANDI	R26,0x0F
		OR		R26,R27
		OUT		LCD_PRT,R26			;LCD data port = R16
		SBI		LCD_PRT,LCD_EN		;EN = 1 for high pulse
		CALL	SDELAY				;make a wide EN pulse
		CBI		LCD_PRT,LCD_EN		;EN = 0 for H-to-L pulse

		CALL	DELAY_100us			;wait 100 us
		RET
;---------------------------------------------------------
DATAWRT:
		MOV		R27,R16
		ANDI	R27,0xF0
		IN		R26,LCD_PRT
		ANDI	R26,0x0F
		OR		R26,R27
		OUT		LCD_PRT,R26			;LCD data port = R16
		SBI		LCD_PRT,LCD_RS		;RS = 1 for data
		CBI		LCD_PRT,LCD_RW		;RW = 0 for write
		SBI		LCD_PRT,LCD_EN		;EN = 1 for high pulse
		CALL	SDELAY				;make a wide EN pulse
		CBI		LCD_PRT,LCD_EN		;EN = 0 for H-to-L pulse

		MOV		R27,R16
		SWAP	R27
		ANDI	R27,0xF0
		IN		R26,LCD_PRT
		ANDI	R26,0x0F
		OR		R26,R27
		OUT		LCD_PRT,R26			;LCD data port = R16
		SBI		LCD_PRT,LCD_EN		;EN = 1 for high pulse
		CALL	SDELAY				;make a wide EN pulse
		CBI		LCD_PRT,LCD_EN		;EN = 0 for H-to-L pulse

		CALL	DELAY_100us			;wait 100 us
		RET
;---------------------------------------------------------
SDELAY:
		NOP
		NOP
		RET
;---------------------------------------------------------
DELAY_100us:
		PUSH	R17
		LDI		R17,60
DR0:	CALL	SDELAY
		DEC		R17
		BRNE	DR0
		POP		R17
		RET
;---------------------------------------------------------
DELAY_2ms:
		PUSH	R17
		LDI		R17,20
LDR0:	CALL	DELAY_100us
		DEC		R17
		BRNE	LDR0
		POP		R17
		RET