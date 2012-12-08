;=============================================================
; title			: eMushibue
; author		: Nobuhiro Kuroiwa
; started on	: 11/11/2012
; clock			: clk=8MHz
;=============================================================
; debug directive
;#define	DEBUG

; chip select
;#define			ATMEGA168	; Atmega168
#define			ATTINY45	; AtTiny45

; melody data select
; uncomment one of followings or ALL
;#define		XMAS		; We wish you a merry christmas
;#define		DONSUKA		; Donsuka pan pan ouendan
;#define		SPERUNKER	; Sperunker
;#define		JAWS		; Jaws theme
#define			ALL			; play all

.include "musicnote.inc"	; definition of music stuff

;=============================================================
; ports and pins
;=============================================================
#ifdef ATMEGA168
.include "m168def.inc"	;
.equ PRT_LV		= portb	; port for level indicator
.equ DDR_LV		= ddrb	; ddr  for PRT_LV
.equ PRT_VOL	= portd	; port for volume pwm
.equ DDR_VOL	= ddrd	; ddr for PRT_VOL
.equ PIN_VOL	= 1		; pin for above
.equ PRT_SND	= portd	; port for sound pwm
.equ DDR_SND	= ddrd	; ddr for PRT_SND
.equ PIN_SND	= 0		; pin for above
.equ PRT_ACCX	= portc	; port for x-axis accelerometer read
.equ DDR_ACCX	= ddrc	; ddr for PRT_ACCX
.equ PIN_ACCX	= 0		; pin for above
.equ PRT_ACCY	= portc	; pin for y-axis accelerometer read
.equ DDR_ACCY	= ddrc	; ddr for PRT_ACCY
.equ PIN_ACCY	= 1		; pin for above
.equ TIMERMASK	= TIMSK0
.equ ADCSRAVAL	= (1<<ADEN)|(1<<ADSC)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
						; adc setting
						; max adc takes 400us, etc
.equ ADMUXVAL	= (1<<REFS0)|(1<<ADLAR)	; adc ref voltage=AVcc, 8bit precision
#endif

#ifdef ATTINY45
.include "tn45def.inc"
.equ PRT_LV		= portb	; port for level indicator
.equ DDR_LV		= ddrb	; ddr  for PRT_LV
.equ PIN_LV		= 3
.equ PRT_VOL	= portb	; port for volume pwm
.equ DDR_VOL	= ddrb	; ddr for PRT_VOL
.equ PIN_VOL	= 1		; pin for above
.equ PRT_SND	= portb	; port for sound pwm
.equ DDR_SND	= ddrb	; ddr for PRT_SND
.equ PIN_SND	= 0		; pin for above
.equ PRT_ACCX	= portb	; port for x-axis accelerometer read
.equ DDR_ACCX 	= ddrb	; ddr for PRT_ACCX
.equ PIN_ACCX	= 2		; pin for above
.equ PRT_ACCY	= portb	; pin for y-axis accelerometer read
.equ DDR_ACCY  	= ddrb	; ddr for PRT_ACCY
.equ PIN_ACCY	= 4		; pin for above
.equ TIMERMASK	= TIMSK
.equ ADCSRAVAL	= (1<<ADEN)|(1<<ADSC)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
						; adc setting
						; max adc takes 400us, etc
.equ ADMUXVAL	= (1<<ADLAR)|(1<<MUX0)	; adc ref vref=internal, 8bit precision
#endif

;=============================================================
; constants
;=============================================================
#ifdef DEBUG
.equ SENSITIVITY = 26	; sensitivity
#else
.equ SENSITIVITY = 0	; sensitivity
#endif
.equ T10USEC	= 248	; Pre Scale=1/8, 100KHz
.equ PRE_SCALE	= 0x2	; 1/8
.equ VCNTMAX	= 20	; max valu of vcnt
.equ ZEROGREADX	= 128	; 0g value of accerelometer read along x-axis
.equ ZEROGREADY	= 128	; same for y-axis

;=============================================================
; variables
;=============================================================
.def sreg_save	= r0	; to store sreg
.def one		= r1	; constant 1
.def ten		= r2	; constant 10
.def t10us		= r3	; count for 10us
.def t100us		= r4	; count for 100us
.def t1ms		= r5	; count for 1ms
.def t10ms		= r6	; count for 10ms
.def t100ms		= r7	; count for 100ms
.def t1s		= r8	; count for 1s
.def sctop		= r9	; sound interval count
.def mcnt		= r10	; t100ms counter
.def mtop		= r11	; tcnt top value
.def scnt		= r12	; compare to scnt for rythm
.def vcnttop	= r13	; volume count top
.def vcnt		= r14	; volume count
.def vval		= r16	; last applied voltage displacement
.def vread		= r17	; voltage displacement read in process
.def acc		= r18	; accumulator
.def acc2		= r19	; accumulator2

;=============================================================
; macro
;=============================================================
; preserve registers
.macro StoreRegs
	in		sreg_save, SREG	; preserve status
	push	sreg_save
	push	acc				; preserve acc
	push	acc2			; preserve acc2
.endmacro

.macro RestoreRegs
	pop		acc2			; restore acc2
	pop		acc				; restore acc
	pop		sreg_save
	out		SREG, sreg_save	; restore sreg
.endmacro

; time count
.macro TimeCount		; TIME_COUNT @0 @1
	inc		@0			; increment register given by @0
	cp		@0, ten		; compare the register
	brne	@1			; if the register != 10 jump to @1
	clr		@0			; clear the register
.endmacro

; flip port output
.macro FlipOut			; FLIP_PORT portx, port_bit
	in		acc, @0
	ldi		acc2, @1	; bit mask
	eor		acc, acc2
	out		@0, acc		; output
.endmacro

; usage: InReg reg, addr 
.macro InReg 
	.if @1 < 0x40 
		in @0, @1 
	.elif ((@1 >= 0x60) && (@1 < SRAM_START)) 
         lds @0,@1 
	.else 
		.error "InReg: Invalid I/O register address" 
	.endif 
.endmacro 

; usage: OutReg addr, reg 
.macro OutReg 
	.if @0 < 0x40 
		out @0, @1 
	.elif ((@0 >= 0x60) && (@0 < SRAM_START)) 
		sts @0,@1 
	.else 
		.error "OutReg: Invalid I/O register address" 
	.endif 
.endmacro 

;=============================================================
; program
;=============================================================
.cseg					   ; Code segment

;=============================================================
; vectors
;=============================================================
#ifdef ATMEGA168
.org 	0x0000		rjmp main	 	; reset handler
.org	0x0020		rjmp intr_time0	; timer0 overflow handler
.org	0x002A		rjmp adc_comp	; adc complete
#endif
#ifdef ATTINY45
.org 	0x0000		rjmp main	 	; reset handler
.org	0x0005		rjmp intr_time0	; timer0 overflow handler
.org	0x0008		rjmp adc_comp	; adc complete
#endif

;=============================================================
; main
;=============================================================
main:
	cli

	; initialize stack pointer
	ldi		acc, low(ramend)	; get lower byte of end of ram address
	out		spl, acc			; init stack lower pointer
	ldi		acc, high(ramend)	; get higher byte of end of ram address
	out		sph, acc			; init stack higher pointer

	; initialize constant register
	ldi		acc, 1			; put constant 1 in register
	mov		one, acc
	ldi		acc, 10			; put constant 10 in register
	mov		ten, acc

	; initialize ports
#ifdef ATMEGA168
	ldi	 	acc, 0xFF		; P_LV all bits are output
	out	 	DDR_LV, acc		; set direction
	ldi	 	acc, 0x00		; set output data
	out	 	PRT_LV, acc		; set all to low
#endif
#ifdef ATTINY45
	sbi		DDR_SND, PIN_SND
	sbi		DDR_VOL, PIN_VOL
	sbi		DDR_LV, PIN_LV
	cbi		DDR_ACCX, PIN_ACCX
	cbi		DDR_ACCY, PIN_ACCY
#endif

	; Timer/Counter 0 initialize
	; tccr0a=0, standard mode
	ldi		acc, 0
	sbr	 	acc,(1<<TOIE0)	; set overflow interruption bit
	OutReg	TIMERMASK, acc	; allow timer0 overflow interruption
	ldi	 	acc, T10USEC	; 10us count
	out	 	TCNT0, acc		; set timer0 counter
	ldi	 	acc, PRE_SCALE	; set prescale
	out	 	TCCR0B, acc		; start timer0

	; initialize our counters
	clr	 	t10us			; init counter for 10us
	clr	 	t100us			; init counter for 100usr
	clr	 	t1ms			; init counter for 1ms
	clr		t10ms			; init counter for 10ms
	clr		t100ms
	clr		t1s

	; initialize sound interval counter
	clr		sctop			; sound interval count

	; initialize melody counter
	clr		mcnt			; initialize melody counter

	; load data
	ldi		zl, low(SNDDATA<<1)		; init zl
	ldi		zh, high(SNDDATA<<1)	; init zh

	lpm		mtop, z+		; initialize tcnt compare value
	lpm		sctop, z+		; count untill scnt becomes this value

	; initialize volume counter
	ldi		acc, 0
	mov		vcnttop, acc
	clr		vcnt

	; initialize adc
	clr		vread
	ldi		acc, ADMUXVAL
	OutReg	ADMUX, acc
	ldi		acc, ADCSRAVAL
	OutReg	ADCSRA, acc
	ldi		acc, 0x00
	OutReg	ADCSRB, acc

	sei						; allow all interruption

main_loop:

	rjmp	main_loop		; loop

;=============================================================
; timer0 interruption
;=============================================================
intr_time0:
	StoreRegs

	; reset timer
	clr		acc				; stop counter
	out		tccr0b, acc
	ldi		acc, T10USEC	; 10usec count
	out		TCNT0, acc		; set timer0

	ldi		acc, PRE_SCALE
	out		tccr0b, acc

	TimeCount	t10us,	intr_time0_sndpwm	; count wrap around for 10us
	TimeCount	t100us,	intr_time0_sndpwm	; count wrap around for 100us
	rcall		vol_ctrl					; call every 100us
	TimeCount	t1ms,	intr_time0_sndpwm	; count wrap around for 1ms
	TimeCount	t10ms,	intr_time0_sndpwm	; count wrap around for 10ms
	TimeCount	t100ms,	initr_time0_setsnd	; count wrap around for 100ms
	TimeCount	t1s,	initr_time0_setsnd	; count wrap around for 1s

initr_time0_setsnd:
	; set sound frequency
	rcall	set_snd							; called every 100ms

	; request adc interruption
	InReg	acc, ADMUX
	ldi		acc2, (1<<MUX1)|(1<<MUX0)
	eor		acc, acc2
	OutReg	ADMUX, acc
	ldi		acc, ADCSRAVAL
	OutReg	ADCSRA, acc

intr_time0_sndpwm:
	rcall	snd_pwm

intr_time0_end:

	RestoreRegs
	reti

;=============================================================
; volume control
;=============================================================
vol_ctrl:
	inc		vcnt
	cp		vcnt, vcnttop
	brlt	vol_ctrl_on	; if vcnt < vcnttop, goto vol_ctrl_high
	rjmp	vol_ctrl_off
vol_ctrl_on:
	sbi		PRT_VOL, PIN_VOL
	rjmp	vol_ctrl_vcnt
vol_ctrl_off:
	cbi		PRT_VOL, PIN_VOL
	rjmp	vol_ctrl_vcnt
vol_ctrl_vcnt:
	ldi		acc, VCNTMAX
	cp		vcnt, acc
	brne vol_ctrl_ext
	clr		vcnt
vol_ctrl_ext:
	ret

;=============================================================
; set sound frequency
;=============================================================
set_snd:
	inc		mcnt
	cp		mtop, mcnt
	brne	set_snd_exit	; if mtop!=mcnt, do nothing
	clr		mcnt

	; check more data left
	cpi		zl, low(SNDDATA_END<<1)
	brne	set_snd_asgn
	cpi		zh, high(SNDDATA_END<<1)
	brne	set_snd_asgn

	; if data is end, reset pointer with head position
	ldi		zl, low(SNDDATA<<1)
	ldi		zh, high(SNDDATA<<1)

set_snd_asgn:
	lpm		mtop, z+		; initialize tcnt compare value
	lpm		sctop, z+		; count untill scnt becomes this value
	clr		scnt
set_snd_exit:
	ret

;=============================================================
; sound frequency pwm
;=============================================================
snd_pwm:
	clc
	adc		scnt, one
	brcc	snd_pwm1
	clc
	adc		scnt, one
	brcc	snd_pwm1
	clc
snd_pwm1:
	cp		sctop, scnt
	brne	snd_pwm_ext
	FlipOut	PRT_SND, 1<<PIN_SND
	clr		scnt
snd_pwm_ext:
	ret

;=============================================================
; adc_complete
;=============================================================
adc_comp:
	StoreRegs
	rcall	readv				; read voltage
	RestoreRegs
	reti

;=============================================================
; readv
;=============================================================
readv:
	ldi		acc2, ZEROGREADX	; use x-axis neutral value
	InReg	acc, ADMUX
	sbrc	acc, 0
	ldi		acc2, ZEROGREADY	; use y-axis neutral value

	clc
	InReg	acc, ADCH			; read D/A converted value
	sub		acc, acc2			; acc-netral value
	brpl	readv_positive		; if acc-acc2 > 0, goto readv_positive
	neg		acc					; else take absolute value
readv_positive:
	InReg	acc2, ADMUX
	sbrc	acc2, 0
	rjmp	readv_y
readv_x:
	mov		vread, acc
	ret
readv_y:
	add		vread, acc
	add		vval, vread
	lsr		vval
readv_compare:
	cpi		vval, 26-SENSITIVITY
	brlt	readv_level0
	cpi		vval, 28-SENSITIVITY
	brlt	readv_level1
	cpi		vval, 30-SENSITIVITY
	brlt	readv_level2
	cpi		vval, 32-SENSITIVITY
	brlt	readv_level3
	cpi		vval, 34-SENSITIVITY
	brlt	readv_level4
	rjmp	readv_level5
readv_level0:
	ldi		acc, 0
	ldi		acc2, 0b0000_0001
	rjmp readv_ext
readv_level1:
	ldi		acc, 5
	ldi		acc2, 0b0000_0011
	rjmp readv_ext
readv_level2:
	ldi		acc, 6
	ldi		acc2, 0b0000_0111
	rjmp readv_ext
readv_level3:
	ldi		acc, 7
	ldi		acc2, 0b0000_1111
	rjmp readv_ext
readv_level4:
	ldi		acc, 8
	ldi		acc2, 0b0001_1111
	rjmp readv_ext
readv_level5:
	ldi		acc, 20
	ldi		acc2, 0b0011_1111
	rjmp readv_ext
readv_ext:
	mov		vcnttop, acc
	clr		vcnt
#ifdef ATMEGA168
	out		;PRT_LV, acc2
#endif
#ifdef ATTINY45
	sbi		PRT_LV, PIN_LV
#endif
	ret

;=============================================================
; data
;=============================================================
#if defined(XMAS) || defined(ALL)
SNDDATA:
	.db NOTE_8, TONE_2G
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2B
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_1A

	.db NOTE_8, TONE_2A
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_1D
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2C
	.db NOTE_8, TONE_1B
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_0G

	.db NOTE_8, TONE_1G
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_1E
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2D
	.db NOTE_8, TONE_2C
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_0A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_8, TONE_1A
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1B
	.db NOTE_4, TONE_2C

	.db NOTE_8, TONE_2G
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2B
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_1A

	.db NOTE_8, TONE_2A
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_1D
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2C
	.db NOTE_8, TONE_1B
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_0G

	.db NOTE_8, TONE_1G
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_1E
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2D
	.db NOTE_8, TONE_2C
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_0A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_8, TONE_1A
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1B
	.db NOTE_4, TONE_2C

	.db NOTE_8, TONE_1G
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_1C
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_1C
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_1C
	.db NOTE_4, TONE_1B
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_2C
	.db NOTE_4, TONE_1B
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_2C
	.db NOTE_8, TONE_2G
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_0G
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_0G
	.db NOTE_8, TONE_1A
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1B
	.db NOTE_4, TONE_2C

	.db NOTE_8, TONE_2G
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2B
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_1A

	.db NOTE_8, TONE_2A
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_1D
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2C
	.db NOTE_8, TONE_1B
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_0G

	.db NOTE_8, TONE_1G
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_1E
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_2D
	.db NOTE_8, TONE_2C
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_0A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_8, TONE_1A
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1B
	.db NOTE_4, TONE_2C
#if !defined(ALL)
SNDDATA_END:
#endif
#endif

#if defined(DONSUKA) || defined(ALL)
#if !defined(ALL)
SNDDATA:
#endif
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_2G
	.db NOTE_16, TONE_2A
	.db NOTE_32, TONE_2G
	.db NOTE_32, TONE_2A
	.db NOTE_16, TONE_2AS
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_2G
	.db NOTE_16, TONE_2F
	.db NOTE_12, TONE_2DS
	.db NOTE_32, TONE_2F
	.db NOTE_32, TONE_2G
	.db NOTE_16, TONE_2GS
	.db NOTE_16, TONE_2G
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_2DS

	.db NOTE_4, TONE_1C
	.db NOTE_4, TONE_1E
	.db NOTE_4, TONE_1G
	.db NOTE_8, TONE_2C
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE

	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_1E
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_1E
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_2G
	.db NOTE_16, TONE_3C
	.db NOTE_16, TONE_2G
	.db NOTE_16, TONE_2F
	.db NOTE_8, TONE_2E
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_NONE

	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_1E
	.db NOTE_16, TONE_2E
	.db NOTE_16, TONE_1E
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_2F
	.db NOTE_16, TONE_1F
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2C
	.db NOTE_4, TONE_1AS

	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_0A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_0A
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_0AS
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_1C
	.db NOTE_8, TONE_2DS
	.db NOTE_8, TONE_2D
	.db NOTE_4, TONE_2C

	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_0A
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_0A
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_0AS
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_1C
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_NONE

	
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_0G
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE
	.db NOTE_2, TONE_NONE
#if !defined(ALL)
SNDDATA_END:
#endif
#endif

#if defined(SPERUNKER) || defined(ALL)
#if !defined(ALL)
SNDDATA:
#endif
	.db NOTE_16, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1E
	.db NOTE_8, TONE_1C

	.db NOTE_16, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1E
	.db NOTE_8, TONE_1C

	.db NOTE_16, TONE_NONE
	.db NOTE_8, TONE_2C
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2DS
	.db NOTE_16, TONE_2D
	.db NOTE_8, TONE_2C
	.db NOTE_32, TONE_2DS
	.db NOTE_32, TONE_2D
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_2DS
	.db NOTE_16, TONE_2D
	.db NOTE_4, TONE_2C

	.db NOTE_16, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1E
	.db NOTE_8, TONE_1C

	.db NOTE_4, TONE_1GS
	.db NOTE_8, TONE_1GS
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_2AS
	.db NOTE_16, TONE_2D
	.db NOTE_8, TONE_2C
	.db NOTE_8, TONE_1AS
	.db NOTE_12, TONE_1AS

	.db NOTE_4, TONE_2C
	.db NOTE_16, TONE_2DS
	.db NOTE_16, TONE_2D
	.db NOTE_16, TONE_2C
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_0G
	.db NOTE_16, TONE_NONE

	.db NOTE_16, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1E
	.db NOTE_8, TONE_1C

	.db NOTE_16, TONE_2C
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2C
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1AS
	.db NOTE_16, TONE_1A
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_1F
	.db NOTE_16, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1G
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1E
	.db NOTE_8, TONE_1C

	.db NOTE_32, TONE_1AS
	.db NOTE_16, TONE_NONE
	.db NOTE_32, TONE_1G
	.db NOTE_32, TONE_1A
	.db NOTE_16, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_1G
	.db NOTE_16, TONE_NONE
	.db NOTE_32, TONE_1DS
	.db NOTE_32, TONE_1F
	.db NOTE_16, TONE_NONE
	.db NOTE_32, TONE_1D
	.db NOTE_32, TONE_1C
	.db NOTE_24, TONE_NONE
	.db NOTE_16, TONE_1DS
	.db NOTE_16, TONE_NONE
	.db NOTE_16, TONE_1C
	.db NOTE_16, TONE_NONE
	.db NOTE_8, TONE_NONE
#if !defined(ALL)
SNDDATA_END:
#endif
#endif

#if defined(JAWS) || defined(ALL)
#if !defined(ALL)
SNDDATA:
#endif
	.db NOTE_8, TONE_1E		;ta-da
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE
	.db NOTE_8, TONE_NONE
	.db NOTE_8, TONE_NONE
	.db NOTE_8, TONE_NONE
	.db NOTE_8, TONE_NONE
	.db NOTE_8, TONE_NONE
	.db NOTE_8, TONE_NONE

	.db NOTE_8, TONE_1E		;ta-da
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE
	.db NOTE_2, TONE_NONE

	.db NOTE_8, TONE_1E		;ta-da,ta-da
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE
	.db NOTE_8, TONE_1E
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_16, TONE_NONE

	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE

	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE

	.db NOTE_32, TONE_2E
	.db NOTE_32, TONE_2G
	.db NOTE_WL, TONE_2AS

	.db NOTE_32, TONE_2E
	.db NOTE_32, TONE_2G
	.db NOTE_WL, TONE_3C

	.db NOTE_8, TONE_3E
	.db NOTE_8, TONE_2B
	.db NOTE_8, TONE_3FS
	.db NOTE_8, TONE_2B
	.db NOTE_16, TONE_3GS
	.db NOTE_16, TONE_3A
	.db NOTE_16, TONE_3B
	.db NOTE_16, TONE_3GS
	.db NOTE_8, TONE_3FS
	.db NOTE_8, TONE_2B

	.db NOTE_8, TONE_3E
	.db NOTE_8, TONE_2B
	.db NOTE_16, TONE_3FS
	.db NOTE_16, TONE_2A
	.db NOTE_16, TONE_3GS
	.db NOTE_16, TONE_2FS
	.db NOTE_8, TONE_3CS
	.db NOTE_4, TONE_2B
	.db NOTE_8, TONE_2B

	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_1F
	.db NOTE_32, TONE_NONE
	
	.db NOTE_32, TONE_2E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2F
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2E
	.db NOTE_32, TONE_NONE
	.db NOTE_32, TONE_2F
	.db NOTE_32, TONE_NONE

	.db NOTE_4, TONE_NONE

#if defined(JAWS) || defined(ALL)
SNDDATA_END:
#endif
#endif

;=============================================================
;=============================================================
;		   END
;=============================================================
