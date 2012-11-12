;=============================================================
; title			: eMushibue
; author		: Nobuhiro Kuroiwa
; started on	: 11/11/2012
; clock			: clk=8MHz
;=============================================================
; melody data select
; TINKLE	: Twinkle twinkle litttle star
; XMAS		: We wish you are merry christmas
;#define		TWINKLE	; uncommend if you want to change melody
#define		XMAS		; commend if you above is commented out

; header files
.include "m168def.inc"	;
.include "musicnote.inc"	; definition of music stuff

;=============================================================
; constants
;=============================================================
; timer cont
.equ T10USEC	= 248	; Pre Scale=1/8, 100KHz
.equ PRE_SCALE	= 0x2	; 1/8
.equ VCNTMAX	= 20	; max valu of vcnt
.equ ADCSRAVAL	= (1<<ADEN)|(1<<ADSC)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
						; adc setting
						; max adc takes 400us, etc
.equ ADMUXVAL	= (1<<REFS0)|(1<<ADLAR)	; adc ref voltage=AVcc, 8bit precision
.equ ZEROGREAD	= 128	; 0g value of accerelometer read

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
.def sctopl		= r9	; sound interval count low byte
.def sctoph		= r10	; sound interval count high byte
.def mcnt		= r11	; t100ms counter
.def mtop		= r12	; tcnt top value
.def scntl		= r13	; scntl compare value
.def scnth		= r14	; scnth compare value
.def vcnttop	= r15	; volume count top
.def vcnt		= r16	; volume count
.def vread		= r17	; voltage displacement read
.def acc		= r18	; accumulator
.def acc2		= r19	; accumulator2

;=============================================================
; macro
;=============================================================
; preserve registers
.macro STORE_REGS
	in		sreg_save, SREG	; preserve status
	push	sreg_save
	push	acc				; preserve acc
	push	acc2			; preserve acc2
.endmacro

.macro RESTORE_REGS
	pop		acc2			; restore acc2			
	pop		acc				; restore acc
	pop		sreg_save
	out		SREG, sreg_save	; restore sreg
.endmacro

; time count
.macro TIME_COUNT		; TIME_COUNT @0 @1
	inc		@0			; increment register given by @0
	cp		@0, ten		; compare the register
	brne	@1			; if the register != 10 jump to @1
	clr		@0			; clear the register
.endmacro

; flip port output
.macro FLIP_PORTOUT		; FLIP_PORT portx, port_bit
	in		acc, @0
	ldi		acc2, @1	; bit mask
	eor		acc, acc2
	out		@0, acc		; output
.endmacro

;=============================================================
; program
;=============================================================
.cseg					   ; Code segment

;=============================================================
; vectors
;=============================================================
.org 	0x0000		jmp	main	 	; reset handler
.org	0x0020		jmp intr_time0	; timer0 overflow handler
.org	0x002A		jmp adc_comp	; adc complete

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

	; initialize port b
	ldi	 	acc, 0xFF		; PORTB all bits are output
	out	 	DDRB, acc		; set direction
	ldi	 	acc, 0x00		; set output data
	out	 	PORTB, acc		; set all to low

	; initialize port c
	ldi		acc, 0x00		; PORTC all bits are input
	out		DDRC, acc		; set direction
	out		portc, acc

	; initialize port d
	ldi	 	acc, 0xFF		; PORTD all bits are output
	out	 	DDRD, acc		; set direction
	ldi	 	acc, 0x00		; set output data
	out	 	PORTD, acc		; set all to low

	; Timer/Counter 0 initialize
	; tccr0a=0, standard mode
	lds		acc, timsk0
	sbr	 	acc,(1<<TOIE0)	; set overflow interruption bit
	sts	 	TIMSK0, acc		; allow timer0 overflow interruption
	ldi	 	acc, T10USEC	; 10us count
	out	 	TCNT0, acc		; set timer0 counter
	ldi	 	acc, PRE_SCALE	; set prescale
	out	 	TCCR0B, acc		; start timer0

	; initialize our counters
	clr	 	t10us			; init counter for 10us
	clr	 	t100us			; init counter for 100usr
	clr	 	t1ms			; init counter for 1ms
	clr		t10ms			; init counter for 10ms

	; initialize sound interval counter
	clr		sctopl			; sound interval count low
	clr		sctoph			; sound interval count high

	; initialize melody counter
	clr		mcnt			; initialize melody counter

	; load data
	ldi		zl, low(SNDDATA<<1)		; init zl
	ldi		zh, high(SNDDATA<<1)	; init zh

	lpm		mtop, z+			; initialize tcnt compare value
	lpm		sctopl, z+		; count untill scntl becomes this value
	clr		sctoph			; and scnth becomes this value. but not used this time

	; initialize volume counter
	ldi		acc, 0
	mov		vcnttop, acc
	clr		vcnt

	; initialize adc
	clr		vread
	ldi		acc, ADMUXVAL
	sts		admux, acc
	ldi		acc, ADCSRAVAL
	sts		adcsra, acc
	ldi		acc, 0x00
	sts		adcsrb, acc

	sei						; allow all interruption

main_loop:
	out		PORTB, vcnttop	; output current sctopl value
	rjmp	main_loop		; loop

;=============================================================
; timer0 interruption
;=============================================================
intr_time0:
	STORE_REGS

	; reset timer
	clr		acc				; stop counter
	out		tccr0b, acc

	ldi		acc, T10USEC	; 10usec count
	out		TCNT0, acc		; set timer0

	ldi		acc, PRE_SCALE
	out		tccr0b, acc

	TIME_COUNT t10us,	intr_time0_sndpwm	; count wrap around for 10us
	TIME_COUNT t100us,	intr_time0_sndpwm	; count wrap around for 100us
	rcall	vol_ctrl		; call every 100us
	TIME_COUNT t1ms,	intr_time0_sndpwm	; count wrap around for 1ms
	TIME_COUNT t10ms,	intr_time0_sndpwm	; count wrap around for 10ms
	TIME_COUNT t100ms,	initr_time0_setsnd	; count wrap around for 100ms
	TIME_COUNT t1s,		initr_time0_setsnd	; count wrap around for 1s

initr_time0_setsnd:
	
	; set sound frequency
	rcall	set_snd			; called every 100ms

	; request adc interruption
	lds		acc, admux
	ldi		acc2, 1<<MUX0
	eor		acc, acc2
	sts		admux, acc
	ldi		acc, ADCSRAVAL
	sts		adcsra, acc

intr_time0_sndpwm:
	rcall	snd_pwm

intr_time0_end:

	RESTORE_REGS
	reti

;=============================================================
; volume control
;=============================================================
vol_ctrl:
	inc		vcnt
	cp		vcnt, vcnttop
	brlt	vol_ctrl_high
	rjmp	vol_ctrl_low
vol_ctrl_high:
	sbi		portd, 1
	rjmp	vol_ctrl_vcnt
vol_ctrl_low:
	cbi		portd, 1
	rjmp	vol_ctrl_vcnt
vol_ctrl_vcnt:
	cpi		vcnt, VCNTMAX
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
	lpm		sctopl, z+		; count untill scntl becomes this value
	clr		sctoph			; and scnth becomes this value. but not used this time
	clr		scntl
	clr		scnth
set_snd_exit:
	ret

;=============================================================
; sound frequency pwm
;=============================================================
snd_pwm:
	clc
	adc		scntl, one
	brcc	snd_pwm1
	clc
	adc		scnth, one
	brcc	snd_pwm1
	clc
snd_pwm1:
	cp		sctopl, scntl
	brne	snd_pwm_ext
	cp		sctoph, scnth
	brne	snd_pwm_ext
	FLIP_PORTOUT portd, 0b0000_0001
	clr		scntl
	clr		scnth
snd_pwm_ext:
	ret

;=============================================================
; adc_complete
;=============================================================
adc_comp:
	STORE_REGS

	rcall	readv				; read voltage

	RESTORE_REGS

	reti

;=============================================================
; readv
;=============================================================
readv:
	lds		acc, adch
	subi	acc, ZEROGREAD
	brpl	readv_positive
	neg		acc
readv_positive:
	lds		acc2, admux
	sbrc	acc2, 0
	rjmp	readv_y
readv_x:
	mov		vread, acc
	ret
readv_y:
	add		vread, acc
readv_compare:
	cpi		vread, 16-8
	brlt	readv_level0
	cpi		vread, 18-8
	brlt	readv_level1
	cpi		vread, 20-8
	brlt	readv_level2
	cpi		vread, 22-8
	brlt	readv_level3
	rjmp	readv_level4
readv_level0:
	ldi		acc, 0
	rjmp readv_ext
readv_level1:
	ldi		acc, 4
	rjmp readv_ext
readv_level2:
	ldi		acc, 10
	rjmp readv_ext
readv_level3:
	ldi		acc, 16
	rjmp readv_ext
readv_level4:
	ldi		acc, 20
	rjmp readv_ext
readv_ext:
	mov		vcnttop, acc
	clr		vcnt
	ret

;=============================================================
; data
;=============================================================
#ifdef TWINKLE
SNDDATA:
	.db NOTE_8, TONE_2C
	.db NOTE_8, TONE_1C
	.db NOTE_8, TONE_2C
	.db NOTE_8, TONE_1C
	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2A
	.db NOTE_8, TONE_1A
	.db NOTE_8, TONE_2A
	.db NOTE_8, TONE_1A
	.db NOTE_2, TONE_2G

	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1D
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1D
	.db NOTE_2, TONE_2C

	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_2, TONE_2D

	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_2, TONE_2D

	.db NOTE_8, TONE_2C
	.db NOTE_8, TONE_1C
	.db NOTE_8, TONE_2C
	.db NOTE_8, TONE_1C
	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2G
	.db NOTE_8, TONE_1G
	.db NOTE_8, TONE_2A
	.db NOTE_8, TONE_1A
	.db NOTE_8, TONE_2A
	.db NOTE_8, TONE_1A
	.db NOTE_2, TONE_2G

	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2F
	.db NOTE_8, TONE_1F
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_8, TONE_2E
	.db NOTE_8, TONE_1E
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1D
	.db NOTE_8, TONE_2D
	.db NOTE_8, TONE_1D
	.db NOTE_2, TONE_2C
SNDDATA_END:
#endif

#ifdef XMAS
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
SNDDATA_END:
#endif

;=============================================================
;=============================================================
;		   END
;=============================================================
