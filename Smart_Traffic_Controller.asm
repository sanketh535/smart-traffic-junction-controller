.include "m328Pdef.inc"

; ==================================================
; SMART TRAFFIC JUNCTION CONTROLLER
; ATmega328P / Arduino Uno / 16 MHz
;
; PD2 = NS Red       PD5 = EW Red
; PD3 = NS Yellow    PD6 = EW Yellow
; PD4 = NS Green     PD7 = EW Green
;
; PC0 = Pedestrian button, active LOW
; PC1 = LDR, ADC1
; PC2-PC5 = Seven-segment A-D
; PB0 = E, PB2 = G, PB3 = F
;
; ==================================================

.dseg
.org SRAM_START
TICK_LO:    .byte 1
TICK_HI:    .byte 1
BUTTON_RELEASED: .byte 1
BUTTON_LOCKOUT: .byte 1

.cseg
.org 0x0000
    jmp RESET

.org OC1Aaddr
    jmp TIMER1_COMPA_ISR

.org PCINT1addr
    jmp BUTTON_PCINT_ISR

.org 0x0034

; ==================================================
; RESET / INITIALIZATION
; ==================================================

RESET:
    ldi r16, high(RAMEND)
    out SPH, r16
    ldi r16, low(RAMEND)
    out SPL, r16

    ; Traffic LEDs: PD2-PD7
    ldi r16, 0b11111100
    out DDRD, r16
    clr r16
    out PORTD, r16

    ; PC0 button input, PC1 ADC input,
    ; PC2-PC5 display outputs
    ldi r16, 0b00111100
    out DDRC, r16

    ; Enable PC0 button pull-up
    ldi r16, 0b00000001
    out PORTC, r16

    ; Enable pin-change interrupt for PC0 (PCINT8)
    ldi r16, 1
    sts BUTTON_RELEASED, r16
    clr r16
    sts BUTTON_LOCKOUT, r16
    ldi r16, (1<<PCINT8)
    sts PCMSK1, r16
    ldi r16, (1<<PCIF1)
    out PCIFR, r16
    ldi r16, (1<<PCIE1)
    sts PCICR, r16

    ; PB0, PB2, PB3 display outputs
    ldi r16, 0b00001101
    out DDRB, r16
    out PORTB, r16

    ; ADC1, AVcc reference, left-adjust
    ldi r16, (1<<REFS0)|(1<<ADLAR)|(1<<MUX0)
    sts ADMUX, r16

    ; Enable ADC, prescaler 128
    ldi r16, (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
    sts ADCSRA, r16

    ; Initialize flags
    clr r23                 ; Pedestrian request
    clr r27                 ; Night mode flag
    clr r26                 ; Current phase identifier

    ; Initialize timer counter
    clr r16
    sts TICK_LO, r16
    sts TICK_HI, r16

    rcall INIT_TIMER1
    sei

; ==================================================
; MAIN PROGRAM
; ==================================================

MAIN:
    rcall CHECK_NIGHT

    tst r27
    breq NORMAL_TRAFFIC

    rjmp NIGHT_MODE

; ==================================================
; NORMAL TRAFFIC CYCLE
; r26:
; 1 = NS green
; 2 = first yellow
; 3 = EW green
; 4 = second yellow
; ==================================================

NORMAL_TRAFFIC:

    ; NS green, EW red
    ldi r16, 0b00110000
    out PORTD, r16

    ldi r26, 1
    ldi r24, 30
    rcall WAIT_100MS_BLOCKS
    rcall HANDLE_REQUEST

    ; Both yellow
    ldi r16, 0b01001000
    out PORTD, r16

    ldi r26, 2
    ldi r24, 15
    rcall WAIT_100MS_BLOCKS
    rcall HANDLE_REQUEST

    ; EW green, NS red
    ldi r16, 0b10000100
    out PORTD, r16

    ldi r26, 3
    ldi r24, 30
    rcall WAIT_100MS_BLOCKS
    rcall HANDLE_REQUEST

    ; Both yellow
    ldi r16, 0b01001000
    out PORTD, r16

    ldi r26, 4
    ldi r24, 15
    rcall WAIT_100MS_BLOCKS
    rcall HANDLE_REQUEST

    rjmp MAIN

; ==================================================
; TIMER1 INITIALIZATION
; 16 MHz / 64 / 250 = 1000 Hz
; Interrupt every 1 ms
; ==================================================

INIT_TIMER1:
    clr r16
    sts TCCR1A, r16
    sts TCCR1B, r16

    ldi r16, high(249)
    sts OCR1AH, r16
    ldi r16, low(249)
    sts OCR1AL, r16

    clr r16
    sts TCNT1H, r16
    sts TCNT1L, r16

    ldi r16, (1<<OCF1A)
    out TIFR1, r16

    ldi r16, (1<<OCIE1A)
    sts TIMSK1, r16

    ldi r16, (1<<WGM12)|(1<<CS11)|(1<<CS10)
    sts TCCR1B, r16

    ret

; ==================================================
; TIMER1 COMPARE MATCH A ISR
; ==================================================

TIMER1_COMPA_ISR:
    push r0
    in r0, SREG
    push r0
    push r16

    lds r16, TICK_LO
    inc r16
    sts TICK_LO, r16
    brne TIMER_ISR_DONE

    lds r16, TICK_HI
    inc r16
    sts TICK_HI, r16

TIMER_ISR_DONE:
    pop r16
    pop r0
    out SREG, r0
    pop r0
    reti

; ==================================================
; PCINT1 ISR - PEDESTRIAN BUTTON ON PC0 / PCINT8
; Falling edge: record request. Rising edge: record release.
; ==================================================

BUTTON_PCINT_ISR:
    push r0
    in r0, SREG
    push r0
    push r16
    push r17

    sbic PINC, 0
    rjmp BUTTON_RELEASE_EDGE

    ; Button is LOW: latch request unless crossing lockout is active
    lds r16, BUTTON_LOCKOUT
    tst r16
    brne BUTTON_MARK_PRESSED
    ldi r23, 1              ; Shared request flag read by main program

BUTTON_MARK_PRESSED:
    clr r16
    sts BUTTON_RELEASED, r16
    rjmp BUTTON_ISR_DONE

BUTTON_RELEASE_EDGE:
    ; Button is HIGH: record release event
    ldi r16, 1
    sts BUTTON_RELEASED, r16

BUTTON_ISR_DONE:
    pop r17
    pop r16
    pop r0
    out SREG, r0
    pop r0
    reti

; ==================================================
; TIMER-BASED WAIT
; Input: r25:r24 = milliseconds
; ==================================================

WAIT_MS_TIMER:
    in r16, SREG
    cli
    lds r18, TICK_LO
    lds r19, TICK_HI
    out SREG, r16

WAIT_MS_LOOP:
    in r16, SREG
    cli
    lds r20, TICK_LO
    lds r21, TICK_HI
    out SREG, r16

    ; elapsed = current - start
    sub r20, r18
    sbc r21, r19

    ; Compare elapsed with requested duration
    sub r20, r24
    sbc r21, r25
    brcc WAIT_MS_DONE

    rjmp WAIT_MS_LOOP

WAIT_MS_DONE:
    ret

; ==================================================
; TIMER-BASED 100 ms DELAY
; ==================================================

DELAY_100MS:
    ldi r24, low(100)
    ldi r25, high(100)
    rcall WAIT_MS_TIMER
    ret

; ==================================================
; WAIT BLOCKS
; Input: r24 = number of 100 ms intervals
; Record pedestrian button requests.
; ==================================================

WAIT_100MS_BLOCKS:
    mov r28, r24

W100_LOOP:
    rcall DELAY_100MS

    ; Button requests are captured by BUTTON_PCINT_ISR
    dec r28
    brne W100_LOOP

    ret

; ==================================================
; READ LDR USING ADC1
; Result: r17 = ADC high byte
; ==================================================

READ_LDR:
    lds r16, ADCSRA
    ori r16, (1<<ADSC)
    sts ADCSRA, r16

ADC_WAIT:
    lds r16, ADCSRA
    sbrc r16, ADSC
    rjmp ADC_WAIT

    ; Read ADCL before ADCH
    lds r16, ADCL
    lds r17, ADCH

    ret

; ==================================================
; NIGHT DETECTION
; ADC high byte < 100 means dark.
; Confirm darkness for 4 seconds before entering Night.
; ==================================================

CHECK_NIGHT:
    rcall READ_LDR
    cpi r17, 100
    brsh CN_DAY

    ; Confirm darkness for approximately 4 seconds
    ldi r24, low(4000)
    ldi r25, high(4000)
    rcall WAIT_MS_TIMER

    rcall READ_LDR
    cpi r17, 100
    brsh CN_DAY

    ; Enter Night Mode
    clr r23
    ldi r27, 1
    ret

CN_DAY:
    clr r27
    ret

; ==================================================
; NIGHT MODE
; Both yellow outputs flash.
; Pedestrian requests are ignored in Night Mode.
; ==================================================

NIGHT_MODE:
    clr r23

NIGHT_LOOP:
    rcall READ_LDR
    cpi r17, 100
    brsh NIGHT_BRIGHT_CONFIRM

    ; Yellow ON
    ldi r16, 0b01001000
    out PORTD, r16

    ldi r24, low(500)
    ldi r25, high(500)
    rcall WAIT_MS_TIMER

    rcall READ_LDR
    cpi r17, 100
    brsh NIGHT_BRIGHT_CONFIRM

    ; Yellow OFF
    clr r16
    out PORTD, r16

    ldi r24, low(500)
    ldi r25, high(500)
    rcall WAIT_MS_TIMER

    rjmp NIGHT_LOOP

; ==================================================
; NIGHT-MODE HYSTERESIS
; Brightness must remain above threshold for 5 seconds.
; Yellow flashing continues during confirmation.
; If darkness returns, restart the confirmation.
; ==================================================

NIGHT_BRIGHT_CONFIRM:
    ; Ten half-second intervals = 5 seconds
    ldi r28, 10
    clr r22

NIGHT_BRIGHT_LOOP:
    ; Toggle yellow flashing state
    tst r22
    brne NIGHT_FLASH_OFF

NIGHT_FLASH_ON:
    ldi r16, 0b01001000
    out PORTD, r16
    ldi r22, 1
    rjmp NIGHT_FLASH_WAIT

NIGHT_FLASH_OFF:
    clr r16
    out PORTD, r16
    clr r22

NIGHT_FLASH_WAIT:
    ldi r24, low(500)
    ldi r25, high(500)
    rcall WAIT_MS_TIMER

    rcall READ_LDR
    cpi r17, 100
    brlo NIGHT_DARK_AGAIN

    dec r28
    brne NIGHT_BRIGHT_LOOP

    ; Bright for 5 continuous seconds
    rjmp NIGHT_EXIT

NIGHT_DARK_AGAIN:
    ; Darkness returned: resume normal night flashing
    rjmp NIGHT_LOOP

NIGHT_EXIT:
    ; Return safely to all-red
    ldi r16, 0b00100100
    out PORTD, r16

    clr r27
    clr r23

    rjmp MAIN

; ==================================================
; HANDLE PEDESTRIAN REQUEST
; ==================================================

HANDLE_REQUEST:
    tst r23
    breq HANDLE_DONE

    ; Consume the request
    clr r23

    ; Do not handle requests during Night Mode
    tst r27
    brne HANDLE_DONE

    rcall PED_TRANSITION
    rcall PED_CROSS

HANDLE_DONE:
    ret

; ==================================================
; PEDESTRIAN TRANSITION
; Yellow transition when leaving a green phase.
; All-red clearance before pedestrian crossing.
; ==================================================

PED_TRANSITION:
    cpi r26, 1
    breq PED_FROM_NS_GREEN

    cpi r26, 2
    breq PED_FROM_FIRST_YELLOW

    cpi r26, 3
    breq PED_FROM_EW_GREEN

    cpi r26, 4
    breq PED_FROM_SECOND_YELLOW

    ; Unknown phase: fail safe to all-red
    rjmp PED_SET_ALL_RED

PED_FROM_NS_GREEN:
    ; NS yellow + EW red
    ldi r16, 0b00101000
    out PORTD, r16

    ; 1.5 seconds
    ldi r24, 15
    rcall WAIT_100MS_BLOCKS

    rjmp PED_SET_ALL_RED

PED_FROM_FIRST_YELLOW:
    ; Yellow phase already completed
    rjmp PED_SET_ALL_RED

PED_FROM_EW_GREEN:
    ; EW yellow + NS red
    ldi r16, 0b01000100
    out PORTD, r16

    ; 1.5 seconds
    ldi r24, 15
    rcall WAIT_100MS_BLOCKS

    rjmp PED_SET_ALL_RED

PED_FROM_SECOND_YELLOW:
    ; Yellow phase already completed
    rjmp PED_SET_ALL_RED

PED_SET_ALL_RED:
    ; All-red clearance
    ldi r16, 0b00100100
    out PORTD, r16

    ldi r24, low(1000)
    ldi r25, high(1000)
    rcall WAIT_MS_TIMER

    ret

; ==================================================
; PEDESTRIAN CROSSING
; Vehicle lights remain red.
; Countdown displays 5, 4, 3, 2, 1, 0.
; ==================================================

PED_CROSS:
    ; All vehicle lights red
    ldi r16, 0b00100100
    out PORTD, r16

    ; Ignore new pedestrian requests while this crossing is in progress
    ldi r16, 1
    sts BUTTON_LOCKOUT, r16

    ; Initialize release flag from the current button level atomically
    in r17, SREG
    cli
    clr r16
    sbis PINC, 0
    rjmp PED_BUTTON_STILL_LOW
    ldi r16, 1
PED_BUTTON_STILL_LOW:
    sts BUTTON_RELEASED, r16
    out SREG, r17

PED_WAIT_RELEASE:
    ; Wait for the pin-change ISR to record button release
    lds r16, BUTTON_RELEASED
    tst r16
    breq PED_WAIT_RELEASE

    ; Release debounce: 200 ms
    ldi r24, low(200)
    ldi r25, high(200)
    rcall WAIT_MS_TIMER

    clr r23

    ; Countdown 5 to 0
    ldi r29, 5

PED_COUNTDOWN:
    rcall SHOW_DIGIT

    ; One second per displayed number
    ldi r28, 10

PED_SECOND:
    rcall DELAY_100MS
    dec r28
    brne PED_SECOND

    dec r29
    brpl PED_COUNTDOWN

    ; Blank display
    ldi r19, 0b01111111
    rcall WRITE_SEGMENTS

    ; Keep vehicle lights red until normal cycling resumes
    ldi r16, 0b00100100
    out PORTD, r16

    ; Allow new pedestrian requests after crossing is complete
    clr r16
    sts BUTTON_LOCKOUT, r16

    ret

; ==================================================
; SEVEN-SEGMENT DIGIT SELECTION
; Common-anode display: LOW = ON
; Input: r29
; ==================================================

SHOW_DIGIT:
    cpi r29, 0
    breq DIGIT_0

    cpi r29, 1
    breq DIGIT_1

    cpi r29, 2
    breq DIGIT_2

    cpi r29, 3
    breq DIGIT_3

    cpi r29, 4
    breq DIGIT_4

    cpi r29, 5
    breq DIGIT_5

    ret

DIGIT_0:
    ldi r19, 0b11000000
    rjmp WRITE_SEGMENTS

DIGIT_1:
    ldi r19, 0b11111001
    rjmp WRITE_SEGMENTS

DIGIT_2:
    ldi r19, 0b10100100
    rjmp WRITE_SEGMENTS

DIGIT_3:
    ldi r19, 0b10110000
    rjmp WRITE_SEGMENTS

DIGIT_4:
    ldi r19, 0b10011001
    rjmp WRITE_SEGMENTS

DIGIT_5:
    ldi r19, 0b10010010
    rjmp WRITE_SEGMENTS

; ==================================================
; WRITE SEVEN-SEGMENT OUTPUTS
; A-D = PC2-PC5
; E   = PB0 / Arduino D8
; G   = PB2 / Arduino D10
; F   = PB3 / Arduino D11
; ==================================================

WRITE_SEGMENTS:
    ; Write A-D to PC2-PC5
    mov r16, r19
    andi r16, 0x0F
    lsl r16
    lsl r16
    ori r16, 0b00000001
    out PORTC, r16

    ; Write E, G, F to PB0, PB2, PB3
    clr r16

    sbrc r19, 4
    ori r16, 0b00000001

    sbrc r19, 6
    ori r16, 0b00000100

    sbrc r19, 5
    ori r16, 0b00001000

    out PORTB, r16

    ret