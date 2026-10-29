; =====================================================================
;  DRONE.ASM  -  Drone flight-control unit using 8086 + 8255 PPI
;  21ECC313P Embedded Microcontrollers - PBL-3 Assignment
;
;  What it does (every pass of the main loop):
;    1. Reads throttle, tilt and battery through the ADC0808 (ch 0,1,2)
;    2. Computes left and right motor speeds to keep the drone level
;    3. Low battery -> buzzer ON and both sides drop to landing speed
;    4. Drives 4 motors with software PWM through 8255 Port B
;    5. Shows status (16x2 LCD on hardware, screen in simulation)
;
;  SIMMODE = 1 : test-case mode. Sensor values come from the TESTS
;                table and the "LCD" lines are printed on the screen.
;  SIMMODE = 0 : real hardware / Proteus. Reads the ADC0808 and
;                writes to the LCD, looping forever.
;
;  Assembles as a .COM program (NASM, EMU8086 or MASM-style).
; =====================================================================

        ORG 100H

; ----------------- 8255 port addresses (see section 3.2) -------------
PORT_A  EQU 00H         ; input  : ADC0808 data D0-D7
PORT_B  EQU 02H         ; output : motors, LCD RS/E, buzzer
PORT_C  EQU 04H         ; output : ADC control (PC0-3), LCD data (PC4-7)
CTRL    EQU 06H         ; 8255 control register
CWORD   EQU 90H         ; mode 0, PA in, PB out, PC out

; ----------------- Port B bit masks ----------------------------------
LEFT_M  EQU 05H         ; PB0 = M1 front-left,  PB2 = M3 rear-left
RIGHT_M EQU 0AH         ; PB1 = M2 front-right, PB3 = M4 rear-right
LCD_RS  EQU 10H         ; PB4
LCD_E   EQU 20H         ; PB5
BUZZER  EQU 40H         ; PB6

; ----------------- Flight constants ----------------------------------
LEVEL   EQU 128         ; tilt reading when the drone is level
BAT_LOW EQU 100         ; below this the battery is low (about 39%)
LAND_SP EQU 60          ; motor speed used while landing
NTESTS  EQU 4           ; number of simulation test cases

; =====================================================================
START:
        MOV  AL, CWORD          ; configure the 8255
        OUT  CTRL, AL
        MOV  AL, 0
        OUT  PORT_B, AL         ; all motors off, buzzer off
        OUT  PORT_C, AL         ; ADC lines idle

        MOV  AL, [SIMMODE]
        CMP  AL, 1
        JE   MAINLOOP
        CALL LCD_INIT           ; hardware only

MAINLOOP:
        MOV  AL, 0              ; channel 0 = throttle
        CALL READ_ADC
        MOV  [THR], AL
        MOV  AL, 1              ; channel 1 = tilt
        CALL READ_ADC
        MOV  [TILT], AL
        MOV  AL, 2              ; channel 2 = battery
        CALL READ_ADC
        MOV  [BAT], AL

        CALL CONTROL            ; decide motor speeds
        CALL PWM_FRAME          ; drive the motors
        CALL SHOW               ; display status

        MOV  AL, [SIMMODE]
        CMP  AL, 1
        JNE  MAINLOOP           ; hardware: repeat forever

        MOV  AL, [CASENO]       ; simulation: next test case
        INC  AL
        MOV  [CASENO], AL
        CMP  AL, NTESTS
        JB   MAINLOOP

        MOV  AH, 4CH            ; simulation finished - back to DOS
        INT  21H

; =====================================================================
; CONTROL : works out LSPD and RSPD from THR, TILT and BAT
;   correction = (LEVEL - TILT) / 4
;   left  = THR + correction      right = THR - correction
;   (tilt below 128 = left side low, so the left motors speed up)
; =====================================================================
CONTROL:
        MOV  AL, 0
        MOV  [ALARM], AL
        MOV  AL, [BAT]
        CMP  AL, BAT_LOW
        JAE  STABILISE
        MOV  AL, 1              ; low battery: alarm and land
        MOV  [ALARM], AL
        MOV  AL, LAND_SP
        MOV  [LSPD], AL
        MOV  [RSPD], AL
        RET

STABILISE:
        MOV  AX, LEVEL
        MOV  BL, [TILT]
        MOV  BH, 0
        SUB  AX, BX             ; AX = 128 - tilt  (signed)
        MOV  CL, 2
        SAR  AX, CL             ; divide by 4, keeping the sign
        MOV  DX, AX             ; DX = correction

        MOV  BL, [THR]
        MOV  BH, 0
        MOV  AX, BX
        ADD  AX, DX             ; left = throttle + correction
        CALL CLAMP
        MOV  [LSPD], AL

        MOV  AX, BX
        SUB  AX, DX             ; right = throttle - correction
        CALL CLAMP
        MOV  [RSPD], AL
        RET

; CLAMP : keeps the signed value in AX inside 0..255
CLAMP:
        CMP  AX, 0
        JGE  CL_HIGH
        MOV  AX, 0
        RET
CL_HIGH:
        CMP  AX, 255
        JLE  CL_DONE
        MOV  AX, 255
CL_DONE:
        RET

; =====================================================================
; PWM_FRAME : one software-PWM frame of 255 steps.
;   At step n a motor is ON if its speed > n,
;   so speed 255 = always on, speed 128 = on about half the time.
; =====================================================================
PWM_FRAME:
        MOV  CX, 255
        MOV  BL, 0              ; BL = step number
PWM_STEP:
        MOV  AL, 0
        CMP  BL, [LSPD]
        JAE  PWM_R
        OR   AL, LEFT_M
PWM_R:
        CMP  BL, [RSPD]
        JAE  PWM_BZ
        OR   AL, RIGHT_M
PWM_BZ:
        MOV  AH, [ALARM]
        CMP  AH, 1
        JNE  PWM_OUT
        OR   AL, BUZZER
PWM_OUT:
        OUT  PORT_B, AL         ; update all four motors at once
        INC  BL
        LOOP PWM_STEP
        RET

; =====================================================================
; READ_ADC : AL = channel (0-2)  ->  AL = 8-bit reading (0-255)
; =====================================================================
READ_ADC:
        PUSH BX
        PUSH CX
        MOV  AH, [SIMMODE]
        CMP  AH, 1
        JNE  ADC_HW

        ; ---- simulation: value = TESTS[CASENO*3 + channel] ----
        MOV  AH, 0
        MOV  BX, AX             ; BX = channel
        MOV  AL, [CASENO]
        MOV  CL, 3
        MUL  CL                 ; AX = case number * 3
        ADD  BX, AX
        MOV  AL, [TESTS+BX]
        JMP  ADC_DONE

        ; ---- hardware: ADC0808 handshake using 8255 BSR mode ----
ADC_HW:
        MOV  AH, AL             ; AH = channel
        MOV  AL, 00H            ; PC0 (ADD A) = bit 0 of channel
        TEST AH, 01H
        JZ   ADC_A
        MOV  AL, 01H
ADC_A:
        OUT  CTRL, AL
        MOV  AL, 02H            ; PC1 (ADD B) = bit 1 of channel
        TEST AH, 02H
        JZ   ADC_B
        MOV  AL, 03H
ADC_B:
        OUT  CTRL, AL
        MOV  AL, 05H            ; PC2 = 1 : ALE + START high
        OUT  CTRL, AL
        MOV  AL, 04H            ; PC2 = 0 : conversion starts
        OUT  CTRL, AL
        MOV  CX, 50             ; wait about 170 us (EOC not used)
ADC_WAIT:
        LOOP ADC_WAIT
        MOV  AL, 07H            ; PC3 = 1 : OE high, result on bus
        OUT  CTRL, AL
        IN   AL, PORT_A         ; read the converted value
        MOV  AH, AL
        MOV  AL, 06H            ; PC3 = 0 : OE low
        OUT  CTRL, AL
        MOV  AL, AH
ADC_DONE:
        POP  CX
        POP  BX
        RET

; =====================================================================
; SHOW : line 1 "THR=150 TILT=128"   line 2 "L=150 R=150 OK  "
;        or, on low battery,          line 2 "L=060 R=060 LAND"
; =====================================================================
SHOW:
        CALL LINE1
        MOV  SI, S_THR
        CALL PUTS
        MOV  AL, [THR]
        CALL PUTNUM
        MOV  SI, S_TILT
        CALL PUTS
        MOV  AL, [TILT]
        CALL PUTNUM

        CALL LINE2
        MOV  SI, S_LEFT
        CALL PUTS
        MOV  AL, [LSPD]
        CALL PUTNUM
        MOV  SI, S_RIGHT
        CALL PUTS
        MOV  AL, [RSPD]
        CALL PUTNUM
        MOV  SI, S_OK
        MOV  AL, [ALARM]
        CMP  AL, 1
        JNE  SHOW_END
        MOV  SI, S_LAND
SHOW_END:
        CALL PUTS
        CALL ENDSHOW
        RET

; LINE1 / LINE2 / ENDSHOW : move to a line (LCD) or new lines (screen)
LINE1:
        MOV  AL, [SIMMODE]
        CMP  AL, 1
        JE   L1_SIM
        MOV  AL, 80H            ; LCD: cursor to line 1
        CALL LCD_CMD
L1_SIM:
        RET

LINE2:
        MOV  AL, [SIMMODE]
        CMP  AL, 1
        JE   NEWLINE
        MOV  AL, 0C0H           ; LCD: cursor to line 2
        CALL LCD_CMD
        RET

ENDSHOW:
        MOV  AL, [SIMMODE]
        CMP  AL, 1
        JNE  ES_DONE
        CALL NEWLINE
        CALL NEWLINE
ES_DONE:
        RET

NEWLINE:
        MOV  AL, 0DH
        CALL PUTC
        MOV  AL, 0AH
        CALL PUTC
        RET

; PUTS : prints the 0-terminated string at DS:SI
PUTS:
        MOV  AL, [SI]
        CMP  AL, 0
        JE   PUTS_END
        CALL PUTC
        INC  SI
        JMP  PUTS
PUTS_END:
        RET

; PUTNUM : prints AL (0-255) as three decimal digits
PUTNUM:
        PUSH BX
        MOV  AH, 0
        MOV  BL, 100
        DIV  BL                 ; AL = hundreds, AH = remainder
        PUSH AX
        ADD  AL, '0'
        CALL PUTC
        POP  AX
        MOV  AL, AH
        MOV  AH, 0
        MOV  BL, 10
        DIV  BL                 ; AL = tens, AH = units
        PUSH AX
        ADD  AL, '0'
        CALL PUTC
        POP  AX
        MOV  AL, AH
        ADD  AL, '0'
        CALL PUTC
        POP  BX
        RET

; PUTC : one character in AL -> screen (simulation) or LCD (hardware)
PUTC:
        PUSH AX
        PUSH DX
        MOV  AH, [SIMMODE]
        CMP  AH, 1
        JNE  PUTC_LCD
        MOV  DL, AL
        MOV  AH, 02H            ; DOS: print character in DL
        INT  21H
        JMP  PUTC_END
PUTC_LCD:
        MOV  AH, LCD_RS         ; RS = 1 : data
        CALL LCD_BYTE
PUTC_END:
        POP  DX
        POP  AX
        RET

; =====================================================================
; LCD routines (hardware only) - 4-bit mode
;   data D4-D7 on PC4-PC7, RS on PB4, E on PB5, R/W tied to ground
; =====================================================================
LCD_INIT:
        MOV  CX, 3000           ; wait > 15 ms after power-up
        CALL DELAY
        MOV  AL, 30H            ; wake-up sequence: 3 x 30H
        MOV  AH, 0
        CALL LCD_NIB
        MOV  CX, 1500
        CALL DELAY
        MOV  AL, 30H
        MOV  AH, 0
        CALL LCD_NIB
        MOV  CX, 600
        CALL DELAY
        MOV  AL, 30H
        MOV  AH, 0
        CALL LCD_NIB
        MOV  CX, 600
        CALL DELAY
        MOV  AL, 20H            ; switch to 4-bit mode
        MOV  AH, 0
        CALL LCD_NIB
        MOV  CX, 600
        CALL DELAY
        MOV  AL, 28H            ; 4-bit, 2 lines, 5x7 font
        CALL LCD_CMD
        MOV  AL, 0CH            ; display on, cursor off
        CALL LCD_CMD
        MOV  AL, 06H            ; cursor moves right
        CALL LCD_CMD
        MOV  AL, 01H            ; clear display
        CALL LCD_CMD
        RET

; LCD_CMD : command byte in AL (RS = 0)
LCD_CMD:
        PUSH AX
        MOV  AH, 0
        CALL LCD_BYTE
        POP  AX
        RET

; LCD_BYTE : byte in AL, RS bit in AH. Sends high nibble then low nibble.
LCD_BYTE:
        PUSH AX
        PUSH CX
        PUSH AX
        CALL LCD_NIB            ; high nibble
        POP  AX
        MOV  CL, 4
        SHL  AL, CL
        CALL LCD_NIB            ; low nibble
        MOV  CX, 600            ; about 2 ms for the LCD to finish
        CALL DELAY
        POP  CX
        POP  AX
        RET

; LCD_NIB : upper 4 bits of AL -> LCD, AH = RS bit. Pulses E.
LCD_NIB:
        AND  AL, 0F0H           ; keep PC0-PC3 (ADC lines) at 0
        OUT  PORT_C, AL
        MOV  AL, AH
        OR   AL, LCD_E          ; E = 1
        OUT  PORT_B, AL
        NOP
        MOV  AL, AH             ; E = 0 : LCD latches the nibble
        OUT  PORT_B, AL
        RET

; DELAY : about 17 clock cycles x CX
DELAY:
        LOOP DELAY
        RET

; =====================================================================
; DATA
; =====================================================================
SIMMODE DB 1                    ; 1 = test cases, 0 = live sensors / hardware
CASENO  DB 0
THR     DB 0
TILT    DB 0
BAT     DB 0
LSPD    DB 0
RSPD    DB 0
ALARM   DB 0

; Simulation test cases : throttle, tilt, battery
TESTS   DB 150, 128, 200        ; 1. level hover
        DB 150,  88, 200        ; 2. tilted to the left
        DB 240, 200, 200        ; 3. hard tilt right at high throttle
        DB 180, 128,  80        ; 4. low battery

S_THR   DB 'THR=', 0
S_TILT  DB ' TILT=', 0
S_LEFT  DB 'L=', 0
S_RIGHT DB ' R=', 0
S_OK    DB ' OK  ', 0
S_LAND  DB ' LAND', 0
