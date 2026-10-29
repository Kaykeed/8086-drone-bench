; DRONE_CORE.ASM - key routines of DRONE.ASM
; Full program (data and LCD routines): DRONE.ASM

PORT_A EQU 00H      ; 8255 Port A : ADC data in
PORT_B EQU 02H      ; Port B : motors, buzzer
CTRL   EQU 06H      ; 8255 control register
LEFT_M EQU 05H      ; PB0 + PB2 = left motors
RIGHT_M EQU 0AH     ; PB1 + PB3 = right motors
BUZZER EQU 40H      ; PB6

START:  MOV  AL, 90H        ; 8255 mode 0, A in
        OUT  CTRL, AL
MAINLOOP:
        MOV  AL, 0          ; ADC ch 0 = throttle
        CALL READ_ADC
        MOV  [THR], AL
        MOV  AL, 1
        CALL READ_ADC
        MOV  [TILT], AL
        MOV  AL, 2
        CALL READ_ADC
        MOV  [BAT], AL
        CALL CONTROL        ; work out motor speeds
        CALL PWM_FRAME      ; drive the motors
        CALL SHOW           ; LCD (in DRONE.ASM)
        JMP  MAINLOOP

CONTROL:                    ; land or stabilise
        MOV  AL, 0
        MOV  [ALARM], AL
        MOV  AL, [BAT]
        CMP  AL, 100        ; BAT_LOW
        JAE  STABILISE
        MOV  AL, 1
        MOV  [ALARM], AL
        MOV  AL, 60         ; LAND_SP
        MOV  [LSPD], AL
        MOV  [RSPD], AL
        RET
STABILISE:
        MOV  AX, 128        ; corr = (128-tilt)/4
        MOV  BL, [TILT]
        MOV  BH, 0
        SUB  AX, BX
        MOV  CL, 2
        SAR  AX, CL
        MOV  DX, AX
        MOV  BL, [THR]
        MOV  BH, 0
        MOV  AX, BX
        ADD  AX, DX         ; left = thr + corr
        CALL CLAMP
        MOV  [LSPD], AL
        MOV  AX, BX
        SUB  AX, DX         ; right = thr - corr
        CALL CLAMP
        MOV  [RSPD], AL
        RET
CLAMP:  CMP  AX, 0          ; keep AX within 0..255
        JGE  CL_HIGH
        MOV  AX, 0
        RET
CL_HIGH: CMP AX, 255
        JLE  CL_DONE
        MOV  AX, 255
CL_DONE: RET

PWM_FRAME:                  ; ON while step < speed
        MOV  CX, 255
        MOV  BL, 0
PWM_STEP:
        MOV  AL, 0
        CMP  BL, [LSPD]
        JAE  PWM_R
        OR   AL, LEFT_M
PWM_R:  CMP  BL, [RSPD]
        JAE  PWM_BZ
        OR   AL, RIGHT_M
PWM_BZ: MOV  AH, [ALARM]
        CMP  AH, 1
        JNE  PWM_OUT
        OR   AL, BUZZER
PWM_OUT: OUT PORT_B, AL
        INC  BL
        LOOP PWM_STEP
        RET

READ_ADC:                   ; AL = channel -> reading
        MOV  AH, AL
        MOV  AL, 00H        ; PC0 = ADD A
        TEST AH, 01H
        JZ   ADC_A
        MOV  AL, 01H
ADC_A:  OUT  CTRL, AL
        MOV  AL, 02H        ; PC1 = ADD B
        TEST AH, 02H
        JZ   ADC_B
        MOV  AL, 03H
ADC_B:  OUT  CTRL, AL
        MOV  AL, 05H        ; PC2 high: ALE + START
        OUT  CTRL, AL
        MOV  AL, 04H        ; PC2 low: conversion starts
        OUT  CTRL, AL
        MOV  CX, 50         ; wait ~170 us
ADC_W:  LOOP ADC_W
        MOV  AL, 07H        ; PC3 high: OE
        OUT  CTRL, AL
        IN   AL, PORT_A     ; read result
        MOV  AH, AL
        MOV  AL, 06H        ; PC3 low
        OUT  CTRL, AL
        MOV  AL, AH
        RET
