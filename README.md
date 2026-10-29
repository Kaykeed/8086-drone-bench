# 8086 Drone Bench

A browser simulator for an 8086-based drone flight-control unit. It runs the real
machine code of `DRONE.ASM` on a simulated 8086 that is wired to an 8255 PPI,
an ADC0808, a 16×2 LCD and four motors, and shows every step of the work:
registers, flags, the physical address, I/O bus cycles (T1–T4), port pins and
the hand calculation for each loop.

Works on phones and laptops. No install, no build step: open `index.html`.

## What the program does

Every pass of the main loop the 8086:

1. Reads throttle, tilt and battery through the ADC0808 (channels 0, 1, 2).
2. Works out the motor speeds: correction = (128 − tilt) / 4,
   left = throttle + correction, right = throttle − correction, clamped to 0–255.
3. On low battery (reading below 100) turns the buzzer on and drops both sides to 60.
4. Drives the four motors with software PWM on 8255 Port B.
5. Shows the status on the LCD.

## Using the simulator

- **Live sensors**: move the throttle, tilt and battery sliders, or press one of the four test cases.
- **Test cases**: runs the four built-in cases from the `TESTS` table and prints the LCD lines.
- **Step** runs one instruction; **Run to** stops at a routine such as `STABILISE`.

| Case | Throttle | Tilt | Battery | Left / Right | LCD line 2 |
|---|---|---|---|---|---|
| Level hover | 150 | 128 | 200 | 150 / 150 | `L=150 R=150 OK` |
| Tilted left | 150 | 88 | 200 | 160 / 140 | `L=160 R=140 OK` |
| Hard tilt right | 240 | 200 | 200 | 222 / 255 | `L=222 R=255 OK` |
| Low battery | 180 | 128 | 80 | 60 / 60 | `L=060 R=060 LAND` |

## Hardware map

| 8255 port | Address | Use |
|---|---|---|
| Port A | 00H | ADC0808 data in |
| Port B | 02H | Motors (PB0–PB3), LCD RS/E (PB4, PB5), buzzer (PB6) |
| Port C | 04H | ADC control (PC0–PC3), LCD data D4–D7 (PC4–PC7) |
| Control | 06H | Control word 90H (mode 0, A in, B and C out) |

## Files

- `index.html`: the simulator (self-contained).
- `asm/DRONE.ASM`: the full 8086 program. Set `SIMMODE DB 0` for real hardware or Proteus.
- `asm/DRONE_CORE.ASM`: the key routines only.

The simulator is not cycle-accurate; the instructions, registers, flags and port values are.
