# Automated Pill Dispenser

## Description

The Automated Pill Dispenser is a low-cost electromechanical prototype designed to investigate controlled, one-at-a-time dispensing of solid tablets. An Arduino coordinates a 28BYJ-48 stepper motor, which operates a refill/release mechanism, and a DC motor, which advances a tablet toward an output. An infrared break-beam sensor monitors the dispensing path and signals the controller to stop the DC motor when an object interrupts the beam. The mechanism is intended to support modular, 3D-printable components and iterative testing.

## Purpose Statement

The purpose of this project is to design, build, and evaluate an automated dispensing mechanism that combines motor control and optical sensing to improve the consistency of single-tablet delivery. The prototype is intended to explore reliable actuation, pill detection, and error handling while identifying mechanical and electronic limitations that must be addressed before any practical medication-dispensing application.

> **Prototype only:** This project is for engineering development and testing, **not for dispensing medication to patients**. Pill counts, stepper position, and one-pill delivery are not independently verified by the current hardware or firmware. Use non-medication test objects during evaluation.

## Hardware

- Arduino Uno (ATmega328P); Nano-compatible firmware with appropriate PlatformIO board selection
- 28BYJ-48 5 V stepper motor and ULN2003 driver
- DC motor with **L293D** motor driver (this firmware does **not** target the DRV8833)
- Active-LOW infrared break-beam sensor
- Two pushbuttons: POWER and NEXT
- Suitable external motor power supply and common ground with Arduino
- Prototype tablet reservoir, release mechanism, and dispensing channel

### Arduino pin assignments

| Function | Arduino pin | Connection |
| --- | --- | --- |
| Break-beam receiver | D2 | Active-LOW sensor output; interrupt input |
| NEXT button | D3 | Button to GND; internal pull-up enabled |
| POWER button | D4 | Button to GND; internal pull-up enabled |
| DC driver enable | D5 | L293D EN |
| DC driver direction 1 | D6 | L293D IN1 |
| DC driver direction 2 | D7 | L293D IN2 |
| Stepper IN1 | D8 | ULN2003 IN1 |
| Stepper IN2 | D9 | ULN2003 IN2 |
| Stepper IN3 | D10 | ULN2003 IN3 |
| Stepper IN4 | D11 | ULN2003 IN4 |

Power the motors through their appropriate driver supply connections, not through Arduino GPIO. Ensure Arduino, sensor, and motor drivers share a ground. Verify driver supply voltage and current capacity for your particular motors.

## Current firmware behavior

1. Press **POWER** to initiate the first cycle. With an estimated empty inventory, the system waits **5 seconds** before operating the refill mechanism.
2. The stepper is commanded approximately **60° forward and back** using direct, nonblocking half-step control at **3 RPM** (software limit: 10 RPM).
3. The firmware assumes **four tablets per refill cycle**; this is an estimate, not a measured count.
4. With the break beam continuously clear for at least **50 ms**, the DC motor is enabled to advance a tablet.
5. When the sensor's beam is interrupted, a hardware interrupt removes forward drive and the controller applies DC motor braking for **300 ms**. The estimated remaining count decreases by one.
6. After the beam is clear for **50 continuous ms**, the sensor is ready again. **Press NEXT** to command another dispensing cycle; sensor re-arming alone does not restart the motor.
7. After the estimated inventory reaches zero, the next requested cycle runs the refill sequence again. Pressing **POWER** while active stops the motors; interruption during stepper movement requires manual re-homing and reset.

Serial diagnostics are printed at **115200 baud**. The state machine also includes a fault-lock condition for detected inconsistencies.

## Build and upload (PlatformIO / VS Code)

1. Install **Visual Studio Code** and the **PlatformIO IDE** extension.
2. Clone or download this repository and open its root folder in VS Code.
3. Connect the Arduino Uno over USB.
4. Build using **PlatformIO: Build** (or `pio run -e uno`).
5. Upload using **PlatformIO: Upload** (or `pio run -e uno -t upload`).
6. Open the PlatformIO Serial Monitor at **115200 baud**.

The project uses the Arduino framework and no additional third-party libraries. The entry point is `src/main.cpp`.

**Using an Arduino Nano?** Change the `board` value in `platformio.ini` from `uno` to `nanoatmega328` (classic/old bootloader) or `nanoatmega328new` (new bootloader), as appropriate for your board.

## Repository structure

```text
Automated_Pill_Dispenser/
├── platformio.ini       # PlatformIO board, framework, and monitor settings
├── README.md            # Project documentation
└── src/
    └── main.cpp        # Arduino firmware
```

## Known limitations and testing priorities

- **Single-tablet accuracy is not guaranteed.** The break beam indicates an interruption, not tablet identity or the number of tablets that passed. Mechanical inertia may allow extra tablets to move after detection.
- **Refill counts are assumptions.** The stepper's commanded position is not confirmed by an encoder or home switch; missed steps, jams, and empty batches may go unnoticed.
- **No motor operation timeout is implemented.** If a tablet never reaches the beam or the sensor fails, the DC motor can continue running. An independent fail-safe shutdown should be added before unattended operation.
- **Sensor and motor timing need physical validation.** In particular, test missed first detections, multiple-object passage, beam bounce, actual braking distance, stepper motion, and recovery after power interruption.
- **No medication safety features are implemented.** There is no prescription validation, dosage verification, patient identification, adherence tracking, or clinical validation.

### Suggested bench-test checklist

- Confirm each button produces the intended state transition.
- Confirm all four ULN2003 inputs step correctly and the stepper physically returns to its home position.
- Run empty-channel sensor tests before testing with inert dummy tablets.
- Verify motor stop/brake behavior for short and sustained beam interruptions.
- Test repeated cycles and count discrepancies; record missed and multiple dispenses.
- Verify that sensor disconnection, blocked channels, and unexpected power interruptions do not create unsafe motor behavior.
