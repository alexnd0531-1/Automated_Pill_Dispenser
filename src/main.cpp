#include <Arduino.h>

// =============================================================
// PILL DISPENSER - SENSOR-PRIORITY, 50 ms STABLE-CLEAR REARM

// =============================================================
// Hardware assumptions:
//   Arduino Uno / Nano (ATmega328P)
//   L293D DC motor driver (NOT the DRV8833)
//   28BYJ-48 5V stepper + ULN2003 driver board
//   Active-LOW break-beam receiver on D2
//
// The stepper uses direct 8-phase half-stepping. No AccelStepper
// library, acceleration ramp, blocking delays, or motor timeouts.
//
// CRITICAL: Stepper position and 4-pill batch size are ESTIMATES.
// No mechanism here confirms that the stepper actually rotated,
// or that exactly one pill physically passed through the sensor.
//
// One detected pill stops the DC motor. After the sensor has been clear
// for 50 continuous ms, it is ready for the NEXT commanded dispense.
// Re-arming NEVER restarts the motor or decrements the count again.

// =============================================================
// -------------------------- PINOUT ---------------------------
const uint8_t BEAM_PIN  = 2;
const uint8_t NEXT_PIN  = 3;
const uint8_t POWER_PIN = 4;
// L293D: EN, IN1, IN2
const uint8_t DC_EN  = 5;
const uint8_t DC_IN1 = 6;
const uint8_t DC_IN2 = 7;
// ULN2003: Arduino D8->IN1, D9->IN2, D10->IN3, D11->IN4
const uint8_t STEPPER_PINS[4] = {8, 9, 10, 11};

// -------------------------- SETTINGS -------------------------
const uint8_t PILLS_PER_LOAD = 4;
const uint16_t HALF_STEPS_PER_REV = 4096;  // Approximate output-shaft steps
const uint16_t RELEASE_HALF_STEPS = 683;  // Approximately 60 degrees
const uint8_t STEPPER_RPM = 3;            // Change up to 10 RPM MAX
static_assert(STEPPER_RPM > 0 && STEPPER_RPM <= 10,
              "Stepper speed must be between 1 and 10 RPM");
// 60 seconds/minute * 1,000,000 microseconds/second
const uint32_t STEP_INTERVAL_US =
    60000000UL / (uint32_t(HALF_STEPS_PER_REV) * STEPPER_RPM);
const uint32_t REFILL_DELAY_MS = 5000;
const uint32_t BRAKE_MS = 300;
const uint32_t BEAM_CLEAR_MS = 50;  // continuous HIGH needed before next dispense
const uint32_t BUTTON_DEBOUNCE_MS = 30;
const uint32_t STEPPER_REPORT_MS = 1000;

// Standard 28BYJ-48 half-step sequence, bit 3 = IN1 ... bit 0 = IN4.
// LEDs on ULN2003 should follow these eight patterns repeatedly.
const uint8_t PHASE_MASKS[8] = {
    0b1000, 0b1100, 0b0100, 0b0110,
    0b0010, 0b0011, 0b0001, 0b1001
};
// ------------------------ SYSTEM STATE -----------------------
enum State : uint8_t {
    OFF,
    IDLE,
    REFILL_WAIT,
    STEPPER_OUT,
    STEPPER_BACK,
    WAIT_FOR_CLEAR_BEAM,
    DISPENSING,
    BRAKING,
    FAULT
};

State state = OFF;
bool powerEnabled = false;
bool firstPowerOn = true;
bool faultLocked = false;
int pillsRemaining = 0;
uint32_t stateBeganMs = 0;

// D2 uses its dedicated hardware interrupt. All ISR-shared flags are one byte.
// beamArmed: stop a running motor on the next falling edge.
// beamReady: sensor has been HIGH continuously for at least 50 ms.
// beamFellFlag: even a short LOW pulse resets the stable-clear timer.
volatile bool beamArmed = false;
volatile bool beamReady = false;
volatile bool beamFellFlag = false;
volatile bool pillDetectedFlag = false;

bool beamClearTiming = false;
uint32_t beamClearSinceMs = 0;
bool announceBeamRearm = false;

// Stepper bookkeeping; this is SOFTWARE position only.
int16_t stepperPosition = 0;
uint8_t phaseIndex = 0;
bool stepperEnergized = false;
uint32_t lastStepUs = 0;
uint32_t lastStepperReportMs = 0;

struct Button {
    uint8_t previousRaw = HIGH;
    uint8_t stableState = HIGH;
    uint32_t changedAtMs = 0;
};

Button nextButton;
Button powerButton;

// =============================================================
// EASY-TO-READ SERIAL MONITOR MESSAGES

// =============================================================
void printStage(const __FlashStringHelper *title) {
    Serial.println();
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(title);
    Serial.println(F("========================================"));
    Serial.println();
}

// =============================================================
// DC MOTOR - L293D ONLY

// =============================================================
void dcOff() {
    digitalWrite(DC_EN, LOW);
    digitalWrite(DC_IN1, LOW);
    digitalWrite(DC_IN2, LOW);
}

void dcBrake() {
    // EN high + IN1 low + IN2 low = active electrical braking.
    digitalWrite(DC_IN1, LOW);
    digitalWrite(DC_IN2, LOW);
    digitalWrite(DC_EN, HIGH);
}

void dcForward() {
    digitalWrite(DC_IN2, LOW);
    digitalWrite(DC_IN1, HIGH);
    digitalWrite(DC_EN, HIGH);
}

// =============================================================
// BREAK BEAM - HIGHEST PRIORITY

// =============================================================
void beamISR() {
    // Record EVERY falling edge, even while the motor is stopped.
    // This prevents a second pill or sensor bounce from being mistaken
    // for a stable HIGH signal before the next dispense.
    beamFellFlag = true;
    beamReady = false;

    if (beamArmed) {
        // Fast path: stop torque immediately (L293D IN2 is already LOW).
        digitalWrite(DC_IN1, LOW);
        beamArmed = false;
        pillDetectedFlag = true;
    }
    // Any subsequent falling edges are ignored for pill counting.
    // In particular, they must NOT cause a FAULT during BRAKING / IDLE.
}

void disarmBeam() {
    noInterrupts();
    beamArmed = false;
    interrupts();
}

bool readPillEvent() {
    noInterrupts();
    const bool detected = pillDetectedFlag;
    pillDetectedFlag = false;
    interrupts();
    return detected;
}

// Always call this each loop, including while BRAKING, IDLE, or OFF.
// After EVERY beam interruption, HIGH must stay continuous for 50 ms.
void serviceBeamReadiness() {
    noInterrupts();
    const bool sawFallingEdge = beamFellFlag;
    beamFellFlag = false;
    const bool blocked = (digitalRead(BEAM_PIN) == LOW);
    interrupts();

    if (sawFallingEdge || blocked) {
        beamReady = false;
        beamClearTiming = false;
        return;
    }

    if (!beamClearTiming) {
        beamClearSinceMs = millis();
        beamClearTiming = true;
        return;
    }

    if (!beamReady &&
        (uint32_t)(millis() - beamClearSinceMs) >= BEAM_CLEAR_MS) {
        // The ISR can also run during the check. Validate atomically
        // so an edge arriving at this instant cannot falsely arm us.
        bool becameReady = false;
        noInterrupts();
        if (!beamFellFlag && digitalRead(BEAM_PIN) == HIGH) {
            beamReady = true;
            becameReady = true;
        }
        interrupts();

        if (becameReady && announceBeamRearm && powerEnabled) {
            announceBeamRearm = false;
            printStage(F("BEAM READY - CLEAR FOR 50 ms"));
            Serial.println(F("Sensor reset and ready for the NEXT cycle."));
            Serial.println(F("DC motor remains OFF until NEXT is pressed."));
            Serial.println();
        }
    }
}

// Start only when the beam is clear AND has been stable for 50 ms.
// Arm and start atomically so a pill can't pass between those actions.
bool tryArmAndStartDC() {
    if (!beamReady) return false;

    noInterrupts();
    if (!beamReady || digitalRead(BEAM_PIN) == LOW ||
        beamFellFlag || pillDetectedFlag) {
        interrupts();
        return false;
    }

    beamArmed = true;
    state = DISPENSING;
    stateBeganMs = millis();
    dcForward();
    interrupts();

    printStage(F("STEP 4 - SENSOR ARMED / DC MOTOR ON"));
    Serial.println(F("Next beam break immediately brakes the motor."));
    Serial.println(F("Only one pill will be counted this cycle."));
    Serial.println();
    return true;
}

void prepareToDispense() {
    dcOff();
    disarmBeam();
    state = WAIT_FOR_CLEAR_BEAM;

    // If already ready, start without any Serial delay in between.
    if (!tryArmAndStartDC()) {
        printStage(F("WAITING - BEAM MUST BE CLEAR FOR 50 ms"));
        Serial.println(F("DC motor OFF. Checking sensor stability."));
        Serial.println(F("Motor starts after 50 ms clear (NEXT was pressed)."));
        Serial.println();
    }
}

// =============================================================
// STEPPER - DIRECT NONBLOCKING HALF-STEP DRIVE

// =============================================================
void applyStepperPhase(uint8_t index) {
    const uint8_t mask = PHASE_MASKS[index & 7];
    for (uint8_t i = 0; i < 4; ++i) {
        digitalWrite(STEPPER_PINS[i], (mask & (0b1000 >> i)) ? HIGH : LOW);
    }
}

void stepperOff() {
    for (uint8_t i = 0; i < 4; ++i) {
        digitalWrite(STEPPER_PINS[i], LOW);
    }
    stepperEnergized = false;
}

void beginStepperOut() {
    // First energize IN1 immediately. You should see a driver LED.
    phaseIndex = 0;
    stepperPosition = 0;
    applyStepperPhase(phaseIndex);
    stepperEnergized = true;
    lastStepUs = micros();
    lastStepperReportMs = millis();
    state = STEPPER_OUT;
    printStage(F("STEP 2 - STEPPER MOVING OUT"));
    Serial.print(F("Speed: "));
    Serial.print(STEPPER_RPM);
    Serial.println(F(" RPM (no acceleration)"));
    Serial.println(F("Watch the four LEDs on the ULN2003 driver."));
    Serial.println(F("They should cycle while the motor turns."));
    Serial.println();
}

void beginStepperBack() {
    state = STEPPER_BACK;
    lastStepperReportMs = millis();
    printStage(F("STEP 3 - STEPPER RETURNING HOME"));
    Serial.println(F("Reversing the eight-phase sequence."));
    Serial.println();
}
// Executes at most one half-step per loop pass.
// The sensor interrupt can always preempt this code.
void serviceStepper() {
    if (!stepperEnergized) return;
    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastStepUs) < STEP_INTERVAL_US) return;
    lastStepUs = nowUs;
    if (state == STEPPER_OUT) {
        phaseIndex = (phaseIndex + 1) & 7;
        applyStepperPhase(phaseIndex);
        ++stepperPosition;
    } else if (state == STEPPER_BACK) {
        phaseIndex = (phaseIndex + 7) & 7;
        applyStepperPhase(phaseIndex);
        --stepperPosition;
    } else {
        return;
    }
    if ((uint32_t)(millis() - lastStepperReportMs) >= STEPPER_REPORT_MS) {
        lastStepperReportMs = millis();
        Serial.print(F("Stepper commanded half-steps: "));
        Serial.print(stepperPosition);
        Serial.print(F(" / "));
        Serial.println(RELEASE_HALF_STEPS);
    }
    if (state == STEPPER_OUT && stepperPosition >= RELEASE_HALF_STEPS) {
        beginStepperBack();
    } else if (state == STEPPER_BACK && stepperPosition <= 0) {
        stepperOff();
        pillsRemaining = PILLS_PER_LOAD;  // Assumed, NOT sensor-confirmed!
        printStage(F("REFILL COMMAND FINISHED"));
        Serial.println(F("Stepper software returned to zero."));
        Serial.println(F("This does NOT prove that the motor physically moved."));
        Serial.print(F("Assumed pills in batch: "));
        Serial.println(pillsRemaining);
        Serial.println();
        prepareToDispense();
    }
}

// =============================================================
// BUTTONS AND SYSTEM CONTROLS

// =============================================================
bool buttonPressed(uint8_t pin, Button &button) {
    const uint8_t raw = digitalRead(pin);
    if (raw != button.previousRaw) {
        button.previousRaw = raw;
        button.changedAtMs = millis();
    }
    if (raw != button.stableState &&
        (uint32_t)(millis() - button.changedAtMs) >= BUTTON_DEBOUNCE_MS) {
        button.stableState = raw;
        return raw == LOW;
    }
    return false;
}

void enterFault(const __FlashStringHelper *reason) {
    disarmBeam();
    dcBrake();
    stepperOff();
    faultLocked = true;
    announceBeamRearm = false;
    state = FAULT;
    stateBeganMs = millis();
    printStage(F("FAULT - MACHINE STOPPED"));
    Serial.println(reason);
    Serial.println();
    Serial.println(F("Inspect the hardware and reset Arduino."));
    Serial.println(F("If the stepper was interrupted, manually re-home it."));
    Serial.println();
}

void turnPowerOff() {
    const bool interruptedStepper =
        (state == STEPPER_OUT || state == STEPPER_BACK);
    disarmBeam();
    dcBrake();
    stepperOff();
    announceBeamRearm = false;
    powerEnabled = false;
    if (interruptedStepper) faultLocked = true;
    state = OFF;
    stateBeganMs = millis();
    printStage(F("POWER OFF"));
    if (interruptedStepper) {
        Serial.println(F("Stepper stopped mid-travel."));
        Serial.println(F("Re-home mechanism and RESET before use."));
    } else {
        Serial.println(F("Motors stopped."));
        Serial.println(F("Press POWER again, then NEXT to resume."));
    }
    Serial.println();
}

void beginPillCycle() {
    disarmBeam();
    dcOff();
    if (pillsRemaining == 0) {
        state = REFILL_WAIT;
        stateBeganMs = millis();
        printStage(F("STEP 1 - REFILL NEEDED"));
        Serial.println(F("Estimated pill count: 0"));
        Serial.println(F("Waiting 5 seconds before moving stepper."));
        Serial.println(F("DC motor remains OFF."));
        Serial.println();
    } else {
        // No printing before the arming attempt.
        prepareToDispense();
    }
}

void handlePillDetected() {
    // The ISR already braked the DC motor.
    dcBrake();
    disarmBeam();
    state = BRAKING;
    stateBeganMs = millis();
    if (pillsRemaining > 0) --pillsRemaining;

    // Sensor remains disarmed for THIS pill. Readiness returns as soon
    // as D2 has stayed HIGH continuously for 50 ms.
    announceBeamRearm = true;

    printStage(F("STEP 5 - PILL DETECTED"));
    Serial.println(F("Break beam interrupted. DC motor BRAKING."));
    Serial.print(F("Estimated pills remaining: "));
    Serial.println(pillsRemaining);
    Serial.println();
}

// =============================================================
// SETUP

// =============================================================
void setup() {
    Serial.begin(115200);
    pinMode(BEAM_PIN, INPUT_PULLUP);
    pinMode(NEXT_PIN, INPUT_PULLUP);
    pinMode(POWER_PIN, INPUT_PULLUP);
    pinMode(DC_EN, OUTPUT);
    pinMode(DC_IN1, OUTPUT);
    pinMode(DC_IN2, OUTPUT);
    dcOff();
    for (uint8_t i = 0; i < 4; ++i) {
        pinMode(STEPPER_PINS[i], OUTPUT);
    }
    stepperOff();
    attachInterrupt(digitalPinToInterrupt(BEAM_PIN), beamISR, FALLING);
    printStage(F("PILL DISPENSER READY"));
    Serial.println(F("Serial Monitor: 115200 baud"));
    Serial.println(F("Stepper: direct half-step drive, 3 RPM"));
    Serial.println(F("Sensor: interrupt braking + 50 ms stable-clear rearm"));
    Serial.println(F("No dispensing or stepper timeout"));
    Serial.println();
    Serial.println(F("Press POWER to start the initial refill cycle."));
    Serial.println();
}

// =============================================================
// MAIN LOOP

// =============================================================
void loop() {
    // --- 1. Sensor safety first: refresh readiness, then handle detection. ---
    // A falling edge is recorded even when the motor is not running.
    serviceBeamReadiness();

    const bool pillEvent = readPillEvent();
    if (pillEvent && powerEnabled) {
        if (state == DISPENSING) {
            handlePillDetected();
        } else {
            enterFault(F("Sensor event received outside DISPENSING."));
        }
    }
    // --- 2. POWER button always takes priority over NEXT. ---
    if (buttonPressed(POWER_PIN, powerButton)) {
        if (powerEnabled) {
            turnPowerOff();
        } else if (faultLocked) {
            printStage(F("FAULT STILL LOCKED"));
            Serial.println(F("Inspect and reset Arduino first."));
            Serial.println();
        } else {
            powerEnabled = true;
            if (firstPowerOn) {
                firstPowerOn = false;
                beginPillCycle();
            } else {
                state = IDLE;
                printStage(F("POWER ON - STANDBY"));
                Serial.println(F("Press NEXT to start a dispensing cycle."));
                Serial.println();
            }
        }
    }
    if (!powerEnabled || state == FAULT) {
        if ((uint32_t)(millis() - stateBeganMs) >= BRAKE_MS) dcOff();
        return;
    }
    // --- 3. NEXT starts one cycle only if ready. ---
    if (buttonPressed(NEXT_PIN, nextButton)) {
        if (state == IDLE) {
            beginPillCycle();
        } else {
            Serial.println();
            Serial.println(F("NEXT ignored - cycle already in progress."));
            Serial.println();
        }
    }
    // --- 4. Nonblocking state handling. ---
    switch (state) {
        case REFILL_WAIT:
            if ((uint32_t)(millis() - stateBeganMs) >= REFILL_DELAY_MS) {
                beginStepperOut();
            }
            break;
        case STEPPER_OUT:
        case STEPPER_BACK:
            serviceStepper();
            break;
        case WAIT_FOR_CLEAR_BEAM:
            tryArmAndStartDC();
            break;
        case DISPENSING:
            // Backup detection for a sustained LOW if an interrupt was missed.
            // Count only once, then remain stopped until NEXT.
            if (digitalRead(BEAM_PIN) == LOW) {
                noInterrupts();
                beamArmed = false;
                pillDetectedFlag = false;
                beamFellFlag = true;   // force a new 50 ms clear interval
                beamReady = false;
                interrupts();
                handlePillDetected();
            }
            break;
        case BRAKING:
            if ((uint32_t)(millis() - stateBeganMs) >= BRAKE_MS) {
                dcOff();
                state = IDLE;
                printStage(F("STEP 6 - READY FOR NEXT PILL"));
                Serial.println(F("DC motor OFF."));
                Serial.println(F("Press NEXT to dispense one more pill."));
                Serial.println();
            }
            break;
        case OFF:
        case IDLE:
        case FAULT:
            break;
    }
}