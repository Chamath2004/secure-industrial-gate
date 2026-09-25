/**
 * @file gate_controller.ino
 * @brief Scenario 1 - Secure Industrial Loading Gate (ESP32).
 * @details A simple, non-blocking state machine that controls a servo gate.
 *          Three mode groups are implemented:
 *            - Autonomous : open / close the gate based on the PIR sensor.
 *            - Manual     : an operator drives the gate by hand with a button.
 *            - Safety     : halt, retreat and lock out when an obstruction
 *                           is found while the gate is closing.
 *
 *          Design challenge (recovery routine) chosen for this build:
 *            First obstruction -> HALT -> retreat to open -> wait -> AUTO RETRY.
 *            Second obstruction -> HALT -> retreat -> LOCKOUT until a worker
 *            presses the reset button AND the lane is confirmed clear.
 *
 *          Module: COMP50069 - Hardware, Microcontrollers and Sensors
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

/* ------------------------- Pin assignments ------------------------- */
#define PIR_PIN     13   // Motion sensor        (digital input)
#define TRIG_PIN    5    // Ultrasonic trigger   (digital output)
#define ECHO_PIN    18   // Ultrasonic echo      (digital input)
#define BUZZER_PIN  23   // Buzzer               (PWM output)
#define SERVO_PIN   19   // Gate servo           (PWM output)
#define POT_PIN     34   // Potentiometer        (analogue input / ADC)
#define BUTTON_PIN  27   // Reset / mode button  (digital input, pull-up)

/* ------------------------- OLED display --------------------------- */
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* ------------------------- Actuators ------------------------------ */
Servo   gateServo;   // gate mechanism
ESP32PWM buzzerPWM;  // buzzer on its own PWM timer (kept away from the servo)

/* ------------------------- Constants ------------------------------ */
#define OPEN_ANGLE     180    // servo angle for a fully open gate
#define CLOSED_ANGLE   0      // servo angle for a fully closed gate
#define STEP_MS        30     // ms per 1-degree step (higher = slower sweep)
#define SENSE_MS       50     // how often the ultrasonic sensor is pinged
#define SAFE_DIST_CM   20     // stop closing if something is closer than this
#define HOLD_LONG_MS   3000   // button hold time to enter/exit manual mode
#define DEBOUNCE_MS    50     // ignore presses shorter than this
#define HALT_TIME_MS   5000   // how long to freeze + chirp before retreating
#define RETRY_WAIT_MS  5000   // wait before first automatic closing retry

/* ------------------------- System states -------------------------- */
enum GateState {
  IDLE_OPEN,      // gate open, waiting for the vehicle to leave
  CLOSING,        // gate moving to the closed position
  CLOSED,         // gate fully closed
  OPENING,        // gate moving to the open position
  SAFETY_HALT,    // obstruction found while closing -> frozen + chirping
  RETREATING,     // moving back to open after a halt
  RETRY_WAIT,     // first obstruction recovery delay before retry
  LOCKOUT,        // second obstruction recovery requires reset
  MANUAL_MODE     // operator controls the gate by hand
};
GateState state = CLOSED;   // gate boots secured (closed), opens on vehicle

/* ------------------------- Button events -------------------------- */
enum ButtonEvent { BTN_NONE, BTN_TAP, BTN_HOLD };

/* ------------------------- Live variables ------------------------- */
int   gateAngle    = CLOSED_ANGLE;   // current servo position (boots closed)
int   holdOpenMs   = 5000;           // hold-open delay, set by the potentiometer
int   safeDistCm   = SAFE_DIST_CM;   // safety distance, tunable over serial
int   stepMs       = STEP_MS;        // servo step delay, tunable over serial (SPEED)
float lastDistance = 0;              // last ultrasonic reading (cm)
bool  lastMotion   = false;          // last PIR reading

/* ------------------------- Non-blocking timers ------------------- */
unsigned long lastStepTime   = 0;   // paces the servo sweep
unsigned long lastSenseTime  = 0;   // paces the ultrasonic sensor
unsigned long holdTimer      = 0;   // times the hold-open delay
unsigned long beepEndTime    = 0;   // when the current beep should stop
unsigned long beepTimer      = 0;   // spaces out repeated alarm chirps
unsigned long safetyStart    = 0;   // when the safety halt began
unsigned long retryWaitStart = 0;   // when retry wait began
bool          beepOn         = false;

/* ------------------------- Button tracking ----------------------- */
bool          btnPrev      = HIGH; // previous raw button level
unsigned long btnDownAt    = 0;    // when the button went down
bool          longFired    = false;// long-press already reported this hold
int           manualTarget = -1;   // target angle while toggling in manual mode
int           obstructionCount = 0; // strikes in the current recovery (0..2)
int           obstacleCounter  = 0; // consecutive close readings (debounce)

/* ------------------------- Hardware timer (interrupt) ------------ */
hw_timer_t   *statusTimer = NULL;
volatile bool serialDue   = false; // set by the ISR, cleared in loop()

/**
 * @brief Hardware timer interrupt, fires every 500 ms.
 * @details It only raises a flag. The slow work (printing) is done later in
 *          loop() so the interrupt stays extremely short.
 * @param None
 * @return void
 */
void IRAM_ATTR onStatusTimer() {
  serialDue = true;
}

/**
 * @brief Convert a state value into readable text for the serial log.
 * @param s The current GateState value.
 * @return const char* Text name of the state.
 */
const char *stateName(GateState s) {
  switch (s) {
    case IDLE_OPEN:   return "IDLE_OPEN";
    case CLOSING:     return "CLOSING";
    case CLOSED:      return "CLOSED";
    case OPENING:     return "OPENING";
    case SAFETY_HALT: return "SAFETY_HALT";
    case RETREATING:  return "RETREATING";
    case RETRY_WAIT:  return "RETRY_WAIT";
    case LOCKOUT:     return "LOCKOUT";
    case MANUAL_MODE: return "MANUAL_MODE";
  }
  return "UNKNOWN";
}

/**
 * @brief Start a beep without blocking the loop.
 * @param freq Tone frequency in Hz.
 * @param ms   How long the beep should last in milliseconds.
 * @return void
 */
void beep(int freq, int ms) {
  buzzerPWM.writeTone(freq);
  buzzerPWM.write(1 << 7);      // 128 = 50% duty at 8-bit resolution
  beepEndTime = millis() + ms;
  beepOn = true;
}

/**
 * @brief Silence the buzzer.
 * @param None
 * @return void
 */
void stopBeep() {
  buzzerPWM.write(0);
  beepOn = false;
}

/**
 * @brief Show three lines of text on the OLED.
 * @param l1 Top line.
 * @param l2 Middle line.
 * @param l3 Bottom line.
 * @return void
 */
void showOLED(String l1, String l2, String l3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);  display.println(l1);
  display.setCursor(0, 24); display.println(l2);
  display.setCursor(0, 48); display.println(l3);
  display.display();
}

/**
 * @brief Read the potentiometer (ADC) and map it to the hold-open delay.
 * @details The ADC returns 0-4095; this is mapped to 2-15 seconds so a
 *          supervisor can tune the delay live without changing the code.
 * @param None
 * @return void
 */
void readPot() {
  int raw = analogRead(POT_PIN);
  holdOpenMs = map(raw, 0, 4095, 2000, 15000);
}

/**
 * @brief Measure distance with the ultrasonic sensor.
 * @details Sends a 10 us pulse and times the echo. A 25 ms timeout stops
 *          pulseIn() from blocking the loop when no echo comes back.
 * @param None
 * @return float Distance in cm, or 0 if nothing was detected.
 */
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  if (dur == 0) return 0;
  return dur * 0.0343 / 2.0;
}

/**
 * @brief Move the gate one degree toward a target angle (non-blocking).
 * @details Advances one degree every stepMs milliseconds, so the sweep speed
 *          is set by the SPEED value and stays smooth and even throughout.
 * @param target The angle we want the gate to reach.
 * @return bool True once the gate has reached the target angle.
 */
bool stepGate(int target) {
  if (gateAngle == target) return true;

  if (millis() - lastStepTime >= (unsigned long)stepMs) {
    lastStepTime = millis();
    gateAngle += (target > gateAngle) ? 1 : -1;   // one degree per step
    gateServo.write(gateAngle);
  }
  return gateAngle == target;
}

/* ------------------------- Serial (UART) ------------------------- */

/**
 * @brief Print one line of live status over the serial terminal.
 * @param None
 * @return void
 */
void printStatus() {
  Serial.print("STATE=");    Serial.print(stateName(state));
  Serial.print(" ANGLE=");   Serial.print(gateAngle);
  Serial.print(" DIST=");    Serial.print(lastDistance, 1);
  Serial.print(" MOTION=");  Serial.print(lastMotion ? 1 : 0);
  Serial.print(" HOLD_MS="); Serial.print(holdOpenMs);
  Serial.print(" SAFEDIST=");Serial.print(safeDistCm);
  Serial.print(" SPEED=");   Serial.println(stepMs);
}

/**
 * @brief Handle configuration and control commands from the serial terminal.
 * @details Supported: STATUS, OPEN, CLOSE, AUTO, SAFEDIST=<cm>, SPEED=<ms>, HELP.
 * @param None
 * @return void
 */
void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("STATUS")) {
    printStatus();
  } else if (cmd.startsWith("SAFEDIST=")) {
    int v = cmd.substring(9).toInt();
    if (v > 0 && v < 500) {
      safeDistCm = v;
      Serial.print("OK SAFEDIST="); Serial.println(safeDistCm);
    } else {
      Serial.println("ERR range 1-499");
    }
  } else if (cmd.equalsIgnoreCase("OPEN")) {
    if (state != SAFETY_HALT && state != RETREATING && state != LOCKOUT) {
      state = MANUAL_MODE; manualTarget = OPEN_ANGLE;
      Serial.println("OK MANUAL OPEN");
      showOLED("SERIAL CONTROL", "OPENING...", "TYPE AUTO TO RESUME");
    } else {
      Serial.println("ERR busy (safety active)");
    }
  } else if (cmd.equalsIgnoreCase("CLOSE")) {
    if (state != SAFETY_HALT && state != RETREATING && state != LOCKOUT) {
      state = MANUAL_MODE; manualTarget = CLOSED_ANGLE;
      Serial.println("OK MANUAL CLOSE");
      showOLED("SERIAL CONTROL", "CLOSING...", "TYPE AUTO TO RESUME");
    } else {
      Serial.println("ERR busy (safety active)");
    }
  } else if (cmd.equalsIgnoreCase("AUTO")) {
    if (state == MANUAL_MODE) {
      state = (gateAngle >= OPEN_ANGLE) ? IDLE_OPEN : CLOSED;
      manualTarget = -1; holdTimer = millis();
      Serial.println("OK AUTO RESUMED");
      showOLED("SERIAL CONTROL", "AUTO RESUMED", "");
    } else {
      Serial.println("OK already in auto mode");
    }
  } else if (cmd.startsWith("SPEED=")) {
    int v = cmd.substring(6).toInt();
    if (v >= 5 && v <= 200) {
      stepMs = v;
      Serial.print("OK SPEED="); Serial.print(stepMs); Serial.println(" ms/deg");
    } else {
      Serial.println("ERR SPEED range 5-200 ms/deg");
    }
  } else if (cmd.equalsIgnoreCase("HELP")) {
    Serial.println("Commands: STATUS | OPEN | CLOSE | AUTO | SAFEDIST=<cm> | SPEED=<ms> | HELP");
  } else {
    Serial.println("ERR unknown command, send HELP");
  }
}

/* ------------------------- Button ------------------------------- */

/**
 * @brief Read the button and report a short tap or a 3-second hold.
 * @details Uses INPUT_PULLUP, so a press reads LOW. A basic debounce filters
 *          out very short glitches.
 * @param None
 * @return ButtonEvent BTN_NONE, BTN_TAP or BTN_HOLD.
 */
ButtonEvent readButton() {
  bool raw = digitalRead(BUTTON_PIN);
  ButtonEvent evt = BTN_NONE;

  if (raw == LOW && btnPrev == HIGH) {                 // just pressed
    btnDownAt = millis();
    longFired = false;
  } else if (raw == LOW && !longFired &&
             millis() - btnDownAt >= HOLD_LONG_MS) {    // held 3 s
    longFired = true;
    evt = BTN_HOLD;
  } else if (raw == HIGH && btnPrev == LOW) {           // just released
    if (!longFired && millis() - btnDownAt >= DEBOUNCE_MS) evt = BTN_TAP;
  }

  btnPrev = raw;
  return evt;
}

/* ------------------------- Setup -------------------------------- */

/**
 * @brief Initialise pins, peripherals, the servo, buzzer and hardware timer.
 * @param None
 * @return void
 */
void setup() {
  Serial.begin(115200);

  pinMode(PIR_PIN,   INPUT_PULLDOWN); // floating/disconnected reads LOW, not a false 1
  pinMode(TRIG_PIN,  OUTPUT);
  pinMode(ECHO_PIN,  INPUT);
  pinMode(POT_PIN,   INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Servo on its own PWM timer
  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(CLOSED_ANGLE);

  // Buzzer on a separate PWM timer (so beeping never disturbs the servo)
  ESP32PWM::allocateTimer(1);
  buzzerPWM.attachPin(BUZZER_PIN, 2000, 8);
  buzzerPWM.write(0);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    for (;;);   // stop here if the display is missing
  }
  showOLED("GATE CONTROLLER", "BOOTING...", "");
  delay(1000);  // one-off splash only; the main loop never uses delay()

  // Hardware timer -> raise the serial flag every 500 ms
  statusTimer = timerBegin(1000000);              // 1 MHz -> 1 tick = 1 us
  timerAttachInterrupt(statusTimer, &onStatusTimer);
  timerAlarm(statusTimer, 500000, true, 0);       // 500,000 us = 500 ms

  holdTimer = millis();
  Serial.println("Ready. Send HELP for commands.");
  showOLED("STATUS: READY", "GATE: CLOSED", "");
}

/* ------------------------- Main loop ---------------------------- */

/**
 * @brief Main non-blocking loop: read sensors, then run the state machine.
 * @param None
 * @return void
 */
void loop() {
  // 1) Read the cheap inputs every pass
  readPot();
  lastMotion = digitalRead(PIR_PIN);

  // 1b) Ping the ultrasonic sensor on a fixed schedule only (non-blocking).
  //     Doing this here instead of 5 blocking reads per loop is what keeps
  //     the servo sweep fast and smooth.
  bool newReading = false;
  if (millis() - lastSenseTime >= SENSE_MS) {
    lastSenseTime = millis();
    lastDistance  = readDistance();
    newReading    = true;
  }

  // 2) Serial: handle commands and print status when the timer flags it
  handleSerial();
  if (serialDue) {
    serialDue = false;
    printStatus();
  }

  // 3) Turn the buzzer off once its beep time is up
  if (beepOn && millis() >= beepEndTime) stopBeep();

  // 4) Button: a 3 s hold enters or leaves manual mode
  ButtonEvent evt = readButton();
  if (evt == BTN_HOLD) {
    if (state == MANUAL_MODE) {
      state = (gateAngle >= OPEN_ANGLE) ? IDLE_OPEN : CLOSED;  // resume auto
      holdTimer = millis();
      showOLED("MANUAL OFF", "AUTO RESUMED", "");
    } else if (state != SAFETY_HALT && state != RETREATING && state != LOCKOUT) {
      state = MANUAL_MODE;                                     // enter manual
      manualTarget = -1;
      showOLED("MANUAL ON", "TAP: TOGGLE", "HOLD 3s: EXIT");
    }
    return;   // one action per hold
  }

  // 5) Run the current state
  switch (state) {

    case IDLE_OPEN:
      if (lastMotion) {                       // vehicle present -> keep open
        holdTimer = millis();
        showOLED("VEHICLE DETECTED", "GATE: HOLDING OPEN", "");
      }
      if (!lastMotion && millis() - holdTimer > (unsigned long)holdOpenMs) {
        state = CLOSING;
        showOLED("CLOSING GATE...", "CHECK CLEARANCE", "");
      }
      break;

    case CLOSING:
      // Safety check runs on each fresh ultrasonic reading.
      //   close reading (<= safe)      -> add a strike
      //   confirmed clear reading (> safe) -> reset strikes
      //   0 (echo timeout)             -> ignore, so one dropped ping can
      //                                   never wipe a real detection
      // Two strikes in a row (~100 ms) trigger the halt.
      if (newReading) {
        if (lastDistance > 0 && lastDistance <= safeDistCm) obstacleCounter++;
        else if (lastDistance > safeDistCm)                 obstacleCounter = 0;
      }

      if (obstacleCounter >= 2) {
        obstacleCounter = 0;
        obstructionCount++;
        if (obstructionCount > 2) obstructionCount = 2;

        state = SAFETY_HALT;
        safetyStart = millis();
        beepTimer   = millis();
        beep(2000, 400);
        showOLED("!! SAFETY HALT !!", "OBSTRUCTION!",
                 "DIST: " + String((int)lastDistance) + "cm");
        break;
      }

      if (stepGate(CLOSED_ANGLE)) {
        obstructionCount = 0;                 // clean close clears the strikes
        state = CLOSED;
        showOLED("GATE: CLOSED", "LANE SECURED", "");
      }
      break;

    case CLOSED:
      if (lastMotion) {
        state = OPENING;
        showOLED("VEHICLE ARRIVING", "GATE: OPENING...", "");
      }
      break;

    case OPENING:
      if (stepGate(OPEN_ANGLE)) {
        state = IDLE_OPEN;
        holdTimer = millis();
        showOLED("GATE: OPEN", "READY", "");
      }
      break;

    case SAFETY_HALT:
      // Freeze and chirp for a short window, then retreat to the safe position
      if (millis() - beepTimer > 500) {
        beep(2000, 200);
        beepTimer = millis();
      }
      if (millis() - safetyStart >= HALT_TIME_MS) {
        stopBeep();
        state = RETREATING;
        showOLED("RETREATING", "MOVING TO OPEN", "");
      }
      break;

    case RETREATING:
      if (stepGate(OPEN_ANGLE)) {
        if (obstructionCount == 1) {          // first strike -> auto retry
          retryWaitStart = millis();
          state = RETRY_WAIT;
          showOLED("GATE SAFE", "WAITING 5 SEC", "AUTO RETRY");
        } else {                              // second strike -> lockout
          state = LOCKOUT;
          beepTimer = millis();
          beep(1000, 300);
          showOLED("LOCKOUT", "CLEAR THE LANE", "PRESS RESET BTN");
        }
      }
      break;

    case RETRY_WAIT:
      if (millis() - retryWaitStart >= RETRY_WAIT_MS) {
        state = CLOSING;
        showOLED("SAFETY RETRY", "CLOSING AGAIN", "SENSOR ACTIVE");
      }
      break;

    case LOCKOUT:
      // Stay open. A worker must confirm the lane is clear with a button tap.
      if (millis() - beepTimer > 1500) {
        beep(1000, 150);
        beepTimer = millis();
      }
      if (evt == BTN_TAP) {
        if (lastDistance == 0 || lastDistance >= safeDistCm) {
          obstructionCount = 0;               // strikes cleared by the operator
          state = IDLE_OPEN;
          holdTimer = millis();
          beep(1500, 200);
          showOLED("RESET OK", "AUTO RESUMED", "");
        } else {
          showOLED("STILL BLOCKED", "CLEAR THE LANE", "THEN PRESS RESET");
        }
      }
      break;

    case MANUAL_MODE:
      // A tap starts a toggle; then the gate steps toward the new target
      if (evt == BTN_TAP && manualTarget == -1) {
        manualTarget = (gateAngle >= OPEN_ANGLE) ? CLOSED_ANGLE : OPEN_ANGLE;
        showOLED("MANUAL MODE",
                 manualTarget == OPEN_ANGLE ? "OPENING..." : "CLOSING...", "");
      }

      // Keep the ultrasonic safety check active during manual closing.
      // Manual opening is allowed because it moves the gate away from danger.
      if (manualTarget == CLOSED_ANGLE &&
          lastDistance > 0 && lastDistance <= safeDistCm) {
        manualTarget = -1;
        state = SAFETY_HALT;
        safetyStart = millis();
        beepTimer = millis();
        beep(2000, 400);
        showOLED("!! SAFETY HALT !!", "MANUAL CLOSE BLOCKED",
                 "DIST: " + String((int)lastDistance) + "cm");
        break;
      }

      if (manualTarget != -1 && stepGate(manualTarget)) {
        showOLED("MANUAL MODE",
                 gateAngle >= OPEN_ANGLE ? "GATE: OPEN" : "GATE: CLOSED",
                 "TAP:TOGGLE HOLD:EXIT");
        manualTarget = -1;
      }
      break;
  }
}
