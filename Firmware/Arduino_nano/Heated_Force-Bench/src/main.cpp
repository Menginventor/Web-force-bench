#include <Arduino.h>

/**
 * Heated Force-Bench Temperature Measurement & Relay Autotuning Protocol
 * Hardware:
 *   - Board: Arduino Nano (ATmega328P)
 *   - Sensor: Microchip MCP9700 / MCP9701 Linear Active Thermistor IC
 *   - Input Pin: Analog Pin A7 (ADC7)
 *   - Heater PWM Pin: Pin 9 (Timer1 PWM)
 *   - Analog Reference: External 3.3V connected to AREF pin
 *
 * Datasheet specifications (MCP970X.pdf - Section 4.0, Equation 4-1):
 *   Vout = Tc * Ta + V0C
 *   Ta = (Vout - V0C) / Tc
 *
 * MCP9700 / MCP9700A / MCP9700B:
 *   - V0C = 500 mV (0.500 V) at 0 deg C
 *   - Tc  = 10.0 mV/deg C (0.010 V/deg C)
 *
 * MCP9701 / MCP9701A:
 *   - V0C = 400 mV (0.400 V) at 0 deg C
 *   - Tc  = 19.5 mV/deg C (0.0195 V/deg C)
 */

// Pin definitions
const int ANALOG_PIN = A7;
const int PWM_PIN = 9; // Connect to MOSFET/driver for heating element

// Calibration & ADC constants
const float V_REF = 3.30f; // 3.3V external reference connected to AREF pin
const float ADC_RESOLUTION = 1023.0f; // 10-bit ADC (0 - 1023)

// Sensor model selection: Set to true for MCP9700, false for MCP9701
#define USE_MCP9700 1

#if USE_MCP9700
// MCP9700 / MCP9700A
const float V_0C = 0.500f; // 500 mV output at 0 °C
const float TC = 0.010f;   // 10.0 mV/°C temperature coefficient
#else
// MCP9701 / MCP9701A
const float V_0C = 0.400f; // 400 mV output at 0 °C
const float TC = 0.0195f;  // 19.5 mV/°C temperature coefficient
#endif

// Direct temperature reading (no ADC filter or averaging)
float currentTempC = -999.0f;

// 10 Hz Control Loop Timing (100 ms)
const unsigned long LOOP_INTERVAL_MS = 100;
unsigned long lastLoopTime = 0;

// Tuning State Machine
enum TuningState {
  STATE_IDLE,      // Tuning inactive / manual control
  STATE_PREP_COOL, // Initial temp > temp_min: cool down with pwm_min until temp
                   // <= temp_min
  STATE_PREP_HEAT, // Initial temp < temp_min: heat up with pwm_max until temp
                   // >= temp_min
  STATE_RELAY_HEAT, // Relay oscillation: heat up with pwm_max until temp >=
                    // temp_max
  STATE_RELAY_COOL  // Relay oscillation: cool down with pwm_min until temp <=
                    // temp_min
};

TuningState tuneState = STATE_IDLE;

// User configurable tuning parameters (defaults)
float tuneTempMin = 40.0f; // Target lower threshold in degC
float tuneTempMax = 50.0f; // Target upper threshold in degC
int tunePwmMin = 0;        // Minimum PWM (0 - 255)
int tunePwmMax = 255;      // Maximum PWM (0 - 255)
int tuneCycles = 3;        // Number of relay cycles N (1 - 20)
bool showVerboseTelemetry = false; // Default: only show Temp and PWM
bool loggingEnabled = false;        // Telemetry reporting disabled by default

// Active tuning runtime variables
int currentCycle = 0;
int currentPwm = 0;
unsigned long tuningStartTime = 0;
unsigned long cycleStartTime = 0;
float cyclePeakTemp = -999.0f;
float cycleValleyTemp = 999.0f;

// Maximum relay tuning cycles
const int MAX_STORED_CYCLES = 20;

// Serial command buffer
char serialBuffer[64];
uint8_t bufferIdx = 0;

// Forward declarations
void handleSerialInput();
void executeCommand(char *cmd);
void printHelp();
void printStatus(float currentTemp);
void startTuning(float currentTemp);
void stopTuning(bool notifyUser);
void updateTuningStateMachine(float currentTemp);
void finishTuning();
float readTemperature(float &voutOut, int &rawAdcOut);

void setup() {
  // CRITICAL: Set analog reference to EXTERNAL BEFORE any analogRead()
  // to avoid shorting internal reference to external 3.3V AREF pin.
  analogReference(EXTERNAL);

  // Initialize PWM pin
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  currentPwm = 0;

  // Initialize Serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // Wait briefly for serial initialization
  }

  // Discard first reading after changing voltage reference to allow AREF
  // capacitor to settle
  analogRead(ANALOG_PIN);
  delay(10);

  // Initial temperature reading directly from ADC
  float initVout = 0.0f;
  int initRaw = 0;
  currentTempC = readTemperature(initVout, initRaw);

  Serial.println(F("\n=================================================="));
  Serial.println(F("Heated Force-Bench Temperature Controller Ready"));
  Serial.println(F("Control Loop: 10 Hz | Heater Output: Pin 9"));
  Serial.println(F("Type 'HELP' or '?' for available commands."));
  Serial.println(F("Telemetry Reporting: OFF by default ('REPORT ON' to enable)"));
  Serial.println(F("=================================================="));

  lastLoopTime = millis();
}

void loop() {
  // 1. Process serial commands non-blockingly
  handleSerialInput();

  // 2. Run read and control loop strictly at 10 Hz (every 100 ms)
  unsigned long currentMillis = millis();
  if (currentMillis - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime += LOOP_INTERVAL_MS;
    if (currentMillis - lastLoopTime > LOOP_INTERVAL_MS) {
      lastLoopTime = currentMillis; // Prevent slip accumulation
    }

    // A. Read sensor directly from ADC (single conversion, no filter or averaging)
    float vout = 0.0f;
    int rawADC = 0;
    float tempC = readTemperature(vout, rawADC);
    currentTempC = tempC;

    // B. Update relay tuning state machine
    updateTuningStateMachine(tempC);

    // C. Apply current PWM to pin 9
    analogWrite(PWM_PIN, currentPwm);

    // D. 10 Hz Telemetry output (structured $DAT: protocol to separate plot data from logs)
    if (loggingEnabled || tuneState != STATE_IDLE) {
      Serial.print(F("$DAT:Temp="));
      Serial.print(tempC, 2); // e.g. 25.40
      Serial.print(F(",PWM="));
      Serial.print(currentPwm);
      if (showVerboseTelemetry) {
        Serial.print(F(",Voltage="));
        Serial.print(vout, 4); // e.g. 0.7540
        Serial.print(F(",RawADC="));
        Serial.print(rawADC);
      }
      Serial.println();
    }
  }
}

/**
 * Reads sensor directly from ADC (single conversion, no filter or average) and computes temperature in °C
 */
float readTemperature(float &voutOut, int &rawAdcOut) {
  rawAdcOut = analogRead(ANALOG_PIN);
  voutOut = ((float)rawAdcOut * V_REF) / ADC_RESOLUTION;
  float tempC = (voutOut - V_0C) / TC;
  return tempC;
}

/**
 * Relay Autotuning State Machine update
 */
void updateTuningStateMachine(float currentTemp) {
  switch (tuneState) {
  case STATE_IDLE:
    // Nothing to update in idle mode
    break;

  case STATE_PREP_COOL:
    // Initial cooling to reach temp_min
    currentPwm = tunePwmMin;
    if (currentTemp <= tuneTempMin) {
      // Crossed temp_min threshold! Start Relay Cycle 1
      Serial.print(F("[TUNE] Crossed Temp Min ("));
      Serial.print(currentTemp, 2);
      Serial.println(F(" degC) -> Starting Relay Cycle 1!"));

      currentCycle = 1;
      tuneState = STATE_RELAY_HEAT;
      currentPwm = tunePwmMax;
      cycleStartTime = millis();
      cyclePeakTemp = currentTemp;
      cycleValleyTemp = currentTemp;

      Serial.print(F("[TUNE] Cycle 1/"));
      Serial.print(tuneCycles);
      Serial.print(F(" [HEATING]: PWM="));
      Serial.print(tunePwmMax);
      Serial.print(F(" targeting "));
      Serial.print(tuneTempMax, 2);
      Serial.println(F(" degC"));
    }
    break;

  case STATE_PREP_HEAT:
    // Initial heating to reach temp_min
    currentPwm = tunePwmMax;
    if (currentTemp >= tuneTempMin) {
      // Crossed temp_min threshold! Start Relay Cycle 1
      Serial.print(F("[TUNE] Crossed Temp Min ("));
      Serial.print(currentTemp, 2);
      Serial.println(F(" degC) -> Starting Relay Cycle 1!"));

      currentCycle = 1;
      tuneState = STATE_RELAY_HEAT;
      currentPwm = tunePwmMax;
      cycleStartTime = millis();
      cyclePeakTemp = currentTemp;
      cycleValleyTemp = currentTemp;

      Serial.print(F("[TUNE] Cycle 1/"));
      Serial.print(tuneCycles);
      Serial.print(F(" [HEATING]: PWM="));
      Serial.print(tunePwmMax);
      Serial.print(F(" targeting "));
      Serial.print(tuneTempMax, 2);
      Serial.println(F(" degC"));
    }
    break;

  case STATE_RELAY_HEAT:
    currentPwm = tunePwmMax;
    if (currentTemp > cyclePeakTemp) {
      cyclePeakTemp = currentTemp;
    }
    if (currentTemp >= tuneTempMax) {
      // Reached temp_max -> Switch to cooling
      tuneState = STATE_RELAY_COOL;
      currentPwm = tunePwmMin;
      cycleValleyTemp = currentTemp;

      Serial.print(F("[TUNE] Cycle "));
      Serial.print(currentCycle);
      Serial.print(F("/"));
      Serial.print(tuneCycles);
      Serial.print(F(" [COOLING]: Reached "));
      Serial.print(currentTemp, 2);
      Serial.print(F(" degC. PWM="));
      Serial.print(tunePwmMin);
      Serial.print(F(" targeting "));
      Serial.print(tuneTempMin, 2);
      Serial.println(F(" degC"));
    }
    break;

  case STATE_RELAY_COOL:
    currentPwm = tunePwmMin;
    if (currentTemp < cycleValleyTemp) {
      cycleValleyTemp = currentTemp;
    }
    if (currentTemp <= tuneTempMin) {
      // Completed this cycle!
      float cycleDurationSec = (millis() - cycleStartTime) / 1000.0f;


      Serial.print(F("[TUNE] >>> Cycle "));
      Serial.print(currentCycle);
      Serial.print(F("/"));
      Serial.print(tuneCycles);
      Serial.print(F(" Complete! Period: "));
      Serial.print(cycleDurationSec, 1);
      Serial.print(F("s | Peak: "));
      Serial.print(cyclePeakTemp, 2);
      Serial.print(F(" degC | Valley: "));
      Serial.print(cycleValleyTemp, 2);
      Serial.println(F(" degC"));

      if (currentCycle < tuneCycles) {
        // Proceed to next cycle
        currentCycle++;
        tuneState = STATE_RELAY_HEAT;
        currentPwm = tunePwmMax;
        cycleStartTime = millis();
        cyclePeakTemp = currentTemp;
        cycleValleyTemp = currentTemp;

        Serial.print(F("[TUNE] Cycle "));
        Serial.print(currentCycle);
        Serial.print(F("/"));
        Serial.print(tuneCycles);
        Serial.print(F(" [HEATING]: PWM="));
        Serial.print(tunePwmMax);
        Serial.print(F(" targeting "));
        Serial.print(tuneTempMax, 2);
        Serial.println(F(" degC"));
      } else {
        // All cycles complete!
        finishTuning();
      }
    }
    break;
  }
}

/**
 * Start tuning sequence with current parameters
 */
void startTuning(float currentTemp) {
  // Validate parameters
  if (tuneTempMax <= tuneTempMin) {
    Serial.println(F("[ERROR] Temp Max must be greater than Temp Min!"));
    return;
  }
  if (tunePwmMax <= tunePwmMin) {
    Serial.println(F("[ERROR] PWM Max must be greater than PWM Min!"));
    return;
  }
  if (tuneCycles < 1) {
    Serial.println(F("[ERROR] Number of cycles must be at least 1!"));
    return;
  }
  Serial.println(F("\n=================================================="));
  Serial.println(F(">>> STARTING RELAY AUTOTUNING PROTOCOL <<<"));
  Serial.print(F("T_min: "));
  Serial.print(tuneTempMin, 2);
  Serial.print(F(" degC | T_max: "));
  Serial.print(tuneTempMax, 2);
  Serial.print(F(" degC\nPWM_min: "));
  Serial.print(tunePwmMin);
  Serial.print(F(" | PWM_max: "));
  Serial.print(tunePwmMax);
  Serial.print(F(" | Cycles: "));
  Serial.println(tuneCycles);
  Serial.print(F("Current Temp: "));
  Serial.print(currentTemp, 2);
  Serial.println(F(" degC"));
  if (!loggingEnabled) {
    Serial.println(F("[REPORT] Telemetry reporting active during autotuning."));
  }

  tuningStartTime = millis();
  currentCycle = 0;

  if (currentTemp > tuneTempMin) {
    // Current temp is above temp_min: cool down with pwm_min
    tuneState = STATE_PREP_COOL;
    currentPwm = tunePwmMin;
    Serial.print(F("[TUNE] Phase: Initial Cooldown (PWM="));
    Serial.print(tunePwmMin);
    Serial.print(F(") to reach T_min ("));
    Serial.print(tuneTempMin, 2);
    Serial.println(F(" degC)..."));
  } else if (currentTemp < tuneTempMin) {
    // Current temp is below temp_min: heat up with pwm_max
    tuneState = STATE_PREP_HEAT;
    currentPwm = tunePwmMax;
    Serial.print(F("[TUNE] Phase: Initial Warmup (PWM="));
    Serial.print(tunePwmMax);
    Serial.print(F(") to reach T_min ("));
    Serial.print(tuneTempMin, 2);
    Serial.println(F(" degC)..."));
  } else {
    // Exactly at temp_min: start Cycle 1 immediately
    tuneState = STATE_RELAY_HEAT;
    currentCycle = 1;
    currentPwm = tunePwmMax;
    cycleStartTime = millis();
    cyclePeakTemp = currentTemp;
    cycleValleyTemp = currentTemp;

    Serial.print(F("[TUNE] Cycle 1/"));
    Serial.print(tuneCycles);
    Serial.print(F(" [HEATING]: Starting at T_min. PWM="));
    Serial.print(tunePwmMax);
    Serial.println(F(" targeting T_max"));
  }
  Serial.println(F("=================================================="));
}

/**
 * Immediately stop tuning and shut off heater
 */
void stopTuning(bool notifyUser) {
  tuneState = STATE_IDLE;
  currentPwm = 0;
  analogWrite(PWM_PIN, 0);

  if (notifyUser) {
    Serial.println(F("\n[TUNE] *** PROCEDURE STOPPED: Heater turned OFF "
                     "immediately (PWM=0) ***"));
    if (!loggingEnabled) {
      Serial.println(F("[REPORT] Autotuning stopped: Telemetry stream stopped."));
    }
  }
}

/**
 * Completed all N cycles - finish tuning and shut off heater
 */
void finishTuning() {
  tuneState = STATE_IDLE;
  currentPwm = 0;
  analogWrite(PWM_PIN, 0);

  Serial.println(F("\n=================================================="));
  Serial.println(F(">>> RELAY AUTOTUNING COMPLETE <<<"));
  Serial.println(F("Heater is safely turned OFF (PWM=0)."));
  if (!loggingEnabled) {
    Serial.println(F("[REPORT] Autotuning complete: Telemetry stream stopped."));
  }
  Serial.println(F("=================================================="));
}

/**
 * Prints current controller status
 */
void printStatus(float currentTemp) {
  Serial.println(F("\n----------------- CONTROLLER STATUS -----------------"));
  Serial.print(F("Current Temp: "));
  Serial.print(currentTemp, 2);
  Serial.println(F(" degC"));
  Serial.print(F("Current PWM:  "));
  Serial.print(currentPwm);
  Serial.println(F(" (Pin 9)"));
  Serial.print(F("Tuning State: "));
  switch (tuneState) {
  case STATE_IDLE:
    Serial.println(F("IDLE"));
    break;
  case STATE_PREP_COOL:
    Serial.println(F("PREPARATION: Cooldown to reach T_min"));
    break;
  case STATE_PREP_HEAT:
    Serial.println(F("PREPARATION: Warmup to reach T_min"));
    break;
  case STATE_RELAY_HEAT:
    Serial.println(F("RELAY: Heating to reach T_max"));
    break;
  case STATE_RELAY_COOL:
    Serial.println(F("RELAY: Cooling to reach T_min"));
    break;
  }
  if (tuneState != STATE_IDLE) {
    Serial.print(F("Progress:     Cycle "));
    Serial.print(currentCycle);
    Serial.print(F(" of "));
    Serial.println(tuneCycles);
  }
  Serial.println(F("Tuning Settings:"));
  Serial.print(F("  T_min:  "));
  Serial.print(tuneTempMin, 2);
  Serial.println(F(" degC"));
  Serial.print(F("  T_max:  "));
  Serial.print(tuneTempMax, 2);
  Serial.println(F(" degC"));
  Serial.print(F("  PWM_min: "));
  Serial.println(tunePwmMin);
  Serial.print(F("  PWM_max: "));
  Serial.println(tunePwmMax);
  Serial.print(F("  Cycles:  "));
  Serial.println(tuneCycles);
  Serial.print(F("  Telemetry: "));
  Serial.println(showVerboseTelemetry ? F("Verbose (Temp, PWM, Voltage, RawADC)")
                                      : F("Default (Temp, PWM only)"));
  Serial.print(F("  Reporting: "));
  if (loggingEnabled) {
    Serial.println(F("ENABLED (Streaming at 10 Hz)"));
  } else if (tuneState != STATE_IDLE) {
    Serial.println(F("ACTIVE (Autotuning in progress)"));
  } else {
    Serial.println(F("DISABLED (Default off)"));
  }
  Serial.println(F("-----------------------------------------------------"));
}

/**
 * Prints user help menu
 */
void printHelp() {
  Serial.println(
      F("\n=============================================================="));
  Serial.println(F("             HEATED FORCE-BENCH COMMAND MENU"));
  Serial.println(
      F("=============================================================="));
  Serial.println(F("COMMANDS:"));
  Serial.println(F("  HELP or ?"));
  Serial.println(F("    Display this help menu."));
  Serial.println(F("  STATUS"));
  Serial.println(
      F("    Show current temperatures, state, PWM and parameters."));
  Serial.println(F("  TUNE <t_min> <t_max> <pwm_min> <pwm_max> <cycles>"));
  Serial.println(
      F("    Configure parameters and start relay tuning immediately."));
  Serial.println(F("    (Telemetry reporting is automatically active during tuning)"));
  Serial.println(F("    Example: TUNE 40 50 0 255 3"));
  Serial.println(F("  START [REPORT|LOG]"));
  Serial.println(
      F("    Start relay tuning, or 'START REPORT' to enable telemetry stream."));
  Serial.println(F("  STOP [REPORT|LOG]"));
  Serial.println(
      F("    Cancel tuning & heater OFF, or 'STOP REPORT' to stop telemetry."));
  Serial.println(F("  REPORT [ON/OFF|ENABLE/DISABLE|START/STOP|1/0]"));
  Serial.println(
      F("    Start, stop, or toggle 10 Hz status telemetry reporting (Default: OFF)."));
  Serial.println(F("  ENABLE [REPORT|LOG] / DISABLE [REPORT|LOG]"));
  Serial.println(
      F("    Enable or disable 10 Hz telemetry stream."));
  Serial.println(F("  PWM <val>"));
  Serial.println(
      F("    Set manual PWM (0 - 255) on pin 9 (aborts active tuning)."));
  Serial.println(F("  VERBOSE [ON/OFF]"));
  Serial.println(
      F("    Toggle Voltage & Raw ADC in telemetry (Default: OFF, Temp & PWM only)."));
  Serial.println(F("  SET TMIN <val>   - Set Temp Min threshold in degC"));
  Serial.println(F("  SET TMAX <val>   - Set Temp Max threshold in degC"));
  Serial.println(F("  SET PMIN <val>   - Set PWM Min (0 - 255)"));
  Serial.println(F("  SET PMAX <val>   - Set PWM Max (0 - 255)"));
  Serial.println(F("  SET CYCLES <n>   - Set number of relay cycles (1 - 20)"));
  Serial.println(F("  SET VERBOSE <1/0>- Enable/disable verbose telemetry"));
  Serial.println(F("  SET REPORT <1/0> - Enable/disable telemetry reporting"));
  Serial.println(
      F("--------------------------------------------------------------"));
  Serial.println(F("TUNING PROTOCOL BEHAVIOR:"));
  Serial.println(
      F("  1. If Current Temp > T_min -> Cool down with PWM_min to T_min."));
  Serial.println(
      F("     If Current Temp < T_min -> Heat up with PWM_max to T_min."));
  Serial.println(F("  2. Upon crossing T_min -> Start relay cycles:"));
  Serial.println(
      F("     Heat with PWM_max to T_max, then cool with PWM_min to T_min."));
  Serial.println(
      F("  3. Repeats N times and safely shuts off heater."));
  Serial.println(
      F("=============================================================="));
}

/**
 * Handle non-blocking incoming serial bytes
 */
void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (bufferIdx > 0) {
        serialBuffer[bufferIdx] = '\0';
        executeCommand(serialBuffer);
        bufferIdx = 0;
      }
    } else {
      if (bufferIdx < (sizeof(serialBuffer) - 1)) {
        serialBuffer[bufferIdx++] = c;
      }
    }
  }
}

/**
 * Parses and executes a command string
 */
void executeCommand(char *cmd) {
  // Trim leading whitespace
  while (*cmd == ' ' || *cmd == '\t')
    cmd++;
  if (*cmd == '\0')
    return;

  // Use latest direct temperature reading for commands
  float currentTemp = (currentTempC > -900.0f) ? currentTempC : 25.0f;

  char *token = strtok(cmd, " \t,");
  if (!token)
    return;

  if (strcasecmp(token, "HELP") == 0 || strcmp(token, "?") == 0) {
    printHelp();
  } else if (strcasecmp(token, "STATUS") == 0) {
    printStatus(currentTemp);
  } else if (strcasecmp(token, "STOP") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (arg && (strcasecmp(arg, "LOG") == 0 || strcasecmp(arg, "LOGGING") == 0 ||
                strcasecmp(arg, "REPORT") == 0 || strcasecmp(arg, "REPORTING") == 0)) {
      loggingEnabled = false;
      Serial.println(F("[REPORT] Telemetry reporting STOPPED (10 Hz stream disabled)"));
    } else {
      stopTuning(true);
    }
  } else if (strcasecmp(token, "START") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (arg && (strcasecmp(arg, "LOG") == 0 || strcasecmp(arg, "LOGGING") == 0 ||
                strcasecmp(arg, "REPORT") == 0 || strcasecmp(arg, "REPORTING") == 0)) {
      loggingEnabled = true;
      Serial.println(F("[REPORT] Telemetry reporting STARTED (10 Hz stream enabled)"));
    } else {
      startTuning(currentTemp);
    }
  } else if (strcasecmp(token, "ENABLE") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (!arg || strcasecmp(arg, "REPORT") == 0 || strcasecmp(arg, "REPORTING") == 0 ||
        strcasecmp(arg, "LOG") == 0 || strcasecmp(arg, "LOGGING") == 0 || strcasecmp(arg, "STATUS") == 0) {
      loggingEnabled = true;
      Serial.println(F("[REPORT] Telemetry reporting ENABLED (10 Hz stream enabled)"));
    } else {
      Serial.println(F("[ERROR] Usage: ENABLE [REPORT|LOG]"));
    }
  } else if (strcasecmp(token, "DISABLE") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (!arg || strcasecmp(arg, "REPORT") == 0 || strcasecmp(arg, "REPORTING") == 0 ||
        strcasecmp(arg, "LOG") == 0 || strcasecmp(arg, "LOGGING") == 0 || strcasecmp(arg, "STATUS") == 0) {
      loggingEnabled = false;
      Serial.println(F("[REPORT] Telemetry reporting DISABLED (10 Hz stream disabled)"));
    } else {
      Serial.println(F("[ERROR] Usage: DISABLE [REPORT|LOG]"));
    }
  } else if (strcasecmp(token, "REPORT") == 0 || strcasecmp(token, "REPORTING") == 0 ||
             strcasecmp(token, "LOG") == 0 || strcasecmp(token, "LOGGING") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (!arg) {
      loggingEnabled = !loggingEnabled;
    } else if (strcasecmp(arg, "ON") == 0 || strcasecmp(arg, "START") == 0 ||
               strcasecmp(arg, "ENABLE") == 0 || strcasecmp(arg, "ENABLED") == 0 ||
               strcmp(arg, "1") == 0) {
      loggingEnabled = true;
    } else if (strcasecmp(arg, "OFF") == 0 || strcasecmp(arg, "STOP") == 0 ||
               strcasecmp(arg, "DISABLE") == 0 || strcasecmp(arg, "DISABLED") == 0 ||
               strcmp(arg, "0") == 0) {
      loggingEnabled = false;
    } else if (strcasecmp(arg, "STATUS") == 0) {
      printStatus(currentTemp);
      return;
    } else {
      Serial.println(
          F("[ERROR] Usage: REPORT [ON/OFF|ENABLE/DISABLE|START/STOP|1/0] or ENABLE/DISABLE REPORT"));
      return;
    }
    Serial.print(F("[REPORT] Telemetry reporting: "));
    Serial.println(loggingEnabled ? F("ENABLED (Streaming at 10 Hz)")
                                  : F("DISABLED (Stopped)"));
  } else if (strcasecmp(token, "TUNE") == 0) {
    char *arg1 = strtok(NULL, " \t,");
    char *arg2 = strtok(NULL, " \t,");
    char *arg3 = strtok(NULL, " \t,");
    char *arg4 = strtok(NULL, " \t,");
    char *arg5 = strtok(NULL, " \t,");

    if (arg1 && arg2 && arg3 && arg4 && arg5) {
      tuneTempMin = atof(arg1);
      tuneTempMax = atof(arg2);
      tunePwmMin = constrain(atoi(arg3), 0, 255);
      tunePwmMax = constrain(atoi(arg4), 0, 255);
      tuneCycles = constrain(atoi(arg5), 1, MAX_STORED_CYCLES);
      startTuning(currentTemp);
    } else if (!arg1) {
      // TUNE without arguments: start using current parameters
      startTuning(currentTemp);
    } else {
      Serial.println(F(
          "[ERROR] Usage: TUNE <t_min> <t_max> <pwm_min> <pwm_max> <cycles>"));
      Serial.println(F("        Example: TUNE 40 50 0 255 3"));
    }
  } else if (strcasecmp(token, "SET") == 0) {
    char *param = strtok(NULL, " \t,=");
    char *val = strtok(NULL, " \t,=");
    if (!param || !val) {
      Serial.println(
          F("[ERROR] Usage: SET <TMIN|TMAX|PMIN|PMAX|CYCLES|VERBOSE|REPORT> <value>"));
      return;
    }
    if (strcasecmp(param, "TMIN") == 0) {
      tuneTempMin = atof(val);
      Serial.print(F("[CONFIG] Temp Min set to: "));
      Serial.print(tuneTempMin, 2);
      Serial.println(F(" degC"));
    } else if (strcasecmp(param, "TMAX") == 0) {
      tuneTempMax = atof(val);
      Serial.print(F("[CONFIG] Temp Max set to: "));
      Serial.print(tuneTempMax, 2);
      Serial.println(F(" degC"));
    } else if (strcasecmp(param, "PMIN") == 0) {
      tunePwmMin = constrain(atoi(val), 0, 255);
      Serial.print(F("[CONFIG] PWM Min set to: "));
      Serial.println(tunePwmMin);
    } else if (strcasecmp(param, "PMAX") == 0) {
      tunePwmMax = constrain(atoi(val), 0, 255);
      Serial.print(F("[CONFIG] PWM Max set to: "));
      Serial.println(tunePwmMax);
    } else if (strcasecmp(param, "CYCLES") == 0) {
      tuneCycles = constrain(atoi(val), 1, MAX_STORED_CYCLES);
      Serial.print(F("[CONFIG] Cycles set to: "));
      Serial.println(tuneCycles);
    } else if (strcasecmp(param, "VERBOSE") == 0) {
      showVerboseTelemetry = (atoi(val) != 0 || strcasecmp(val, "ON") == 0 ||
                              strcasecmp(val, "TRUE") == 0);
      Serial.print(F("[CONFIG] Verbose telemetry: "));
      Serial.println(showVerboseTelemetry ? F("ON") : F("OFF"));
    } else if (strcasecmp(param, "LOG") == 0 || strcasecmp(param, "LOGGING") == 0 ||
               strcasecmp(param, "REPORT") == 0 || strcasecmp(param, "REPORTING") == 0) {
      loggingEnabled = (atoi(val) != 0 || strcasecmp(val, "ON") == 0 ||
                        strcasecmp(val, "TRUE") == 0 || strcasecmp(val, "START") == 0 ||
                        strcasecmp(val, "ENABLE") == 0);
      Serial.print(F("[CONFIG] Telemetry reporting: "));
      Serial.println(loggingEnabled ? F("ENABLED (Streaming at 10 Hz)")
                                    : F("DISABLED (Stopped)"));
    } else {
      Serial.println(
          F("[ERROR] Unknown parameter. Valid: TMIN, TMAX, PMIN, PMAX, CYCLES, VERBOSE, REPORT (or LOG)"));
    }
  } else if (strcasecmp(token, "VERBOSE") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (arg) {
      showVerboseTelemetry = (atoi(arg) != 0 || strcasecmp(arg, "ON") == 0 ||
                              strcasecmp(arg, "TRUE") == 0);
    } else {
      showVerboseTelemetry = !showVerboseTelemetry;
    }
    Serial.print(F("[CONFIG] Verbose telemetry: "));
    Serial.println(showVerboseTelemetry ? F("ON (Temp, PWM, Voltage, RawADC)")
                                        : F("OFF (Temp, PWM only)"));
  } else if (strcasecmp(token, "PWM") == 0) {
    char *val = strtok(NULL, " \t,");
    if (val) {
      stopTuning(false);
      currentPwm = constrain(atoi(val), 0, 255);
      analogWrite(PWM_PIN, currentPwm);
      Serial.print(F("[MANUAL] PWM set to: "));
      Serial.println(currentPwm);
    } else {
      Serial.print(F("[MANUAL] Current PWM: "));
      Serial.println(currentPwm);
    }
  } else {
    Serial.print(F("[ERROR] Unknown command '"));
    Serial.print(token);
    Serial.println(F("'. Type 'HELP' or '?' for command list."));
  }
}