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

// Number of oversampling samples for stable temperature readings
const int NUM_SAMPLES = 16;

// Low-pass filter (Exponential Moving Average / 1st-order IIR) configuration
// alpha is the smoothing factor: 0.0 < alpha <= 1.0 (1.0 = raw / no filtering)
// At 10 Hz sampling rate, alpha = 0.20 provides a cutoff of ~0.35 Hz for smooth tracking
float lpfAlpha = 0.20f;
float filteredTempC = -999.0f; // Seeded with first sample in setup()

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
bool loggingEnabled = true;        // 10 Hz telemetry logging active by default

// Active tuning runtime variables
int currentCycle = 0;
int currentPwm = 0;
unsigned long tuningStartTime = 0;
unsigned long cycleStartTime = 0;
float cyclePeakTemp = -999.0f;
float cycleValleyTemp = 999.0f;

// Cycle metrics storage for autotune PID calculation
const int MAX_STORED_CYCLES = 20;
struct CycleMetric {
  float periodSec;
  float peakTemp;
  float valleyTemp;
};
CycleMetric cycleHistory[MAX_STORED_CYCLES];

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
float readRawTemperature(float &voutOut, float &rawAdcOut);

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

  // Initialize low-pass filter with ambient temperature reading
  float initVout = 0.0f;
  float initRaw = 0.0f;
  filteredTempC = readRawTemperature(initVout, initRaw);

  Serial.println(F("\n=================================================="));
  Serial.println(F("Heated Force-Bench Temperature Controller Ready"));
  Serial.println(F("Control Loop: 10 Hz | Heater Output: Pin 9"));
  Serial.println(F("Type 'HELP' or '?' for available commands."));
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

    // A. Read sensor and calculate raw temperature
    float vout = 0.0f;
    float avgRawADC = 0.0f;
    float rawTempC = readRawTemperature(vout, avgRawADC);

    // B. Apply low-pass filter (Exponential Moving Average / 1st-order IIR)
    if (filteredTempC < -900.0f) {
      filteredTempC = rawTempC;
    } else {
      filteredTempC += lpfAlpha * (rawTempC - filteredTempC);
    }
    float tempC = filteredTempC;

    // C. Update relay tuning state machine
    updateTuningStateMachine(tempC);

    // E. Apply current PWM to pin 9
    analogWrite(PWM_PIN, currentPwm);

    // F. 10 Hz Telemetry output (shows filtered Temp and PWM by default when logging is enabled)
    if (loggingEnabled) {
      Serial.print(F("Temperature: "));
      Serial.print(tempC, 2); // e.g. 25.40 °C (low-pass filtered)
      Serial.print(F(" degC, PWM: "));
      Serial.print(currentPwm);
      if (showVerboseTelemetry) {
        Serial.print(F(", RawTemp: "));
        Serial.print(rawTempC, 2);
        Serial.print(F(" degC, Voltage: "));
        Serial.print(vout, 4); // e.g. 0.7540 V
        Serial.print(F(" V, RawADC: "));
        Serial.print(avgRawADC, 1);
      }
      Serial.println();
    }
  }
}

/**
 * Reads sensor with oversampling and computes raw temperature in °C
 */
float readRawTemperature(float &voutOut, float &rawAdcOut) {
  long adcSum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    adcSum += analogRead(ANALOG_PIN);
  }
  rawAdcOut = (float)adcSum / NUM_SAMPLES;
  voutOut = (rawAdcOut * V_REF) / ADC_RESOLUTION;
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

      if (currentCycle <= MAX_STORED_CYCLES) {
        cycleHistory[currentCycle - 1].periodSec = cycleDurationSec;
        cycleHistory[currentCycle - 1].peakTemp = cyclePeakTemp;
        cycleHistory[currentCycle - 1].valleyTemp = cycleValleyTemp;
      }

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
  }
}

/**
 * Completed all N cycles - compute summary metrics and PID recommendations
 */
void finishTuning() {
  tuneState = STATE_IDLE;
  currentPwm = 0;
  analogWrite(PWM_PIN, 0);

  float totalDurationSec = (millis() - tuningStartTime) / 1000.0f;

  // Average period and amplitude over completed cycles
  float sumPeriod = 0.0f;
  float sumAmplitude = 0.0f;
  int validCycles = min(tuneCycles, MAX_STORED_CYCLES);

  for (int i = 0; i < validCycles; i++) {
    sumPeriod += cycleHistory[i].periodSec;
    sumAmplitude +=
        (cycleHistory[i].peakTemp - cycleHistory[i].valleyTemp) / 2.0f;
  }

  float avgPeriod = sumPeriod / validCycles;                 // Pu (seconds)
  float avgAmplitude = sumAmplitude / validCycles;           // a (°C)
  float relayStep = (float)(tunePwmMax - tunePwmMin) / 2.0f; // d

  // Ultimate Gain: Ku = (4 * d) / (pi * a)
  float ku = 0.0f;
  if (avgAmplitude > 0.001f) {
    ku = (4.0f * relayStep) / (3.14159265f * avgAmplitude);
  }

  // Classic Ziegler-Nichols PID
  float zn_kp = 0.6f * ku;
  float zn_ti = 0.5f * avgPeriod;
  float zn_td = 0.125f * avgPeriod;
  float zn_ki = (zn_ti > 0.001f) ? (zn_kp / zn_ti) : 0.0f;
  float zn_kd = zn_kp * zn_td;

  // Tyreus-Luyben PID (more conservative, reduced overshoot for thermal
  // systems)
  float tl_kp = ku / 3.2f;
  float tl_ti = 2.2f * avgPeriod;
  float tl_td = avgPeriod / 6.3f;
  float tl_ki = (tl_ti > 0.001f) ? (tl_kp / tl_ti) : 0.0f;
  float tl_kd = tl_kp * tl_td;

  Serial.println(F("\n=================================================="));
  Serial.println(F(">>> RELAY AUTOTUNING COMPLETE <<<"));
  Serial.print(F("Total Tuning Time: "));
  Serial.print(totalDurationSec, 1);
  Serial.println(F(" s"));
  Serial.print(F("Completed Cycles: "));
  Serial.println(validCycles);
  Serial.print(F("Avg Oscillation Period (Pu): "));
  Serial.print(avgPeriod, 2);
  Serial.println(F(" s"));
  Serial.print(F("Avg Temp Amplitude (a):      "));
  Serial.print(avgAmplitude, 2);
  Serial.println(F(" degC"));
  Serial.print(F("Ultimate Gain (Ku):          "));
  Serial.println(ku, 3);
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("RECOMMENDED PID GAINS:"));
  Serial.println(F("1. Ziegler-Nichols (Standard):"));
  Serial.print(F("   Kp = "));
  Serial.print(zn_kp, 3);
  Serial.print(F(" | Ki = "));
  Serial.print(zn_ki, 4);
  Serial.print(F(" | Kd = "));
  Serial.println(zn_kd, 3);
  Serial.print(F("   (Ti = "));
  Serial.print(zn_ti, 2);
  Serial.print(F(" s | Td = "));
  Serial.print(zn_td, 2);
  Serial.println(F(" s)"));
  Serial.println(
      F("2. Tyreus-Luyben (Less Overshoot - Recommended for Thermal):"));
  Serial.print(F("   Kp = "));
  Serial.print(tl_kp, 3);
  Serial.print(F(" | Ki = "));
  Serial.print(tl_ki, 4);
  Serial.print(F(" | Kd = "));
  Serial.println(tl_kd, 3);
  Serial.print(F("   (Ti = "));
  Serial.print(tl_ti, 2);
  Serial.print(F(" s | Td = "));
  Serial.print(tl_td, 2);
  Serial.println(F(" s)"));
  Serial.println(F("Heater is safely turned OFF (PWM=0)."));
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
  Serial.print(F("  LPF Alpha: "));
  Serial.print(lpfAlpha, 2);
  Serial.println(F(" (1.0 = raw, lower = smoother)"));
  Serial.print(F("  Telemetry: "));
  Serial.println(showVerboseTelemetry ? F("Verbose (Temp, PWM, Voltage, Raw)")
                                      : F("Default (Temp, PWM only)"));
  Serial.print(F("  Logging:   "));
  Serial.println(loggingEnabled ? F("ENABLED (Streaming at 10 Hz)")
                                : F("DISABLED (Stopped)"));
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
  Serial.println(F("    Example: TUNE 40 50 0 255 3"));
  Serial.println(F("  START [LOG]"));
  Serial.println(
      F("    Start relay tuning, or 'START LOG' to enable telemetry logging."));
  Serial.println(F("  STOP [LOG]"));
  Serial.println(
      F("    Cancel tuning & heater OFF, or 'STOP LOG' to disable telemetry."));
  Serial.println(F("  LOG [ON/OFF|START/STOP|1/0]"));
  Serial.println(
      F("    Start, stop, or toggle 10 Hz telemetry logging stream."));
  Serial.println(F("  PWM <val>"));
  Serial.println(
      F("    Set manual PWM (0 - 255) on pin 9 (aborts active tuning)."));
  Serial.println(F("  FILTER <alpha>"));
  Serial.println(
      F("    Set lowpass filter factor (0.01 - 1.0, default: 0.20)."));
  Serial.println(F("  VERBOSE [ON/OFF]"));
  Serial.println(
      F("    Toggle Voltage & Raw ADC in telemetry (Default: OFF, Temp & PWM only)."));
  Serial.println(F("  SET TMIN <val>   - Set Temp Min threshold in degC"));
  Serial.println(F("  SET TMAX <val>   - Set Temp Max threshold in degC"));
  Serial.println(F("  SET PMIN <val>   - Set PWM Min (0 - 255)"));
  Serial.println(F("  SET PMAX <val>   - Set PWM Max (0 - 255)"));
  Serial.println(F("  SET CYCLES <n>   - Set number of relay cycles (1 - 20)"));
  Serial.println(F("  SET ALPHA <val>  - Set LPF filter factor (0.01 - 1.0)"));
  Serial.println(F("  SET VERBOSE <1/0>- Enable/disable verbose telemetry"));
  Serial.println(F("  SET LOG <1/0>    - Enable/disable telemetry logging"));
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
      F("  3. Repeats N times, shuts off heater, and computes PID gains."));
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

  // Use currently filtered temperature for commands
  float currentTemp = (filteredTempC > -900.0f) ? filteredTempC : 25.0f;

  char *token = strtok(cmd, " \t,");
  if (!token)
    return;

  if (strcasecmp(token, "HELP") == 0 || strcmp(token, "?") == 0) {
    printHelp();
  } else if (strcasecmp(token, "STATUS") == 0) {
    printStatus(currentTemp);
  } else if (strcasecmp(token, "STOP") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (arg && (strcasecmp(arg, "LOG") == 0 || strcasecmp(arg, "LOGGING") == 0)) {
      loggingEnabled = false;
      Serial.println(F("[LOG] Telemetry logging STOPPED (10 Hz stream disabled)"));
    } else {
      stopTuning(true);
    }
  } else if (strcasecmp(token, "START") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (arg && (strcasecmp(arg, "LOG") == 0 || strcasecmp(arg, "LOGGING") == 0)) {
      loggingEnabled = true;
      Serial.println(F("[LOG] Telemetry logging STARTED (10 Hz stream enabled)"));
    } else {
      startTuning(currentTemp);
    }
  } else if (strcasecmp(token, "LOG") == 0 || strcasecmp(token, "LOGGING") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (!arg) {
      loggingEnabled = !loggingEnabled;
    } else if (strcasecmp(arg, "ON") == 0 || strcasecmp(arg, "START") == 0 ||
               strcmp(arg, "1") == 0 || strcasecmp(arg, "ENABLE") == 0) {
      loggingEnabled = true;
    } else if (strcasecmp(arg, "OFF") == 0 || strcasecmp(arg, "STOP") == 0 ||
               strcmp(arg, "0") == 0 || strcasecmp(arg, "DISABLE") == 0) {
      loggingEnabled = false;
    } else {
      Serial.println(
          F("[ERROR] Usage: LOG [ON/OFF|START/STOP|1/0] or START LOG / STOP LOG"));
      return;
    }
    Serial.print(F("[LOG] Telemetry logging: "));
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
          F("[ERROR] Usage: SET <TMIN|TMAX|PMIN|PMAX|CYCLES|ALPHA|VERBOSE|LOG> <value>"));
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
    } else if (strcasecmp(param, "ALPHA") == 0 || strcasecmp(param, "LPF") == 0) {
      lpfAlpha = constrain(atof(val), 0.01f, 1.0f);
      Serial.print(F("[CONFIG] LPF Alpha set to: "));
      Serial.println(lpfAlpha, 2);
    } else if (strcasecmp(param, "VERBOSE") == 0) {
      showVerboseTelemetry = (atoi(val) != 0 || strcasecmp(val, "ON") == 0 ||
                              strcasecmp(val, "TRUE") == 0);
      Serial.print(F("[CONFIG] Verbose telemetry: "));
      Serial.println(showVerboseTelemetry ? F("ON") : F("OFF"));
    } else if (strcasecmp(param, "LOG") == 0 || strcasecmp(param, "LOGGING") == 0) {
      loggingEnabled = (atoi(val) != 0 || strcasecmp(val, "ON") == 0 ||
                        strcasecmp(val, "TRUE") == 0 || strcasecmp(val, "START") == 0 ||
                        strcasecmp(val, "ENABLE") == 0);
      Serial.print(F("[CONFIG] Telemetry logging: "));
      Serial.println(loggingEnabled ? F("ENABLED (Streaming at 10 Hz)")
                                    : F("DISABLED (Stopped)"));
    } else {
      Serial.println(F(
          "[ERROR] Unknown parameter. Valid: TMIN, TMAX, PMIN, PMAX, CYCLES, ALPHA, VERBOSE, LOG"));
    }
  } else if (strcasecmp(token, "FILTER") == 0 || strcasecmp(token, "ALPHA") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (arg) {
      lpfAlpha = constrain(atof(arg), 0.01f, 1.0f);
      Serial.print(F("[CONFIG] LPF Alpha set to: "));
      Serial.println(lpfAlpha, 2);
    } else {
      Serial.print(F("[CONFIG] Current LPF Alpha: "));
      Serial.println(lpfAlpha, 2);
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
    Serial.println(showVerboseTelemetry ? F("ON (Temp, PWM, Voltage, Raw)")
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