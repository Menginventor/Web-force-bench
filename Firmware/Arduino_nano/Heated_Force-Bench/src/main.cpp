#include <Arduino.h>
#include <EEPROM.h>

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

// PI Closed-Loop Controller Configuration & State
float piKp = 2.0f;
float piKi = 0.05f;
float piUmin = 0.0f;
float piUmax = 255.0f;
float piSetpoint = 45.0f; // Target temperature in degC
bool piControlActive = false;
float piIntegral = 0.0f;

// EEPROM Persistent Storage Structure
struct PiConfig {
  uint32_t magic;      // 0x50494333 ("PIC3")
  float kp;
  float ki;
  float u_min;
  float u_max;
};

const uint32_t EEPROM_MAGIC = 0x50494333;
const int EEPROM_ADDR = 0;

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
void updatePiController(float currentTemp);
void startController();
void stopController(bool notifyUser = true);
void saveConfigToEeprom(bool verbose = true);
void loadConfigFromEeprom();

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

  // Load persistent PI controller settings from EEPROM
  loadConfigFromEeprom();

  Serial.println(F("\n=================================================="));
  Serial.println(F("Heated Force-Bench Temperature Controller Ready"));
  Serial.println(F("Control Loop: 10 Hz | Closed-Loop PI & Autotune"));
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

    // B. Update closed-loop controllers & tuning
    if (tuneState != STATE_IDLE) {
      updateTuningStateMachine(tempC);
    } else if (piControlActive) {
      updatePiController(tempC);
    }

    // C. Apply current PWM to pin 9
    analogWrite(PWM_PIN, currentPwm);

    // D. 10 Hz Telemetry output (structured $DAT: protocol to separate plot data from logs)
    if (loggingEnabled || tuneState != STATE_IDLE || piControlActive) {
      Serial.print(F("$DAT:Temp="));
      Serial.print(tempC, 2); // e.g. 25.40
      Serial.print(F(",PWM="));
      Serial.print(currentPwm);
      if (piControlActive) {
        Serial.print(F(",SP="));
        Serial.print(piSetpoint, 2);
      }
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
  if (piControlActive) {
    stopController(false);
  }
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
 * PI Closed-Loop Temperature Controller
 * Features Anti-Windup using Instant Back-Calculation:
 *   u_P = Kp * error
 *   u_I = integral
 *   if ((u_P + u_I) > u_max) { u_I = u_max - u_P; integral = u_I; }
 *   if ((u_P + u_I) < u_min) { u_I = u_min - u_P; integral = u_I; }
 */
void updatePiController(float currentTemp) {
  if (!piControlActive) {
    return;
  }

  float error = piSetpoint - currentTemp;
  float u_P = piKp * error;
  piIntegral += piKi * error * 0.1f; // dt = 0.1s (10 Hz control loop)
  float u_I = piIntegral;

  // Upper saturation
  if ((u_P + u_I) > piUmax) {
    u_I = piUmax - u_P;
    if (u_I < 0.0f) {
      u_I = 0.0f; // Prevent integral from going negative during heating saturation!
    }
    piIntegral = u_I;
  }

  // Lower saturation
  if ((u_P + u_I) < piUmin) {
    u_I = piUmin - u_P;
    if (u_I > 0.0f) {
      u_I = 0.0f; // Prevent integral from going positive during cooling saturation!
    }
    piIntegral = u_I;
  }

  float u = u_P + u_I;
  currentPwm = constrain((int)round(u), (int)round(piUmin), (int)round(piUmax));
}

void startController() {
  if (tuneState != STATE_IDLE) {
    stopTuning(false);
  }
  piControlActive = true;
  piIntegral = 0.0f; // Reset integral accumulator on start
  Serial.println(F("\n=================================================="));
  Serial.println(F(">>> PI TEMPERATURE CONTROLLER STARTED <<<"));
  Serial.print(F("Setpoint (SP): "));
  Serial.print(piSetpoint, 2);
  Serial.println(F(" degC"));
  Serial.print(F("Kp:            "));
  Serial.println(piKp, 3);
  Serial.print(F("Ki:            "));
  Serial.println(piKi, 4);
  Serial.print(F("U_min:         "));
  Serial.print(piUmin, 1);
  Serial.print(F(" | U_max: "));
  Serial.println(piUmax, 1);
  Serial.println(F("Anti-windup:   Instant Back-Calculation Active"));
  Serial.println(F("=================================================="));
}

void stopController(bool notifyUser) {
  piControlActive = false;
  currentPwm = 0;
  analogWrite(PWM_PIN, 0);
  piIntegral = 0.0f;
  if (notifyUser) {
    Serial.println(F("\n[CONTROL] PI Controller STOPPED. Heater turned OFF (PWM=0)."));
  }
}

void saveConfigToEeprom(bool verbose) {
  PiConfig cfg;
  cfg.magic = EEPROM_MAGIC;
  cfg.kp = piKp;
  cfg.ki = piKi;
  cfg.u_min = piUmin;
  cfg.u_max = piUmax;
  EEPROM.put(EEPROM_ADDR, cfg);
  if (verbose) {
    Serial.println(F("[EEPROM] Configuration saved successfully."));
  }
}

void loadConfigFromEeprom() {
  PiConfig cfg;
  EEPROM.get(EEPROM_ADDR, cfg);
  if (cfg.magic == EEPROM_MAGIC && !isnan(cfg.kp) && !isnan(cfg.ki) &&
      !isnan(cfg.u_min) && !isnan(cfg.u_max)) {
    piKp = cfg.kp;
    piKi = cfg.ki;
    piUmin = constrain(cfg.u_min, 0.0f, 255.0f);
    piUmax = constrain(cfg.u_max, 0.0f, 255.0f);
  } else {
    // Defaults
    piKp = 2.0f;
    piKi = 0.05f;
    piUmin = 0.0f;
    piUmax = 255.0f;
    saveConfigToEeprom(false);
  }
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
  Serial.print(F("Control Mode: "));
  if (piControlActive) {
    Serial.println(F("PI CLOSED-LOOP ACTIVE"));
    Serial.print(F("  Setpoint:   "));
    Serial.print(piSetpoint, 2);
    Serial.print(F(" degC (Error: "));
    Serial.print(piSetpoint - currentTemp, 2);
    Serial.println(F(" degC)"));
    Serial.print(F("  u_I (Integ):"));
    Serial.println(piIntegral, 2);
  } else if (tuneState != STATE_IDLE) {
    Serial.println(F("RELAY AUTOTUNING ACTIVE"));
  } else {
    Serial.println(F("IDLE (Manual)"));
  }
  Serial.print(F("Current Setpoint: "));
  Serial.print(piSetpoint, 2);
  Serial.println(F(" degC"));
  Serial.println(F("PI Settings (EEPROM):"));
  Serial.print(F("  Kp:         "));
  Serial.println(piKp, 3);
  Serial.print(F("  Ki:         "));
  Serial.println(piKi, 4);
  Serial.print(F("  U_min:      "));
  Serial.println(piUmin, 1);
  Serial.print(F("  U_max:      "));
  Serial.println(piUmax, 1);
  Serial.println(F("Tuning Settings:"));
  Serial.print(F("  T_min:      "));
  Serial.print(tuneTempMin, 2);
  Serial.println(F(" degC"));
  Serial.print(F("  T_max:      "));
  Serial.print(tuneTempMax, 2);
  Serial.println(F(" degC"));
  Serial.print(F("  PWM_min:    "));
  Serial.println(tunePwmMin);
  Serial.print(F("  PWM_max:    "));
  Serial.println(tunePwmMax);
  Serial.print(F("  Cycles:     "));
  Serial.println(tuneCycles);
  Serial.print(F("Telemetry:    "));
  Serial.println(showVerboseTelemetry ? F("Verbose (Temp, PWM, Voltage, RawADC)")
                                      : F("Default (Temp, PWM, SP)"));
  Serial.print(F("Reporting:    "));
  if (loggingEnabled) {
    Serial.println(F("ENABLED (Streaming at 10 Hz)"));
  } else if (tuneState != STATE_IDLE || piControlActive) {
    Serial.println(F("ACTIVE (Streaming control data)"));
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
      F("    Show current temperatures, PI state, PWM and parameters."));
  Serial.println(F("  START [CONTROL|TUNE|REPORT]"));
  Serial.println(
      F("    Start closed-loop PI controller, or autotuning, or reporting."));
  Serial.println(F("  STOP [CONTROL|TUNE|REPORT]"));
  Serial.println(
      F("    Stop PI controller / autotuning & shut heater OFF."));
  Serial.println(F("  CONTROL [ON/OFF|1/0] or PI [ON/OFF|1/0]"));
  Serial.println(
      F("    Start or stop closed-loop PI temperature controller."));
  Serial.println(F("  SP <temp> or SETPOINT <temp>"));
  Serial.println(
      F("    Set target temperature in degC (Runtime setting)."));
  Serial.println(F("  KP <val> / KI <val>"));
  Serial.println(
      F("    Set PI proportional / integral gain (saved to EEPROM)."));
  Serial.println(F("  UMIN <val> / UMAX <val>"));
  Serial.println(
      F("    Set output PWM saturation limits 0 - 255 (saved to EEPROM)."));
  Serial.println(F("  SAVE / LOAD"));
  Serial.println(
      F("    Manually save or reload PI parameters to / from EEPROM."));
  Serial.println(F("  TUNE <t_min> <t_max> <pwm_min> <pwm_max> <cycles>"));
  Serial.println(
      F("    Configure parameters and start relay tuning immediately."));
  Serial.println(F("  REPORT [ON/OFF|ENABLE/DISABLE|START/STOP|1/0]"));
  Serial.println(
      F("    Start, stop, or toggle 10 Hz status telemetry reporting (Default: OFF)."));
  Serial.println(F("  PWM <val>"));
  Serial.println(
      F("    Set manual PWM (0 - 255) on pin 9 (aborts active closed-loop)."));
  Serial.println(F("  VERBOSE [ON/OFF]"));
  Serial.println(
      F("    Toggle Voltage & Raw ADC in telemetry stream."));
  Serial.println(F("  SET SP <val>     - Set target setpoint in degC"));
  Serial.println(F("  SET KP <val>     - Set proportional gain Kp"));
  Serial.println(F("  SET KI <val>     - Set integral gain Ki"));
  Serial.println(F("  SET UMIN <val>   - Set minimum output saturation limit"));
  Serial.println(F("  SET UMAX <val>   - Set maximum output saturation limit"));
  Serial.println(F("  SET TMIN <val>   - Set Temp Min threshold in degC"));
  Serial.println(F("  SET TMAX <val>   - Set Temp Max threshold in degC"));
  Serial.println(F("  SET PMIN <val>   - Set PWM Min (0 - 255)"));
  Serial.println(F("  SET PMAX <val>   - Set PWM Max (0 - 255)"));
  Serial.println(F("  SET CYCLES <n>   - Set number of relay cycles (1 - 20)"));
  Serial.println(F("  SET VERBOSE <1/0>- Enable/disable verbose telemetry"));
  Serial.println(F("  SET REPORT <1/0> - Enable/disable telemetry reporting"));
  Serial.println(
      F("--------------------------------------------------------------"));
  Serial.println(F("ANTI-WINDUP PROTOCOL:"));
  Serial.println(
      F("  Instant back-calculation clamps integral accumulation when"));
  Serial.println(
      F("  actuator saturates at U_max or U_min to eliminate overshoot lag."));
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
    } else if (arg && (strcasecmp(arg, "CONTROL") == 0 || strcasecmp(arg, "PI") == 0)) {
      stopController(true);
    } else if (arg && (strcasecmp(arg, "TUNE") == 0 || strcasecmp(arg, "TUNING") == 0)) {
      stopTuning(true);
    } else {
      // Generic STOP: stop whichever closed-loop mode is active
      if (piControlActive) {
        stopController(true);
      }
      if (tuneState != STATE_IDLE) {
        stopTuning(true);
      }
      if (!piControlActive && tuneState == STATE_IDLE) {
        currentPwm = 0;
        analogWrite(PWM_PIN, 0);
        Serial.println(F("[MANUAL] Heater turned OFF (PWM=0)."));
      }
    }
  } else if (strcasecmp(token, "START") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (arg && (strcasecmp(arg, "LOG") == 0 || strcasecmp(arg, "LOGGING") == 0 ||
                strcasecmp(arg, "REPORT") == 0 || strcasecmp(arg, "REPORTING") == 0)) {
      loggingEnabled = true;
      Serial.println(F("[REPORT] Telemetry reporting STARTED (10 Hz stream enabled)"));
    } else if (arg && (strcasecmp(arg, "TUNE") == 0 || strcasecmp(arg, "TUNING") == 0)) {
      startTuning(currentTemp);
    } else {
      // Default START: start PI closed-loop controller
      startController();
    }
  } else if (strcasecmp(token, "CONTROL") == 0 || strcasecmp(token, "PI") == 0) {
    char *arg = strtok(NULL, " \t,");
    if (!arg) {
      if (piControlActive) {
        stopController(true);
      } else {
        startController();
      }
    } else if (strcasecmp(arg, "ON") == 0 || strcasecmp(arg, "START") == 0 ||
               strcasecmp(arg, "ENABLE") == 0 || strcmp(arg, "1") == 0) {
      startController();
    } else if (strcasecmp(arg, "OFF") == 0 || strcasecmp(arg, "STOP") == 0 ||
               strcasecmp(arg, "DISABLE") == 0 || strcmp(arg, "0") == 0) {
      stopController(true);
    } else {
      Serial.println(F("[ERROR] Usage: CONTROL [ON/OFF|1/0] or START CONTROL / STOP CONTROL"));
    }
  } else if (strcasecmp(token, "SP") == 0 || strcasecmp(token, "SETPOINT") == 0) {
    char *val = strtok(NULL, " \t,");
    if (val) {
      piSetpoint = atof(val);
      Serial.print(F("[CONFIG] Setpoint set to: "));
      Serial.print(piSetpoint, 2);
      Serial.println(F(" degC"));
    } else {
      Serial.print(F("[CONFIG] Current Setpoint: "));
      Serial.print(piSetpoint, 2);
      Serial.println(F(" degC"));
    }
  } else if (strcasecmp(token, "KP") == 0) {
    char *val = strtok(NULL, " \t,");
    if (val) {
      piKp = atof(val);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] Kp set to: "));
      Serial.print(piKp, 3);
      Serial.println(F(" (Saved to EEPROM)"));
    } else {
      Serial.print(F("[CONFIG] Current Kp: "));
      Serial.println(piKp, 3);
    }
  } else if (strcasecmp(token, "KI") == 0) {
    char *val = strtok(NULL, " \t,");
    if (val) {
      piKi = atof(val);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] Ki set to: "));
      Serial.print(piKi, 4);
      Serial.println(F(" (Saved to EEPROM)"));
    } else {
      Serial.print(F("[CONFIG] Current Ki: "));
      Serial.println(piKi, 4);
    }
  } else if (strcasecmp(token, "UMIN") == 0) {
    char *val = strtok(NULL, " \t,");
    if (val) {
      piUmin = constrain(atof(val), 0.0f, piUmax);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] U_min set to: "));
      Serial.print(piUmin, 1);
      Serial.println(F(" (Saved to EEPROM)"));
    } else {
      Serial.print(F("[CONFIG] Current U_min: "));
      Serial.println(piUmin, 1);
    }
  } else if (strcasecmp(token, "UMAX") == 0) {
    char *val = strtok(NULL, " \t,");
    if (val) {
      piUmax = constrain(atof(val), piUmin, 255.0f);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] U_max set to: "));
      Serial.print(piUmax, 1);
      Serial.println(F(" (Saved to EEPROM)"));
    } else {
      Serial.print(F("[CONFIG] Current U_max: "));
      Serial.println(piUmax, 1);
    }
  } else if (strcasecmp(token, "SAVE") == 0) {
    saveConfigToEeprom(true);
  } else if (strcasecmp(token, "LOAD") == 0) {
    loadConfigFromEeprom();
    Serial.println(F("[EEPROM] Configuration loaded from EEPROM."));
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
          F("[ERROR] Usage: SET <SP|KP|KI|UMIN|UMAX|TMIN|TMAX|PMIN|PMAX|CYCLES|VERBOSE|REPORT> <value>"));
      return;
    }
    if (strcasecmp(param, "SP") == 0 || strcasecmp(param, "SETPOINT") == 0 || strcasecmp(param, "TARGET") == 0) {
      piSetpoint = atof(val);
      Serial.print(F("[CONFIG] Setpoint set to: "));
      Serial.print(piSetpoint, 2);
      Serial.println(F(" degC"));
    } else if (strcasecmp(param, "KP") == 0) {
      piKp = atof(val);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] Kp set to: "));
      Serial.print(piKp, 3);
      Serial.println(F(" (Saved to EEPROM)"));
    } else if (strcasecmp(param, "KI") == 0) {
      piKi = atof(val);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] Ki set to: "));
      Serial.print(piKi, 4);
      Serial.println(F(" (Saved to EEPROM)"));
    } else if (strcasecmp(param, "UMIN") == 0) {
      piUmin = constrain(atof(val), 0.0f, piUmax);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] U_min set to: "));
      Serial.print(piUmin, 1);
      Serial.println(F(" (Saved to EEPROM)"));
    } else if (strcasecmp(param, "UMAX") == 0) {
      piUmax = constrain(atof(val), piUmin, 255.0f);
      saveConfigToEeprom(false);
      Serial.print(F("[CONFIG] U_max set to: "));
      Serial.print(piUmax, 1);
      Serial.println(F(" (Saved to EEPROM)"));
    } else if (strcasecmp(param, "CONTROL") == 0 || strcasecmp(param, "PI") == 0) {
      if (atoi(val) != 0 || strcasecmp(val, "ON") == 0 || strcasecmp(val, "START") == 0 || strcasecmp(val, "TRUE") == 0) {
        startController();
      } else {
        stopController(true);
      }
    } else if (strcasecmp(param, "TMIN") == 0) {
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
          F("[ERROR] Unknown parameter. Valid: SP, KP, KI, UMIN, UMAX, TMIN, TMAX, PMIN, PMAX, CYCLES, VERBOSE, REPORT"));
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
      stopController(false);
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