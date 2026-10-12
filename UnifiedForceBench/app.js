/**
 * Unified Force-Bench Controller - Main Application Logic
 * Integrates Robotis U2D2 Dynamixel Motion (1 rev = 1 mm lead screw)
 * and Arduino Heated Loadcell Telemetry Suite into a unified interface.
 */

class UnifiedForceBenchApp {
  constructor() {
    // ------------------------------------------------------------------------
    // Lead Screw Mechanical Specifications
    // ------------------------------------------------------------------------
    this.LEAD_PITCH_MM = 1.0;          // 1 rev = 1 mm
    this.TICKS_PER_REV = 4096;         // Protocol 2.0 (XL430/XM430)
    this.TICKS_PER_MM = 4096.0;        // 4096 ticks = 1 mm linear travel
    this.CURRENT_SPEED_MM_S = 1.0;      // Default 1.0 mm/s (60 RPM)

    // ------------------------------------------------------------------------
    // Port 1: U2D2 / Dynamixel State
    // ------------------------------------------------------------------------
    this.motorPort = null;
    this.motorReader = null;
    this.motorWriter = null;
    this.motorKeepReading = false;
    this.isMotorConnected = false;
    this.motorActiveId = 1;
    this.motorProtocolVersion = "2.0";
    this.motorTorqueEnabled = false;
    this.motorOperatingMode = 4;        // Default: Mode 4 Extended Position (Multiturn)
    this.motorPresentTicks = 0;
    this.motorPresentVelocity = 0;
    this.motorRxBuffer = [];
    this.motorActiveResolver = null;
    this.motorPollTimer = null;
    this.motorIsPollingBusy = false;
    this.motorIsContinuousMoving = false;

    // ------------------------------------------------------------------------
    // Port 2: Arduino Heated Loadcell State
    // ------------------------------------------------------------------------
    this.arduinoPort = null;
    this.arduinoReader = null;
    this.arduinoWriter = null;
    this.arduinoKeepReading = false;
    this.isArduinoConnected = false;
    this.arduinoBuffer = "";
    this.liveTempC = null;
    this.liveWeightKg = 0.0;
    this.liveForceN = 0.0;
    this.livePwm = 0;
    this.liveSetpoint = 45.0;
    this.heaterActive = false;
    this.isForceUnitNewton = true;       // true: Newtons (N), false: Kilograms (kg)

    // ------------------------------------------------------------------------
    // Telemetry Oscilloscope Buffers (Rolling 30s)
    // ------------------------------------------------------------------------
    this.tempHistory = [];              // { t, temp, sp }
    this.forceHistory = [];             // { t, forceN, weightKg }
    this.historyDurationSec = 30;
    this.startTime = performance.now();
    this.peakTemp = -Infinity;
    this.peakForceN = 0.0;

    // ------------------------------------------------------------------------
    // Automated Push / Pull Mechanical Test State
    // ------------------------------------------------------------------------
    this.isTestRunning = false;
    this.testDataPoints = [];           // Array of { t, dispMm, forceN, tempC }
    this.testDirection = "push";        // "push" (compression / +) | "pull" (tension / -)
    this.testTravelMm = 10.0;
    this.testSpeedMmS = 1.0;
    this.testMaxForceLimitN = 50.0;
    this.testStartMm = 0.0;
    this.testPeakForceN = 0.0;
    this.testDispAtPeakMm = 0.0;
    this.testStiffnessNPerMm = 0.0;
    this.testEnergyMJ = 0.0;
    this.testPollInterval = null;

    // ------------------------------------------------------------------------
    // Initialization
    // ------------------------------------------------------------------------
    this.cacheDOMElements();
    this.bindEvents();
    this.initCanvasPlots();
    this.startPlotAnimationLoop();
    this.log("SYS", "Unified Force-Bench Application initialized. Ready to connect ports.", "bench");
  }

  // --------------------------------------------------------------------------
  // DOM Cache
  // --------------------------------------------------------------------------
  cacheDOMElements() {
    this.el = {
      // Ports
      cardMotorPort: document.getElementById('cardMotorPort'),
      indMotor: document.getElementById('indMotor'),
      motorBaudSelect: document.getElementById('motorBaudSelect'),
      motorIdInput: document.getElementById('motorIdInput'),
      btnConnectMotor: document.getElementById('btnConnectMotor'),

      cardArduinoPort: document.getElementById('cardArduinoPort'),
      indArduino: document.getElementById('indArduino'),
      arduinoBaudSelect: document.getElementById('arduinoBaudSelect'),
      btnConnectArduino: document.getElementById('btnConnectArduino'),
      btnEmergencyStop: document.getElementById('btnEmergencyStop'),

      // Motion Axis
      badgeTorqueState: document.getElementById('badgeTorqueState'),
      dispActiveId: document.getElementById('dispActiveId'),
      dispLinearMm: document.getElementById('dispLinearMm'),
      dispRevs: document.getElementById('dispRevs'),
      dispTicks: document.getElementById('dispTicks'),
      btnSyncMotor: document.getElementById('btnSyncMotor'),
      dispCurrentSpeed: document.getElementById('dispCurrentSpeed'),
      inputCustomSpeed: document.getElementById('inputCustomSpeed'),
      btnApplyCustomSpeed: document.getElementById('btnApplyCustomSpeed'),
      dispContSpeed: document.getElementById('dispContSpeed'),
      btnFastRewind: document.getElementById('btnFastRewind'),
      btnHaltMotion: document.getElementById('btnHaltMotion'),
      btnFastForward: document.getElementById('btnFastForward'),
      btnResetEncoder: document.getElementById('btnResetEncoder'),
      btnGotoOrigin: document.getElementById('btnGotoOrigin'),
      btnToggleTorque: document.getElementById('btnToggleTorque'),
      labelToggleTorque: document.getElementById('labelToggleTorque'),
      btnHomingOffset: document.getElementById('btnHomingOffset'),

      // Canvases
      canvasTempPlot: document.getElementById('canvasTempPlot'),
      canvasForcePlot: document.getElementById('canvasForcePlot'),
      canvasPushPullPlot: document.getElementById('canvasPushPullPlot'),

      // Plot Stats
      statLiveTemp: document.getElementById('statLiveTemp'),
      statLiveSp: document.getElementById('statLiveSp'),
      statPeakTemp: document.getElementById('statPeakTemp'),
      statLiveForce: document.getElementById('statLiveForce'),
      statPeakForce: document.getElementById('statPeakForce'),
      btnToggleUnits: document.getElementById('btnToggleUnits'),
      badgeTestState: document.getElementById('badgeTestState'),
      statTestPeak: document.getElementById('statTestPeak'),
      statTestStiffness: document.getElementById('statTestStiffness'),

      // Thermal Chamber & Loadcell
      dispHeaterState: document.getElementById('dispHeaterState'),
      dispChamberTemp: document.getElementById('dispChamberTemp'),
      dispChamberPwm: document.getElementById('dispChamberPwm'),
      inputTempSetpoint: document.getElementById('inputTempSetpoint'),
      btnSetTempSp: document.getElementById('btnSetTempSp'),
      btnStartHeater: document.getElementById('btnStartHeater'),
      btnStopHeater: document.getElementById('btnStopHeater'),
      sliderManualPwm: document.getElementById('sliderManualPwm'),
      dispManualPwmVal: document.getElementById('dispManualPwmVal'),
      dispLoadcellWeight: document.getElementById('dispLoadcellWeight'),
      btnTareLoadcell: document.getElementById('btnTareLoadcell'),
      btnCalibratePrompt: document.getElementById('btnCalibratePrompt'),

      // Automated Mechanical Test
      dispTestPhase: document.getElementById('dispTestPhase'),
      selectTestDirection: document.getElementById('selectTestDirection'),
      inputTestDistance: document.getElementById('inputTestDistance'),
      inputTestSpeed: document.getElementById('inputTestSpeed'),
      inputMaxForceLimit: document.getElementById('inputMaxForceLimit'),
      checkAutoReturn: document.getElementById('checkAutoReturn'),
      checkAutoTare: document.getElementById('checkAutoTare'),
      btnExecuteTest: document.getElementById('btnExecuteTest'),
      labelExecuteTest: document.getElementById('labelExecuteTest'),
      cardTestResults: document.getElementById('cardTestResults'),
      resPeakForce: document.getElementById('resPeakForce'),
      resPeakDisp: document.getElementById('resPeakDisp'),
      resStiffness: document.getElementById('resStiffness'),
      resEnergy: document.getElementById('resEnergy'),
      btnExportCsv: document.getElementById('btnExportCsv'),

      // Terminal
      termOutput: document.getElementById('termOutput'),
      btnClearTerm: document.getElementById('btnClearTerm'),
    };
  }

  // --------------------------------------------------------------------------
  // Event Binding
  // --------------------------------------------------------------------------
  bindEvents() {
    // Port Connections
    this.el.btnConnectMotor.addEventListener('click', () => this.toggleMotorConnection());
    this.el.btnConnectArduino.addEventListener('click', () => this.toggleArduinoConnection());
    this.el.btnEmergencyStop.addEventListener('click', () => this.triggerEmergencyStop());

    // Motor ID Input change
    this.el.motorIdInput.addEventListener('change', (e) => {
      this.motorActiveId = parseInt(e.target.value, 10) || 1;
      this.el.dispActiveId.textContent = `ID ${this.motorActiveId} (XL430)`;
      if (this.isMotorConnected) this.syncMotorHardware();
    });

    // Speed Presets
    document.querySelectorAll('.speed-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        this.setJogSpeed(parseFloat(btn.dataset.speed));
      });
    });

    this.el.btnApplyCustomSpeed.addEventListener('click', () => {
      const spd = parseFloat(this.el.inputCustomSpeed.value);
      if (spd > 0) this.setJogSpeed(spd);
    });

    // Incremental Step Jog (+ / - mm)
    document.querySelectorAll('[data-jog]').forEach(btn => {
      btn.addEventListener('click', () => {
        const deltaMm = parseFloat(btn.dataset.jog);
        this.jogRelativeMm(deltaMm);
      });
    });

    // Continuous Motion
    this.el.btnFastForward.addEventListener('click', () => this.startContinuousMove(1));
    this.el.btnFastRewind.addEventListener('click', () => this.startContinuousMove(-1));
    this.el.btnHaltMotion.addEventListener('click', () => this.haltMotion());

    // Origin and Hardware Tools
    this.el.btnResetEncoder.addEventListener('click', () => this.resetMultiTurnEncoder());
    this.el.btnGotoOrigin.addEventListener('click', () => this.gotoOriginMm());
    this.el.btnToggleTorque.addEventListener('click', () => this.toggleTorque());
    this.el.btnSyncMotor.addEventListener('click', () => this.syncMotorHardware(true));

    // Force Units Toggle
    this.el.btnToggleUnits.addEventListener('click', () => {
      this.isForceUnitNewton = !this.isForceUnitNewton;
      this.el.btnToggleUnits.textContent = this.isForceUnitNewton ? "UNIT: NEWTON (N)" : "UNIT: KILOGRAM (KG)";
      this.updateForceStats();
    });

    // Heater Controls
    this.el.btnSetTempSp.addEventListener('click', () => {
      const sp = parseFloat(this.el.inputTempSetpoint.value);
      this.setHeaterSetpoint(sp);
    });

    document.querySelectorAll('.preset-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const sp = parseFloat(btn.dataset.sp);
        this.el.inputTempSetpoint.value = sp;
        this.setHeaterSetpoint(sp);
      });
    });

    this.el.btnStartHeater.addEventListener('click', () => this.enableHeater());
    this.el.btnStopHeater.addEventListener('click', () => this.stopHeater());

    this.el.sliderManualPwm.addEventListener('input', (e) => {
      const pwm = parseInt(e.target.value, 10);
      this.el.dispManualPwmVal.textContent = `PWM: ${pwm}`;
      this.sendArduinoCommand(`PWM ${pwm}`);
    });

    // Load Cell Controls
    this.el.btnTareLoadcell.addEventListener('click', () => this.tareLoadcell());
    this.el.btnCalibratePrompt.addEventListener('click', () => {
      const wStr = prompt("Enter known calibration test weight in kg (e.g. 0.500):", "0.500");
      if (wStr) this.sendArduinoCommand(`CAL ${wStr}`);
    });

    // Automated Push / Pull Test
    this.el.btnExecuteTest.addEventListener('click', () => {
      if (this.isTestRunning) {
        this.abortTestSequence();
      } else {
        this.startTestSequence();
      }
    });

    this.el.btnExportCsv.addEventListener('click', () => this.exportTestCsv());

    // Terminal Clear & Filters
    this.el.btnClearTerm.addEventListener('click', () => { this.el.termOutput.innerHTML = ""; });
    document.querySelectorAll('.tab-btn[data-filter]').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.tab-btn[data-filter]').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const f = btn.dataset.filter;
        document.querySelectorAll('.log-entry').forEach(entry => {
          if (f === 'all') entry.style.display = 'block';
          else entry.style.display = entry.classList.contains(f) ? 'block' : 'none';
        });
      });
    });
  }

  // --------------------------------------------------------------------------
  // Speed & Coordinate Conversion Helpers
  // --------------------------------------------------------------------------
  setJogSpeed(speedMmS) {
    this.CURRENT_SPEED_MM_S = Math.max(0.05, speedMmS);
    const rpm = (this.CURRENT_SPEED_MM_S * 60).toFixed(1);
    this.el.dispCurrentSpeed.textContent = `${this.CURRENT_SPEED_MM_S} mm/s (${rpm} RPM)`;
    this.el.dispContSpeed.textContent = `@ ${this.CURRENT_SPEED_MM_S} mm/s`;
    this.el.inputCustomSpeed.value = this.CURRENT_SPEED_MM_S;
  }

  // Velocity units in Protocol 2.0: 0.229 RPM / unit
  speedMmSToVelocityUnits(speedMmS) {
    const rpm = speedMmS * 60.0;
    return Math.max(1, Math.round(rpm / 0.229));
  }

  linearMmToTicks(mm) {
    return Math.round(mm * this.TICKS_PER_MM);
  }

  ticksToLinearMm(ticks) {
    return ticks / this.TICKS_PER_MM;
  }

  // --------------------------------------------------------------------------
  // PORT 1: U2D2 Dynamixel Serial Driver
  // --------------------------------------------------------------------------
  async toggleMotorConnection() {
    if (this.isMotorConnected) {
      await this.disconnectMotor();
    } else {
      await this.connectMotor();
    }
  }

  async connectMotor() {
    if (!navigator.serial) {
      alert("Web Serial API is not supported in this browser. Please use Google Chrome or Edge.");
      return;
    }

    try {
      this.motorPort = await navigator.serial.requestPort();
      const baudRate = parseInt(this.el.motorBaudSelect.value, 10);
      await this.motorPort.open({ baudRate, bufferSize: 4096 });

      this.isMotorConnected = true;
      this.motorKeepReading = true;
      this.el.cardMotorPort.classList.add('connected');
      this.el.btnConnectMotor.textContent = "DISCONNECT";
      this.el.btnConnectMotor.classList.replace('btn-primary-sm', 'btn-danger-sm');

      // Start non-blocking reader loop
      this.startMotorReaderLoop();

      this.log("MOTOR", `Connected to U2D2 at ${baudRate} baud.`, "motor");

      // Synchronize actuator state
      await this.syncMotorHardware(true);

      // Start telemetry polling at 20 Hz
      this.startMotorTelemetryPolling();
    } catch (err) {
      console.error("Motor connect failed:", err);
      this.log("MOTOR", `Connection failed: ${err.message}`, "err");
      this.isMotorConnected = false;
      this.el.cardMotorPort.classList.remove('connected');
    }
  }

  async disconnectMotor() {
    this.stopMotorTelemetryPolling();
    this.motorKeepReading = false;

    if (this.motorReader) {
      try { await this.motorReader.cancel(); } catch (e) {}
    }

    if (this.motorPort) {
      try { await this.motorPort.close(); } catch (e) {}
      this.motorPort = null;
    }

    this.isMotorConnected = false;
    this.el.cardMotorPort.classList.remove('connected');
    this.el.btnConnectMotor.textContent = "CONNECT U2D2";
    this.el.btnConnectMotor.classList.replace('btn-danger-sm', 'btn-primary-sm');
    this.log("MOTOR", "U2D2 Port closed.", "motor");
  }

  async startMotorReaderLoop() {
    while (this.motorPort && this.motorPort.readable && this.motorKeepReading) {
      try {
        this.motorReader = this.motorPort.readable.getReader();
        while (this.motorKeepReading) {
          const { value, done } = await this.motorReader.read();
          if (done) break;
          if (value && value.length > 0) {
            this.handleMotorIncomingBytes(value);
          }
        }
      } catch (err) {
        if (this.motorKeepReading) {
          console.warn("Motor framing / read error:", err);
          this.log("MOTOR", `Framing/Serial stream error. Re-establishing reader...`, "err");
          this.motorRxBuffer = [];
          if (this.motorActiveResolver) {
            const res = this.motorActiveResolver;
            this.motorActiveResolver = null;
            res.resolve(null);
          }
          await new Promise(r => setTimeout(r, 40));
        }
      } finally {
        if (this.motorReader) {
          try { this.motorReader.releaseLock(); } catch (e) {}
          this.motorReader = null;
        }
      }
    }
  }

  handleMotorIncomingBytes(bytes) {
    for (let i = 0; i < bytes.length; i++) {
      this.motorRxBuffer.push(bytes[i]);
    }

    // Safety guard to prevent memory bloat on continuous noise
    if (this.motorRxBuffer.length > 512) {
      this.motorRxBuffer = this.motorRxBuffer.slice(-256);
    }

    this.parseMotorProtocol2Stream();
  }

  parseMotorProtocol2Stream() {
    // Protocol 2.0 Header: 0xFF, 0xFF, 0xFD, 0x00
    while (this.motorRxBuffer.length >= 11) {
      if (
        this.motorRxBuffer[0] !== 0xff ||
        this.motorRxBuffer[1] !== 0xff ||
        this.motorRxBuffer[2] !== 0xfd ||
        this.motorRxBuffer[3] !== 0x00
      ) {
        this.motorRxBuffer.shift();
        continue;
      }

      const id = this.motorRxBuffer[4];
      const length = this.motorRxBuffer[5] | (this.motorRxBuffer[6] << 8);
      const totalLen = 7 + length;

      // Sanity length check against framing error corruption
      if (length < 4 || length > 256) {
        this.motorRxBuffer.shift();
        continue;
      }

      if (this.motorRxBuffer.length < totalLen) {
        // Look ahead for another header inside buffer before totalLen
        let nextHeaderIdx = -1;
        for (let i = 1; i <= this.motorRxBuffer.length - 4; i++) {
          if (
            this.motorRxBuffer[i] === 0xff &&
            this.motorRxBuffer[i + 1] === 0xff &&
            this.motorRxBuffer[i + 2] === 0xfd &&
            this.motorRxBuffer[i + 3] === 0x00
          ) {
            nextHeaderIdx = i;
            break;
          }
        }
        if (nextHeaderIdx !== -1 && nextHeaderIdx < totalLen) {
          this.motorRxBuffer.splice(0, nextHeaderIdx);
          continue;
        }
        return; // Await full packet
      }

      // Verify CRC before consuming
      const packetBytes = this.motorRxBuffer.slice(0, totalLen);
      const crcExpected = packetBytes[totalLen - 2] | (packetBytes[totalLen - 1] << 8);
      const crcActual = DXL.calculateCRC2(packetBytes, totalLen - 2);

      if (crcActual !== crcExpected) {
        this.motorRxBuffer.shift();
        continue;
      }

      this.motorRxBuffer.splice(0, totalLen);

      const instruction = packetBytes[7];
      const errorByte = packetBytes[8];
      const params = packetBytes.slice(9, totalLen - 2);

      this.processMotorStatusPacket(id, instruction, errorByte, params);
    }
  }

  processMotorStatusPacket(id, instruction, errorByte, params) {
    // Check explicit memory query resolver
    if (this.motorActiveResolver && id === this.motorActiveId) {
      if (params.length === this.motorActiveResolver.length) {
        const res = this.motorActiveResolver;
        this.motorActiveResolver = null;
        res.resolve({ addr: res.addr, error: errorByte, params });
        return;
      }
    }

    // Telemetry updates from active actuator
    if (id === this.motorActiveId) {
      if (params.length >= 8) {
        // Addr 128..131: Present Velocity
        this.motorPresentVelocity = DXL.bytesToInt(params.slice(0, 4), true);
        // Addr 132..135: Present Position (ticks)
        this.motorPresentTicks = DXL.bytesToInt(params.slice(4, 8), true);
        this.updateMotionDisplay();
      }
    }
  }

  async sendMotorPacket(packetBytes, desc = "") {
    if (!this.motorPort || !this.motorPort.writable || !this.isMotorConnected) return false;
    try {
      this.motorWriter = this.motorPort.writable.getWriter();
      await this.motorWriter.write(new Uint8Array(packetBytes));
      this.motorWriter.releaseLock();
      this.motorWriter = null;
      return true;
    } catch (e) {
      if (this.motorWriter) {
        try { this.motorWriter.releaseLock(); } catch (_) {}
        this.motorWriter = null;
      }
      return false;
    }
  }

  async readMotorMemory(addr, length) {
    if (!this.isMotorConnected) return null;
    const packet = DXL.buildPacketP2(this.motorActiveId, DXL.INST.READ, [
      addr & 0xff, (addr >> 8) & 0xff,
      length & 0xff, (length >> 8) & 0xff
    ]);

    return new Promise(async resolve => {
      this.motorActiveResolver = { addr, length, resolve };
      setTimeout(() => {
        if (this.motorActiveResolver && this.motorActiveResolver.resolve === resolve) {
          this.motorActiveResolver = null;
          resolve(null);
        }
      }, 250);

      await this.sendMotorPacket(packet, `READ ADDR ${addr}`);
    });
  }

  async syncMotorHardware(notify = false) {
    if (!this.isMotorConnected) return;
    try {
      // 1. Operating Mode (Addr 11)
      const mBlock = await this.readMotorMemory(11, 1);
      if (mBlock && mBlock.params && mBlock.params.length > 0) {
        this.motorOperatingMode = mBlock.params[0];
      }

      await new Promise(r => setTimeout(r, 20));

      // 2. Torque Enable (Addr 64)
      const tBlock = await this.readMotorMemory(64, 1);
      if (tBlock && tBlock.params && tBlock.params.length > 0) {
        const armed = tBlock.params[0] === 1;
        this.updateTorqueUI(armed);
      }

      if (notify) {
        this.log("MOTOR", `Actuator ID ${this.motorActiveId} Synced: Mode ${this.motorOperatingMode}, Torque=${this.motorTorqueEnabled ? "ON" : "OFF"}`, "motor");
      }
    } catch (e) {
      console.warn("syncMotorHardware error:", e);
    }
  }

  updateTorqueUI(armed) {
    this.motorTorqueEnabled = armed;
    this.el.badgeTorqueState.textContent = armed ? "TORQUE: ARMED" : "TORQUE: OFF";
    this.el.badgeTorqueState.className = armed ? "badge-emerald" : "val-pill";
    this.el.labelToggleTorque.textContent = armed ? "DISARM TORQUE" : "ARM TORQUE";
  }

  async setMotorTorque(enable) {
    this.updateTorqueUI(enable);
    const packet = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [
      64, 0, enable ? 1 : 0
    ]);
    await this.sendMotorPacket(packet, `TORQUE = ${enable ? 1 : 0}`);
    this.log("MOTOR", `Torque ${enable ? "ARMED" : "DISABLED"} on ID ${this.motorActiveId}`, "motor");
  }

  async toggleTorque() {
    if (!this.isMotorConnected) return;
    await this.setMotorTorque(!this.motorTorqueEnabled);
  }

  // --------------------------------------------------------------------------
  // Motion Control on Lead Screw (1 rev = 1 mm)
  // --------------------------------------------------------------------------
  startMotorTelemetryPolling() {
    this.stopMotorTelemetryPolling();
    // Poll at 20 Hz (50 ms)
    this.motorPollTimer = setInterval(() => {
      if (this.isMotorConnected && !this.motorIsPollingBusy && !this.motorActiveResolver) {
        this.queryMotorTelemetry();
      }
    }, 50);
  }

  stopMotorTelemetryPolling() {
    if (this.motorPollTimer) {
      clearInterval(this.motorPollTimer);
      this.motorPollTimer = null;
    }
  }

  async queryMotorTelemetry() {
    this.motorIsPollingBusy = true;
    try {
      // Read 8 bytes starting from Present Velocity (128):
      // 128..131: Present Velocity (4 bytes)
      // 132..135: Present Position (4 bytes)
      const packet = DXL.buildPacketP2(this.motorActiveId, DXL.INST.READ, [128, 0, 8, 0]);
      await this.sendMotorPacket(packet);
    } finally {
      this.motorIsPollingBusy = false;
    }
  }

  updateMotionDisplay() {
    const mm = this.ticksToLinearMm(this.motorPresentTicks);
    const revs = this.motorPresentTicks / this.TICKS_PER_REV;

    this.el.dispLinearMm.textContent = `${mm >= 0 ? '+' : ''}${mm.toFixed(2)} mm`;
    this.el.dispRevs.textContent = `${revs >= 0 ? '+' : ''}${revs.toFixed(2)} Revs`;
    this.el.dispTicks.textContent = `${this.motorPresentTicks.toLocaleString()} ticks`;
  }

  async ensureExtendedPositionMode() {
    if (this.motorOperatingMode !== 4) {
      this.log("MOTOR", "Actuator not in Extended Position Mode (Mode 4). Switching...", "motor");
      // Disable torque before mode switch
      await this.setMotorTorque(false);
      await new Promise(r => setTimeout(r, 40));
      // Write Mode 4 (Extended Position)
      const modePkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [11, 0, 4]);
      await this.sendMotorPacket(modePkt, "SET MODE 4");
      await new Promise(r => setTimeout(r, 40));
      this.motorOperatingMode = 4;
      await this.setMotorTorque(true);
      await new Promise(r => setTimeout(r, 40));
    } else if (!this.motorTorqueEnabled) {
      await this.setMotorTorque(true);
      await new Promise(r => setTimeout(r, 30));
    }
  }

  /**
   * Step Jog by relative distance in mm
   */
  async jogRelativeMm(deltaMm) {
    if (!this.isMotorConnected) {
      alert("Please connect U2D2 motor port first.");
      return;
    }

    await this.ensureExtendedPositionMode();

    // Set Profile Velocity according to selected mm/s jog speed
    const velUnits = this.speedMmSToVelocityUnits(this.CURRENT_SPEED_MM_S);
    const velBytes = DXL.intToBytes(velUnits, 4);
    const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
    await this.sendMotorPacket(velPkt, `PROFILE VEL = ${velUnits}`);

    // Calculate incremental target position from current motor position
    const currentMm = this.ticksToLinearMm(this.motorPresentTicks);
    const targetMm = currentMm + deltaMm;
    const targetTicks = this.linearMmToTicks(targetMm);

    const posBytes = DXL.intToBytes(targetTicks, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, `JOG TO ${targetMm.toFixed(2)} mm`);

    this.log("MOTOR", `Jog ${deltaMm >= 0 ? '+' : ''}${deltaMm} mm (Target: ${targetMm.toFixed(2)} mm @ ${this.CURRENT_SPEED_MM_S} mm/s)`, "motor");
  }

  /**
   * Continuous Move (Fast Forward or Rewind at fixed mm/s)
   */
  async startContinuousMove(direction) {
    if (!this.isMotorConnected) {
      alert("Please connect U2D2 motor port first.");
      return;
    }

    await this.ensureExtendedPositionMode();

    const speedMmS = this.CURRENT_SPEED_MM_S;
    const velUnits = this.speedMmSToVelocityUnits(speedMmS);
    const velBytes = DXL.intToBytes(velUnits, 4);
    const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
    await this.sendMotorPacket(velPkt, `PROFILE VEL = ${velUnits}`);

    // Drive towards max multi-turn envelope (+/- 200 mm continuous)
    const farTargetMm = direction > 0 ? 250.0 : -250.0;
    const targetTicks = this.linearMmToTicks(farTargetMm);
    const posBytes = DXL.intToBytes(targetTicks, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, `CONTINUOUS MOVE ${direction > 0 ? "FWD" : "REV"}`);

    this.motorIsContinuousMoving = true;
    this.log("MOTOR", `Continuous Move ${direction > 0 ? "FORWARD" : "REVERSE"} started @ ${speedMmS} mm/s`, "motor");
  }

  async haltMotion() {
    if (!this.isMotorConnected) return;

    // Immediately command current present position to stop in place
    const stopTicks = this.motorPresentTicks;
    const posBytes = DXL.intToBytes(stopTicks, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, "HALT MOTION");

    this.motorIsContinuousMoving = false;
    this.log("MOTOR", `Motion HALTED at ${this.ticksToLinearMm(stopTicks).toFixed(2)} mm`, "motor");
  }

  async gotoOriginMm() {
    if (!this.isMotorConnected) return;
    await this.ensureExtendedPositionMode();

    const velUnits = this.speedMmSToVelocityUnits(this.CURRENT_SPEED_MM_S);
    const velBytes = DXL.intToBytes(velUnits, 4);
    const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
    await this.sendMotorPacket(velPkt, `PROFILE VEL = ${velUnits}`);

    const posBytes = DXL.intToBytes(0, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, "GOTO 0.0 MM");

    this.log("MOTOR", `Driving carriage to 0.0 mm origin @ ${this.CURRENT_SPEED_MM_S} mm/s`, "motor");
  }

  /**
   * Reset Multi-turn Encoder to 0 at Current Location
   * Executes official Robotis Protocol 2.0 Clear (0x10) instruction
   */
  async resetMultiTurnEncoder() {
    if (!this.isMotorConnected) {
      alert("Please connect U2D2 motor port first.");
      return;
    }

    this.log("MOTOR", `Executing Hardware Multi-Turn Clear (0x10) on Actuator ID ${this.motorActiveId}...`, "motor");

    try {
      const prevTorque = this.motorTorqueEnabled;

      // 1. Robotis Rule: Torque MUST be disabled to execute Clear instruction
      if (prevTorque) {
        await this.setMotorTorque(false);
        await new Promise(r => setTimeout(r, 40));
      }

      // 2. Clear Instruction: 0x10 with params [0x01, 0x44, 0x58, 0x4C, 0x22]
      const clearParams = [0x01, 0x44, 0x58, 0x4C, 0x22];
      const clearPacket = DXL.buildPacketP2(this.motorActiveId, DXL.INST.CLEAR, clearParams);
      await this.sendMotorPacket(clearPacket, "CLEAR MULTI-TURN REVOLUTIONS");

      await new Promise(r => setTimeout(r, 60));

      // 3. Re-arm torque if it was armed
      if (prevTorque) {
        await this.setMotorTorque(true);
        await new Promise(r => setTimeout(r, 40));
      }

      // Query fresh position
      await this.queryMotorTelemetry();

      this.log("MOTOR", `✓ Multi-turn encoder successfully zeroed! Current position is now 0.00 mm origin.`, "motor");
    } catch (e) {
      console.error("Encoder reset error:", e);
      this.log("MOTOR", `Failed to reset encoder: ${e.message}`, "err");
    }
  }

  // --------------------------------------------------------------------------
  // PORT 2: Arduino Heated Force-Bench Serial Driver
  // --------------------------------------------------------------------------
  async toggleArduinoConnection() {
    if (this.isArduinoConnected) {
      await this.disconnectArduino();
    } else {
      await this.connectArduino();
    }
  }

  async connectArduino() {
    if (!navigator.serial) {
      alert("Web Serial API is not supported in this browser.");
      return;
    }

    try {
      this.arduinoPort = await navigator.serial.requestPort();
      const baudRate = parseInt(this.el.arduinoBaudSelect.value, 10);
      await this.arduinoPort.open({ baudRate, bufferSize: 4096 });

      this.isArduinoConnected = true;
      this.arduinoKeepReading = true;
      this.el.cardArduinoPort.classList.add('connected');
      this.el.btnConnectArduino.textContent = "DISCONNECT";
      this.el.btnConnectArduino.classList.replace('btn-primary-sm', 'btn-danger-sm');

      // Start stream reading loop
      this.startArduinoReaderLoop();

      this.log("BENCH", `Connected to Arduino at ${baudRate} baud.`, "bench");

      // Send REPORT ON to enable structured 10Hz telemetry
      await new Promise(r => setTimeout(r, 300));
      await this.sendArduinoCommand("REPORT ON");
    } catch (err) {
      console.error("Arduino connect failed:", err);
      this.log("BENCH", `Connection failed: ${err.message}`, "err");
      this.isArduinoConnected = false;
      this.el.cardArduinoPort.classList.remove('connected');
    }
  }

  async disconnectArduino() {
    this.arduinoKeepReading = false;

    if (this.arduinoReader) {
      try { await this.arduinoReader.cancel(); } catch (e) {}
    }

    if (this.arduinoPort) {
      try { await this.arduinoPort.close(); } catch (e) {}
      this.arduinoPort = null;
    }

    this.isArduinoConnected = false;
    this.el.cardArduinoPort.classList.remove('connected');
    this.el.btnConnectArduino.textContent = "CONNECT ARDUINO";
    this.el.btnConnectArduino.classList.replace('btn-danger-sm', 'btn-primary-sm');
    this.log("BENCH", "Arduino Bench port closed.", "bench");
  }

  async startArduinoReaderLoop() {
    const textDecoder = new TextDecoder();
    while (this.arduinoPort && this.arduinoPort.readable && this.arduinoKeepReading) {
      try {
        this.arduinoReader = this.arduinoPort.readable.getReader();
        while (this.arduinoKeepReading) {
          const { value, done } = await this.arduinoReader.read();
          if (done) break;
          if (value) {
            const chunk = textDecoder.decode(value, { stream: true });
            this.handleArduinoIncomingChunk(chunk);
          }
        }
      } catch (err) {
        if (this.arduinoKeepReading) {
          console.warn("Arduino framing / read error:", err);
          this.log("BENCH", `Arduino read error. Reconnecting stream...`, "err");
          this.arduinoBuffer = "";
          await new Promise(r => setTimeout(r, 40));
        }
      } finally {
        if (this.arduinoReader) {
          try { this.arduinoReader.releaseLock(); } catch (e) {}
          this.arduinoReader = null;
        }
      }
    }
  }

  handleArduinoIncomingChunk(chunk) {
    this.arduinoBuffer += chunk;
    const lines = this.arduinoBuffer.split('\n');
    this.arduinoBuffer = lines.pop(); // Keep partial line

    lines.forEach(rawLine => {
      const line = rawLine.trim();
      if (!line) return;

      // Check for structured telemetry: $DAT:Temp=25.40,Weight=0.125,PWM=0,SP=50.00
      if (line.startsWith("$DAT:")) {
        this.parseArduinoTelemetry(line.substring(5));
      } else {
        // Diagnostic or log message
        this.log("BENCH", line, "bench");
      }
    });
  }

  parseArduinoTelemetry(dataStr) {
    // Key-value pairs separated by commas
    const pairs = dataStr.split(',');
    pairs.forEach(pair => {
      const parts = pair.split('=');
      if (parts.length === 2) {
        const key = parts[0].trim().toUpperCase();
        const val = parseFloat(parts[1].trim());

        if (key === "TEMP") {
          this.liveTempC = val;
          if (val > this.peakTemp) this.peakTemp = val;
          this.el.dispChamberTemp.textContent = `${val.toFixed(2)} °C`;
          this.el.statLiveTemp.textContent = `TEMP: ${val.toFixed(2)} °C`;
          this.el.statPeakTemp.textContent = `PEAK: ${this.peakTemp.toFixed(1)} °C`;
        } else if (key === "WEIGHT") {
          this.liveWeightKg = val;
          this.liveForceN = val * 9.80665; // Convert kg to Newtons
          if (Math.abs(this.liveForceN) > Math.abs(this.peakForceN)) {
            this.peakForceN = this.liveForceN;
          }
          this.updateForceStats();
        } else if (key === "PWM") {
          this.livePwm = Math.round(val);
          this.el.dispChamberPwm.textContent = `${this.livePwm} / 255`;
          this.el.dispManualPwmVal.textContent = `PWM: ${this.livePwm}`;
          this.el.dispHeaterState.textContent = this.livePwm > 0 ? `HEATING (${this.livePwm})` : "HEATER: OFF";
          this.el.dispHeaterState.className = this.livePwm > 0 ? "badge-emerald" : "val-pill";
        } else if (key === "SP") {
          this.liveSetpoint = val;
          this.el.statLiveSp.textContent = `SP: ${val.toFixed(1)} °C`;
        }
      }
    });

    // Record into real-time plot buffer
    const nowSec = (performance.now() - this.startTime) / 1000.0;
    if (this.liveTempC !== null) {
      this.tempHistory.push({ t: nowSec, temp: this.liveTempC, sp: this.liveSetpoint });
    }
    this.forceHistory.push({ t: nowSec, forceN: this.liveForceN, weightKg: this.liveWeightKg });

    // Prune history older than 30s
    const cutoff = nowSec - this.historyDurationSec;
    while (this.tempHistory.length > 0 && this.tempHistory[0].t < cutoff) {
      this.tempHistory.shift();
    }
    while (this.forceHistory.length > 0 && this.forceHistory[0].t < cutoff) {
      this.forceHistory.shift();
    }

    // Safety monitor during Push/Pull automated test
    if (this.isTestRunning) {
      this.sampleMechanicalTestDataPoint(nowSec);
    }
  }

  updateForceStats() {
    const forceStr = this.isForceUnitNewton
      ? `${this.liveForceN.toFixed(3)} N`
      : `${this.liveWeightKg.toFixed(3)} kg`;
    const peakStr = this.isForceUnitNewton
      ? `${this.peakForceN.toFixed(3)} N`
      : `${(this.peakForceN / 9.80665).toFixed(3)} kg`;

    this.el.statLiveForce.textContent = `FORCE: ${forceStr}`;
    this.el.statPeakForce.textContent = `PEAK: ${peakStr}`;
    this.el.dispLoadcellWeight.textContent = `${this.liveWeightKg.toFixed(3)} kg (${this.liveForceN.toFixed(2)} N)`;
  }

  async sendArduinoCommand(cmdStr) {
    if (!this.arduinoPort || !this.arduinoPort.writable || !this.isArduinoConnected) return false;
    try {
      this.arduinoWriter = this.arduinoPort.writable.getWriter();
      const textEncoder = new TextEncoder();
      await this.arduinoWriter.write(textEncoder.encode(cmdStr + "\n"));
      this.arduinoWriter.releaseLock();
      this.arduinoWriter = null;
      this.log("BENCH", `TX &gt; ${cmdStr}`, "bench");
      return true;
    } catch (e) {
      if (this.arduinoWriter) {
        try { this.arduinoWriter.releaseLock(); } catch (_) {}
        this.arduinoWriter = null;
      }
      return false;
    }
  }

  async setHeaterSetpoint(sp) {
    this.liveSetpoint = sp;
    await this.sendArduinoCommand(`SP ${sp.toFixed(1)}`);
  }

  async enableHeater() {
    await this.sendArduinoCommand("START");
    this.heaterActive = true;
    this.log("BENCH", "PI Closed-loop Heater STARTED", "bench");
  }

  async stopHeater() {
    await this.sendArduinoCommand("STOP");
    this.heaterActive = false;
    this.log("BENCH", "Heater STOPPED", "bench");
  }

  async tareLoadcell() {
    this.log("BENCH", "Taring HX711 Loadcell...", "bench");
    await this.sendArduinoCommand("TARE");
  }

  // --------------------------------------------------------------------------
  // Global Emergency Stop (E-STOP)
  // --------------------------------------------------------------------------
  async triggerEmergencyStop() {
    this.log("SYS", "🛑 EMERGENCY STOP TRIGGERED! Cutting motor torque and heater power...", "err");

    // 1. Abort any running automated test
    if (this.isTestRunning) {
      this.abortTestSequence();
    }

    // 2. Disarm motor immediately
    if (this.isMotorConnected) {
      await this.setMotorTorque(false);
      this.motorIsContinuousMoving = false;
    }

    // 3. Shut off heater immediately
    if (this.isArduinoConnected) {
      await this.sendArduinoCommand("STOP");
      await this.sendArduinoCommand("PWM 0");
    }

    alert("EMERGENCY STOP EXECUTED:\n• Actuator Torque Disarmed\n• Heater Turned OFF (PWM=0)\n• Any Running Tests Aborted");
  }

  // --------------------------------------------------------------------------
  // Automated Push / Pull Mechanical Test Runner
  // --------------------------------------------------------------------------
  async startTestSequence() {
    if (!this.isMotorConnected) {
      alert("Please connect U2D2 motor port first.");
      return;
    }
    if (!this.isArduinoConnected) {
      alert("Please connect Arduino Bench port to record force data.");
      return;
    }

    this.testDirection = this.el.selectTestDirection.value; // "push" or "pull"
    this.testTravelMm = parseFloat(this.el.inputTestDistance.value) || 10.0;
    this.testSpeedMmS = parseFloat(this.el.inputTestSpeed.value) || 1.0;
    this.testMaxForceLimitN = parseFloat(this.el.inputMaxForceLimit.value) || 50.0;
    const autoTare = this.el.checkAutoTare.checked;

    this.isTestRunning = true;
    this.testDataPoints = [];
    this.testPeakForceN = 0.0;
    this.testDispAtPeakMm = 0.0;
    this.el.dispTestPhase.textContent = "RUNNING";
    this.el.badgeTestState.textContent = "TEST: RUNNING";
    this.el.badgeTestState.className = "badge-emerald";
    this.el.btnExecuteTest.classList.add('abort-mode');
    this.el.labelExecuteTest.textContent = "ABORT TEST RUN";
    this.el.btnExportCsv.disabled = true;

    this.log("TEST", `Starting automated ${this.testDirection.toUpperCase()} test (${this.testTravelMm} mm @ ${this.testSpeedMmS} mm/s, Limit: ${this.testMaxForceLimitN} N)...`, "test");

    // 1. Optional Auto-Tare
    if (autoTare) {
      this.log("TEST", "Auto-taring loadcell before motion begins...", "test");
      await this.tareLoadcell();
      await new Promise(r => setTimeout(r, 600));
    }

    // 2. Record initial linear position
    this.testStartMm = this.ticksToLinearMm(this.motorPresentTicks);

    // 3. Ensure Mode 4 and set test velocity
    await this.ensureExtendedPositionMode();
    const velUnits = this.speedMmSToVelocityUnits(this.testSpeedMmS);
    const velBytes = DXL.intToBytes(velUnits, 4);
    const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
    await this.sendMotorPacket(velPkt);

    // 4. Command target displacement
    const sign = (this.testDirection === "push") ? 1 : -1;
    const targetMm = this.testStartMm + (sign * this.testTravelMm);
    const targetTicks = this.linearMmToTicks(targetMm);

    const posBytes = DXL.intToBytes(targetTicks, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, `TEST MOVE TO ${targetMm.toFixed(2)} mm`);

    // 5. Monitor motion completion
    const checkInterval = setInterval(async () => {
      if (!this.isTestRunning) {
        clearInterval(checkInterval);
        return;
      }

      const currMm = this.ticksToLinearMm(this.motorPresentTicks);
      const dispTravel = Math.abs(currMm - this.testStartMm);

      // Check distance arrival
      if (dispTravel >= this.testTravelMm - 0.05) {
        clearInterval(checkInterval);
        await this.finishTestSequence(false);
      }
    }, 50);
  }

  sampleMechanicalTestDataPoint(nowSec) {
    const currMm = this.ticksToLinearMm(this.motorPresentTicks);
    const dispFromStart = Math.abs(currMm - this.testStartMm);
    const forceAbsN = Math.abs(this.liveForceN);

    this.testDataPoints.push({
      t: nowSec,
      dispMm: dispFromStart,
      forceN: this.liveForceN,
      forceAbsN: forceAbsN,
      tempC: this.liveTempC || 0
    });

    // Update test peak
    if (forceAbsN > Math.abs(this.testPeakForceN)) {
      this.testPeakForceN = this.liveForceN;
      this.testDispAtPeakMm = dispFromStart;
    }

    // Safety force limit check
    if (forceAbsN >= this.testMaxForceLimitN) {
      this.log("TEST", `⚠️ SAFETY FORCE LIMIT REACHED (${forceAbsN.toFixed(2)} N >= ${this.testMaxForceLimitN} N)! Auto-stopping...`, "err");
      this.haltMotion();
      this.finishTestSequence(true);
    }
  }

  async finishTestSequence(safetyStopped = false) {
    this.isTestRunning = false;
    this.el.dispTestPhase.textContent = safetyStopped ? "LIMIT HALT" : "COMPLETED";
    this.el.badgeTestState.textContent = safetyStopped ? "TEST: LIMIT HALT" : "TEST: COMPLETED";
    this.el.badgeTestState.className = safetyStopped ? "badge-purple" : "badge-emerald";
    this.el.btnExecuteTest.classList.remove('abort-mode');
    this.el.labelExecuteTest.textContent = "START PUSH / PULL TEST SEQUENCE";

    // Calculate metrics
    this.computeTestCharacterization();

    // Auto-return if requested
    if (this.el.checkAutoReturn.checked) {
      this.log("TEST", "Auto-returning carriage back to start position...", "test");
      await new Promise(r => setTimeout(r, 200));
      const startTicks = this.linearMmToTicks(this.testStartMm);
      const posBytes = DXL.intToBytes(startTicks, 4);
      const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
      await this.sendMotorPacket(posPkt, "RETURN TO START");
    }

    this.el.btnExportCsv.disabled = false;
    this.log("TEST", `✓ Test sequence finished. Recorded ${this.testDataPoints.length} mechanical data points.`, "test");
  }

  abortTestSequence() {
    this.isTestRunning = false;
    this.haltMotion();
    this.el.dispTestPhase.textContent = "ABORTED";
    this.el.badgeTestState.textContent = "TEST: ABORTED";
    this.el.btnExecuteTest.classList.remove('abort-mode');
    this.el.labelExecuteTest.textContent = "START PUSH / PULL TEST SEQUENCE";
    this.log("TEST", "Push/Pull test ABORTED by user.", "err");
    this.computeTestCharacterization();
    this.el.btnExportCsv.disabled = (this.testDataPoints.length === 0);
  }

  computeTestCharacterization() {
    if (this.testDataPoints.length < 2) return;

    // Peak Force
    let maxF = 0;
    let dispAtMax = 0;
    this.testDataPoints.forEach(p => {
      const f = Math.abs(p.forceN);
      if (f > maxF) {
        maxF = f;
        dispAtMax = p.dispMm;
      }
    });

    this.testPeakForceN = maxF;
    this.testDispAtPeakMm = dispAtMax;

    // Stiffness (Linear slope ΔF / ΔX from 20% to 80% of peak)
    const fLow = maxF * 0.2;
    const fHigh = maxF * 0.8;
    const pLow = this.testDataPoints.find(p => Math.abs(p.forceN) >= fLow);
    const pHigh = this.testDataPoints.find(p => Math.abs(p.forceN) >= fHigh);

    let stiffness = 0;
    if (pLow && pHigh && Math.abs(pHigh.dispMm - pLow.dispMm) > 0.05) {
      stiffness = (Math.abs(pHigh.forceN) - Math.abs(pLow.forceN)) / (pHigh.dispMm - pLow.dispMm);
    }

    this.testStiffnessNPerMm = stiffness;

    // Energy = trapezoidal integration ∫ F dx (mJ = N * mm)
    let energy = 0;
    for (let i = 1; i < this.testDataPoints.length; i++) {
      const dx = Math.abs(this.testDataPoints[i].dispMm - this.testDataPoints[i - 1].dispMm);
      const avgF = (Math.abs(this.testDataPoints[i].forceN) + Math.abs(this.testDataPoints[i - 1].forceN)) / 2.0;
      energy += avgF * dx;
    }
    this.testEnergyMJ = energy;

    // Update UI
    this.el.resPeakForce.textContent = `${this.testPeakForceN.toFixed(2)} N (${(this.testPeakForceN / 9.80665).toFixed(3)} kg)`;
    this.el.resPeakDisp.textContent = `${this.testDispAtPeakMm.toFixed(2)} mm`;
    this.el.resStiffness.textContent = stiffness > 0 ? `${stiffness.toFixed(2)} N/mm` : "-- N/mm";
    this.el.resEnergy.textContent = `${energy.toFixed(1)} mJ`;

    this.el.statTestPeak.textContent = `PEAK: ${this.testPeakForceN.toFixed(2)} N @ ${this.testDispAtPeakMm.toFixed(2)} mm`;
    this.el.statTestStiffness.textContent = `STIFFNESS: ${stiffness.toFixed(1)} N/mm`;
  }

  exportTestCsv() {
    if (this.testDataPoints.length === 0) {
      alert("No test data recorded to export.");
      return;
    }

    const t0 = this.testDataPoints[0].t;
    let csv = "Time_Sec,Displacement_mm,Force_N,Weight_kg,Temperature_degC\n";
    this.testDataPoints.forEach(p => {
      const tRel = (p.t - t0).toFixed(3);
      const d = p.dispMm.toFixed(3);
      const fn = p.forceN.toFixed(3);
      const w = (p.forceN / 9.80665).toFixed(3);
      const tc = p.tempC.toFixed(2);
      csv += `${tRel},${d},${fn},${w},${tc}\n`;
    });

    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `PushPullTest_${this.testDirection.toUpperCase()}_${Date.now()}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }

  // --------------------------------------------------------------------------
  // Canvas Oscilloscope Engine (Subplots 1, 2, and 3)
  // --------------------------------------------------------------------------
  initCanvasPlots() {
    this.resizeCanvases();
    window.addEventListener('resize', () => this.resizeCanvases());
  }

  resizeCanvases() {
    const dpr = window.devicePixelRatio || 1;
    [this.el.canvasTempPlot, this.el.canvasForcePlot, this.el.canvasPushPullPlot].forEach(c => {
      if (!c) return;
      const rect = c.getBoundingClientRect();
      c.width = rect.width * dpr;
      c.height = (c.getAttribute('height') || 180) * dpr;
    });
  }

  startPlotAnimationLoop() {
    const render = () => {
      this.renderSubplot1Temp();
      this.renderSubplot2Force();
      this.renderSubplot3PushPull();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  // Subplot 1: Temperature vs Time
  renderSubplot1Temp() {
    const c = this.el.canvasTempPlot;
    if (!c) return;
    const ctx = c.getContext('2d');
    const w = c.width;
    const h = c.height;
    const dpr = window.devicePixelRatio || 1;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#040711";
    ctx.fillRect(0, 0, w, h);

    const pad = { top: 15 * dpr, right: 15 * dpr, bottom: 20 * dpr, left: 55 * dpr };
    const pw = w - pad.left - pad.right;
    const ph = h - pad.top - pad.bottom;
    if (pw <= 0 || ph <= 0) return;

    // Y Range
    let minY = 20.0;
    let maxY = 70.0;
    this.tempHistory.forEach(d => {
      if (d.temp < minY) minY = d.temp - 2;
      if (d.temp > maxY) maxY = d.temp + 5;
      if (d.sp && d.sp > maxY) maxY = d.sp + 5;
    });
    const yRange = maxY - minY || 1;

    // Grid lines
    ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
    ctx.lineWidth = 1 * dpr;
    ctx.fillStyle = "#64748b";
    ctx.font = `${9 * dpr}px "JetBrains Mono", monospace`;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";

    const ySteps = 4;
    for (let i = 0; i <= ySteps; i++) {
      const yVal = minY + (i / ySteps) * yRange;
      const y = pad.top + ph - (i / ySteps) * ph;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(w - pad.right, y);
      ctx.stroke();
      ctx.fillText(`${yVal.toFixed(1)}°`, pad.left - 6 * dpr, y);
    }

    if (this.tempHistory.length < 2) return;

    const nowSec = (performance.now() - this.startTime) / 1000.0;
    const minT = nowSec - this.historyDurationSec;

    // Target Setpoint Dashed Line (Rose)
    if (this.liveSetpoint) {
      const spY = pad.top + ph - ((this.liveSetpoint - minY) / yRange) * ph;
      ctx.save();
      ctx.strokeStyle = "rgba(244, 63, 94, 0.7)";
      ctx.lineWidth = 1.5 * dpr;
      ctx.setLineDash([6 * dpr, 4 * dpr]);
      ctx.beginPath();
      ctx.moveTo(pad.left, spY);
      ctx.lineTo(w - pad.right, spY);
      ctx.stroke();
      ctx.restore();
    }

    // Actual Temperature Curve (Cyan)
    ctx.beginPath();
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2.2 * dpr;
    this.tempHistory.forEach((d, i) => {
      const x = pad.left + ((d.t - minT) / this.historyDurationSec) * pw;
      const y = pad.top + ph - ((d.temp - minY) / yRange) * ph;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  // Subplot 2: Force / Weight vs Time
  renderSubplot2Force() {
    const c = this.el.canvasForcePlot;
    if (!c) return;
    const ctx = c.getContext('2d');
    const w = c.width;
    const h = c.height;
    const dpr = window.devicePixelRatio || 1;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#040711";
    ctx.fillRect(0, 0, w, h);

    const pad = { top: 15 * dpr, right: 15 * dpr, bottom: 20 * dpr, left: 55 * dpr };
    const pw = w - pad.left - pad.right;
    const ph = h - pad.top - pad.bottom;
    if (pw <= 0 || ph <= 0) return;

    // Y Range (Symmetric or auto)
    let maxF = 5.0; // 5 N default
    this.forceHistory.forEach(d => {
      const val = this.isForceUnitNewton ? Math.abs(d.forceN) : Math.abs(d.weightKg);
      if (val > maxF) maxF = val * 1.15;
    });
    const minY = 0.0;
    const maxY = maxF;
    const yRange = maxY - minY || 1;

    // Grid lines
    ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
    ctx.lineWidth = 1 * dpr;
    ctx.fillStyle = "#64748b";
    ctx.font = `${9 * dpr}px "JetBrains Mono", monospace`;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";

    const ySteps = 4;
    for (let i = 0; i <= ySteps; i++) {
      const yVal = minY + (i / ySteps) * yRange;
      const y = pad.top + ph - (i / ySteps) * ph;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(w - pad.right, y);
      ctx.stroke();
      const unitLbl = this.isForceUnitNewton ? "N" : "kg";
      ctx.fillText(`${yVal.toFixed(2)} ${unitLbl}`, pad.left - 6 * dpr, y);
    }

    if (this.forceHistory.length < 2) return;

    const nowSec = (performance.now() - this.startTime) / 1000.0;
    const minT = nowSec - this.historyDurationSec;

    // Live Force Curve (Emerald Green)
    ctx.beginPath();
    ctx.strokeStyle = "#10b981";
    ctx.lineWidth = 2.2 * dpr;
    this.forceHistory.forEach((d, i) => {
      const val = this.isForceUnitNewton ? Math.abs(d.forceN) : Math.abs(d.weightKg);
      const x = pad.left + ((d.t - minT) / this.historyDurationSec) * pw;
      const y = pad.top + ph - ((val - minY) / yRange) * ph;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  // Subplot 3: Push / Pull Test (Force vs Displacement)
  renderSubplot3PushPull() {
    const c = this.el.canvasPushPullPlot;
    if (!c) return;
    const ctx = c.getContext('2d');
    const w = c.width;
    const h = c.height;
    const dpr = window.devicePixelRatio || 1;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#040711";
    ctx.fillRect(0, 0, w, h);

    const pad = { top: 20 * dpr, right: 25 * dpr, bottom: 30 * dpr, left: 60 * dpr };
    const pw = w - pad.left - pad.right;
    const ph = h - pad.top - pad.bottom;
    if (pw <= 0 || ph <= 0) return;

    // X Range: Displacement (0 to max test travel)
    const maxX = Math.max(5.0, this.testTravelMm);
    // Y Range: Force (0 to safety limit or peak)
    let maxY = Math.max(10.0, this.testMaxForceLimitN);
    this.testDataPoints.forEach(p => {
      if (p.forceAbsN > maxY) maxY = p.forceAbsN * 1.1;
    });

    // Grid & Axis
    ctx.strokeStyle = "rgba(255, 255, 255, 0.06)";
    ctx.lineWidth = 1 * dpr;
    ctx.fillStyle = "#64748b";
    ctx.font = `${9 * dpr}px "JetBrains Mono", monospace`;

    // Horizontal grid (Force)
    const ySteps = 4;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let i = 0; i <= ySteps; i++) {
      const fVal = (i / ySteps) * maxY;
      const y = pad.top + ph - (i / ySteps) * ph;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(w - pad.right, y);
      ctx.stroke();
      ctx.fillText(`${fVal.toFixed(1)} N`, pad.left - 6 * dpr, y);
    }

    // Vertical grid (Displacement)
    const xSteps = 5;
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let i = 0; i <= xSteps; i++) {
      const xVal = (i / xSteps) * maxX;
      const x = pad.left + (i / xSteps) * pw;
      ctx.beginPath();
      ctx.moveTo(x, pad.top);
      ctx.lineTo(x, pad.top + ph);
      ctx.stroke();
      ctx.fillText(`${xVal.toFixed(1)} mm`, x, pad.top + ph + 6 * dpr);
    }

    // Axis Labels
    ctx.font = `bold ${10 * dpr}px "Outfit", sans-serif`;
    ctx.fillStyle = "rgba(192, 132, 252, 0.85)";
    ctx.textAlign = "right";
    ctx.fillText("FORCE (N)", pad.left - 6 * dpr, pad.top - 8 * dpr);

    ctx.textAlign = "right";
    ctx.fillText("DISPLACEMENT (MM)", w - pad.right, pad.top + ph + 16 * dpr);

    // Render Mechanical Curve
    if (this.testDataPoints.length >= 2) {
      ctx.save();
      ctx.beginPath();
      ctx.strokeStyle = "#c084fc";
      ctx.lineWidth = 2.5 * dpr;
      ctx.lineJoin = "round";

      this.testDataPoints.forEach((p, i) => {
        const x = pad.left + (p.dispMm / maxX) * pw;
        const y = pad.top + ph - (p.forceAbsN / maxY) * ph;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      // Peak Point Reticle
      if (this.testPeakForceN > 0) {
        const peakX = pad.left + (this.testDispAtPeakMm / maxX) * pw;
        const peakY = pad.top + ph - (this.testPeakForceN / maxY) * ph;
        ctx.fillStyle = "#f43f5e";
        ctx.beginPath();
        ctx.arc(peakX, peakY, 5 * dpr, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 1.5 * dpr;
        ctx.stroke();
      }
      ctx.restore();
    } else {
      // Empty watermark
      ctx.fillStyle = "rgba(255, 255, 255, 0.1)";
      ctx.font = `600 ${12 * dpr}px "Outfit", sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("PUSH / PULL CURVE AWAITING TEST EXECUTION", pad.left + pw / 2, pad.top + ph / 2);
    }
  }

  // --------------------------------------------------------------------------
  // Diagnostics & Terminal Logger
  // --------------------------------------------------------------------------
  log(source, message, type = "all") {
    const entry = document.createElement("div");
    entry.className = `log-entry ${type}`;
    const timeStr = new Date().toLocaleTimeString();
    entry.textContent = `[${timeStr}] [${source}] ${message}`;
    this.el.termOutput.appendChild(entry);
    this.el.termOutput.scrollTop = this.el.termOutput.scrollHeight;
  }
}

// Instantiate on DOM load
window.addEventListener('DOMContentLoaded', () => {
  window.app = new UnifiedForceBenchApp();
});
