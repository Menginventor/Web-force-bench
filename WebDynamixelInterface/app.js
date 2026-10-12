/**
 * Web Dynamixel Interface - Application Logic
 * Native Web Serial API controller for Robotis Dynamixel Servos
 */

class WebDynamixelApp {
  constructor() {
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.readLoopPromise = null;
    this.keepReading = false;
    this.isConnected = false;

    // Active Servo & Bus State
    this.protocolVersion = "2.0"; // "2.0" | "1.0" | "ascii"
    this.activeId = 1;
    this.activeModel = "XL430-W250";
    this.discoveredServos = new Map(); // id -> { id, model, fw, pingTime }
    
    // Servo Controls State
    this.torqueEnabled = false;
    this.ledEnabled = false;
    this.goalPosition = 2048; // Default center for P2.0 (0-4095)
    this.goalVelocity = 200;
    this.torqueLimit = 800;

    // Telemetry State
    this.presentPosition = 2048;
    this.presentVelocity = 0;
    this.presentLoad = 0;
    this.presentVoltage = 12.0;
    this.presentTemp = 32;
    this.isMoving = false;
    this.hardwareError = 0;

    // Polling Timer
    this.pollIntervalMs = 100; // 10 Hz
    this.pollTimer = null;
    this.isPollingBusy = false;

    // Scope Plotter
    this.scopeCanvas = null;
    this.scopeCtx = null;
    this.scopeData = []; // Buffer of { t, singleTarget, singleActual, multiTargetTurns, multiActualTurns, speedTarget, speedActual, load }
    this.scopeMaxPoints = 200;
    this.isScopePaused = false;
    this.scopeAnimFrame = null;
    this.startTime = performance.now();
    this.scopePlotMode = "single"; // "single" | "multiturn" | "speed"
    this.goalVelocityRpm = 0;
    this.multiturnGoalTurns = 0;

    // Sweep / Sine Sequencer
    this.isSweeping = false;
    this.sweepTimer = null;
    this.sweepStartTime = 0;

    // Stream Buffer for Raw Serial Parsing
    this.rxBuffer = [];
    this.pendingResolvers = new Map(); // key -> callback

    // Rotary Dial Interaction
    this.isDialDragging = false;

    this.initElements();
    this.initRotaryTicks();
    this.initScopeCanvas();
    this.attachEventListeners();
    this.checkWebSerialSupport();
    this.refreshPortList();
  }

  // --------------------------------------------------------------------------
  // UI Initialization
  // --------------------------------------------------------------------------
  initElements() {
    this.el = {
      portSelect: document.getElementById('portSelect'),
      baudSelect: document.getElementById('baudSelect'),
      protocolSelect: document.getElementById('protocolSelect'),
      btnRequestPort: document.getElementById('btnRequestPort'),
      btnConnect: document.getElementById('btnConnect'),
      connStatusBadge: document.getElementById('connStatusBadge'),
      connStatusText: document.getElementById('connStatusText'),
      vidPidLabel: document.getElementById('vidPidLabel'),
      deviceInfoPill: document.getElementById('deviceInfoPill'),

      // Scanner
      scanStartId: document.getElementById('scanStartId'),
      scanEndId: document.getElementById('scanEndId'),
      btnScanBus: document.getElementById('btnScanBus'),
      btnPingBroadcast: document.getElementById('btnPingBroadcast'),
      scanProgressBar: document.getElementById('scanProgressBar'),
      discoveredCount: document.getElementById('discoveredCount'),
      discoveredServoList: document.getElementById('discoveredServoList'),
      btnAddManualId: document.getElementById('btnAddManualId'),

      // Active Servo
      activeIdDisplay: document.getElementById('activeIdDisplay'),
      activeModelDisplay: document.getElementById('activeModelDisplay'),
      activeProtocolBadge: document.getElementById('activeProtocolBadge'),
      activeModeBadge: document.getElementById('activeModeBadge'),
      operatingModeSelect: document.getElementById('operatingModeSelect'),
      btnSetOperatingMode: document.getElementById('btnSetOperatingMode'),
      torqueAlertBanner: document.getElementById('torqueAlertBanner'),
      btnQuickArmTorque: document.getElementById('btnQuickArmTorque'),

      // Dial & Horn
      rotarySvg: document.getElementById('rotarySvg'),
      svgArcActive: document.getElementById('svgArcActive'),
      svgPresentArm: document.getElementById('svgPresentArm'),
      svgGoalArm: document.getElementById('svgGoalArm'),
      dialAngleText: document.getElementById('dialAngleText'),
      dialTicksText: document.getElementById('dialTicksText'),

      // Switches & Sliders
      toggleTorque: document.getElementById('toggleTorque'),
      torqueStateLabel: document.getElementById('torqueStateLabel'),
      toggleLed: document.getElementById('toggleLed'),
      ledStateLabel: document.getElementById('ledStateLabel'),
      sliderPosition: document.getElementById('sliderPosition'),
      sliderPosReadout: document.getElementById('sliderPosReadout'),
      sliderVelocity: document.getElementById('sliderVelocity'),
      sliderVelReadout: document.getElementById('sliderVelReadout'),
      sliderTorqueLimit: document.getElementById('sliderTorqueLimit'),
      sliderTorqueLimitReadout: document.getElementById('sliderTorqueLimitReadout'),

      // HUD
      hudPosVal: document.getElementById('hudPosVal'),
      hudPosDeg: document.getElementById('hudPosDeg'),
      hudVelVal: document.getElementById('hudVelVal'),
      hudLoadVal: document.getElementById('hudLoadVal'),
      hudTempVal: document.getElementById('hudTempVal'),
      hudVoltVal: document.getElementById('hudVoltVal'),
      hudErrorVal: document.getElementById('hudErrorVal'),
      hudMovingVal: document.getElementById('hudMovingVal'),

      // Maintenance
      btnRebootServo: document.getElementById('btnRebootServo'),
      btnEmergencyStop: document.getElementById('btnEmergencyStop'),

      // Scope
      scopeCanvas: document.getElementById('scopeCanvas'),
      btnScopePause: document.getElementById('btnScopePause'),
      btnScopeClear: document.getElementById('btnScopeClear'),
      btnScopeExportCsv: document.getElementById('btnScopeExportCsv'),
      scopeWindowSelect: document.getElementById('scopeWindowSelect'),
      scopeTargetBadge: document.getElementById('scopeTargetBadge'),
      scopeActualBadge: document.getElementById('scopeActualBadge'),
      scopeErrorBadge: document.getElementById('scopeErrorBadge'),
      scopeModeWatermark: document.getElementById('scopeModeWatermark'),
      scopePointsWatermark: document.getElementById('scopePointsWatermark'),
      scopeLegendTarget: document.getElementById('scopeLegendTarget'),
      scopeLegendActual: document.getElementById('scopeLegendActual'),
      pollRateSelect: document.getElementById('pollRateSelect'),
      pollFpsLabel: document.getElementById('pollFpsLabel'),

      // Sequencer
      sweepMinAngle: document.getElementById('sweepMinAngle'),
      sweepMaxAngle: document.getElementById('sweepMaxAngle'),
      sweepFreq: document.getElementById('sweepFreq'),
      sweepFreqReadout: document.getElementById('sweepFreqReadout'),
      btnStartSweep: document.getElementById('btnStartSweep'),
      btnStopSweep: document.getElementById('btnStopSweep'),

      // Packet Inspector
      packetLogBox: document.getElementById('packetLogBox'),
      btnClearPackets: document.getElementById('btnClearPackets'),
      pktIdInput: document.getElementById('pktIdInput'),
      pktInstSelect: document.getElementById('pktInstSelect'),
      pktAddrInput: document.getElementById('pktAddrInput'),
      pktValInput: document.getElementById('pktValInput'),
      btnSendCustomPkt: document.getElementById('btnSendCustomPkt'),
      hexRawInput: document.getElementById('hexRawInput'),
      btnSendRawHex: document.getElementById('btnSendRawHex'),

      // Dynamic Mode Controls
      velocityControlBox: document.getElementById('velocityControlBox'),
      sliderLiveVelocity: document.getElementById('sliderLiveVelocity'),
      velLiveReadout: document.getElementById('velLiveReadout'),
      btnLiveStopVelocity: document.getElementById('btnLiveStopVelocity'),

      multiturnControlBox: document.getElementById('multiturnControlBox'),
      inputLiveTurns: document.getElementById('inputLiveTurns'),
      multiturnLiveReadout: document.getElementById('multiturnLiveReadout'),
      btnApplyLiveTurns: document.getElementById('btnApplyLiveTurns'),
      btnZeroLiveTurns: document.getElementById('btnZeroLiveTurns'),

      // Mode Tests Tab
      btnSwitchSpeedMode: document.getElementById('btnSwitchSpeedMode'),
      testSpeedSlider: document.getElementById('testSpeedSlider'),
      testSpeedValReadout: document.getElementById('testSpeedValReadout'),
      testSpeedActualReadout: document.getElementById('testSpeedActualReadout'),
      btnTestStopSpeed: document.getElementById('btnTestStopSpeed'),
      btnRampSpeedTest: document.getElementById('btnRampSpeedTest'),
      btnOscillateSpeedTest: document.getElementById('btnOscillateSpeedTest'),

      btnSwitchMultiturnMode: document.getElementById('btnSwitchMultiturnMode'),
      testTurnsInput: document.getElementById('testTurnsInput'),
      testTurnsTicksReadout: document.getElementById('testTurnsTicksReadout'),
      testMultiturnActualReadout: document.getElementById('testMultiturnActualReadout'),
      btnExecuteMultiturnMove: document.getElementById('btnExecuteMultiturnMove'),
      btnMultiturnSweepTest: document.getElementById('btnMultiturnSweepTest'),
      btnReturnToJointMode: document.getElementById('btnReturnToJointMode'),

      // Motor Parameters Tab
      btnInspectParamsDirect: document.getElementById('btnInspectParamsDirect'),
      btnReadAllParams: document.getElementById('btnReadAllParams'),
      btnExportParamsJson: document.getElementById('btnExportParamsJson'),
      paramReadStatus: document.getElementById('paramReadStatus'),
      paramModelName: document.getElementById('paramModelName'),
      paramModelNum: document.getElementById('paramModelNum'),
      paramSeries: document.getElementById('paramSeries'),
      paramBus: document.getElementById('paramBus'),
      paramFwVer: document.getElementById('paramFwVer'),
      paramTableBody: document.getElementById('paramTableBody'),
    };

    // Restore previously used connection settings
    this.restoreSavedSettings();
  }

  restoreSavedSettings() {
    try {
      const savedBaud = localStorage.getItem('dxl_baud_rate');
      if (savedBaud && this.el.baudSelect) {
        let exists = false;
        for (let i = 0; i < this.el.baudSelect.options.length; i++) {
          if (this.el.baudSelect.options[i].value === savedBaud) {
            exists = true;
            break;
          }
        }
        if (!exists) {
          const opt = document.createElement('option');
          opt.value = savedBaud;
          opt.textContent = `${Number(savedBaud).toLocaleString()} bps (Saved / Custom)`;
          this.el.baudSelect.appendChild(opt);
        }
        this.el.baudSelect.value = savedBaud;
      }

      const savedProto = localStorage.getItem('dxl_protocol_version');
      if (savedProto && this.el.protocolSelect) {
        this.el.protocolSelect.value = savedProto;
        this.protocolVersion = savedProto;
        if (this.el.activeProtocolBadge) {
          this.el.activeProtocolBadge.textContent = `PROTOCOL ${savedProto.toUpperCase()}`;
        }
      }
    } catch (e) {
      console.warn("Could not access localStorage:", e);
    }
  }

  saveCurrentSettings() {
    try {
      if (this.el.baudSelect) {
        localStorage.setItem('dxl_baud_rate', this.el.baudSelect.value);
      }
      if (this.el.protocolSelect) {
        localStorage.setItem('dxl_protocol_version', this.el.protocolSelect.value);
      }
    } catch (e) {
      console.warn("Could not save to localStorage:", e);
    }
  }

  initRotaryTicks() {
    const ticksGroup = document.getElementById('svgTicks');
    if (!ticksGroup) return;
    const cx = 130, cy = 130, r1 = 114, r2 = 120;
    for (let deg = 0; deg < 360; deg += 30) {
      const rad = (deg - 90) * Math.PI / 180;
      const x1 = cx + r1 * Math.cos(rad);
      const y1 = cy + r1 * Math.sin(rad);
      const x2 = cx + r2 * Math.cos(rad);
      const y2 = cy + r2 * Math.sin(rad);
      const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
      line.setAttribute("x1", x1);
      line.setAttribute("y1", y1);
      line.setAttribute("x2", x2);
      line.setAttribute("y2", y2);
      if (deg % 90 === 0) {
        line.setAttribute("stroke", "#38bdf8");
        line.setAttribute("stroke-width", "2.5");
      }
      ticksGroup.appendChild(line);
    }
  }

  // --------------------------------------------------------------------------
  // Web Serial API Connection
  // --------------------------------------------------------------------------
  checkWebSerialSupport() {
    if (!("serial" in navigator)) {
      alert("Web Serial API is not supported in this browser. Please use Google Chrome, Microsoft Edge, or Opera.");
      this.el.btnConnect.disabled = true;
      this.el.btnRequestPort.disabled = true;
    }
  }

  async refreshPortList() {
    if (!("serial" in navigator)) return;
    try {
      const ports = await navigator.serial.getPorts();
      this.el.portSelect.innerHTML = "";
      if (ports.length === 0) {
        const opt = document.createElement("option");
        opt.value = "";
        opt.textContent = "No Authorized Ports (Click PAIR PORT)";
        this.el.portSelect.appendChild(opt);
        return;
      }

      ports.forEach((p, idx) => {
        const info = p.getInfo();
        const vidHex = info.usbVendorId ? `0x${info.usbVendorId.toString(16).padStart(4, '0').toUpperCase()}` : "VID:--";
        const pidHex = info.usbProductId ? `0x${info.usbProductId.toString(16).padStart(4, '0').toUpperCase()}` : "PID:--";
        const opt = document.createElement("option");
        opt.value = idx;
        opt.textContent = `Port ${idx + 1} (${vidHex}:${pidHex})`;
        this.el.portSelect.appendChild(opt);
      });
    } catch (e) {
      console.warn("Could not retrieve ports:", e);
    }
  }

  async requestNewPort() {
    try {
      const p = await navigator.serial.requestPort();
      await this.refreshPortList();
      const ports = await navigator.serial.getPorts();
      const index = ports.indexOf(p);
      if (index >= 0) {
        this.el.portSelect.value = index;
      }
    } catch (e) {
      console.warn("Port pairing canceled or failed:", e);
    }
  }

  async toggleConnection() {
    if (this.isConnected) {
      await this.disconnect();
    } else {
      await this.connect();
    }
  }

  async connect() {
    const ports = await navigator.serial.getPorts();
    const selIdx = parseInt(this.el.portSelect.value, 10);
    if (isNaN(selIdx) || !ports[selIdx]) {
      try {
        this.port = await navigator.serial.requestPort();
      } catch (err) {
        alert("No port selected or pairing cancelled.");
        return;
      }
    } else {
      this.port = ports[selIdx];
    }

    const baudRate = parseInt(this.el.baudSelect.value, 10);
    this.protocolVersion = this.el.protocolSelect.value;
    this.saveCurrentSettings();

    try {
      await this.port.open({ baudRate, dataBits: 8, stopBits: 1, parity: "none" });
      this.isConnected = true;
      this.keepReading = true;

      // Update UI Status
      this.el.connStatusBadge.classList.add("connected");
      this.el.connStatusText.textContent = `CONNECTED (${baudRate} bps)`;
      this.el.btnConnect.textContent = "DISCONNECT";
      this.el.btnConnect.classList.replace("btn-primary", "btn-danger");
      this.el.btnScanBus.disabled = false;
      this.el.btnPingBroadcast.disabled = false;

      const info = this.port.getInfo();
      const vidHex = info.usbVendorId ? `0x${info.usbVendorId.toString(16).padStart(4, '0').toUpperCase()}` : "NONE";
      const pidHex = info.usbProductId ? `0x${info.usbProductId.toString(16).padStart(4, '0').toUpperCase()}` : "NONE";
      this.el.vidPidLabel.textContent = `VID: ${vidHex} | PID: ${pidHex}`;
      this.el.deviceInfoPill.firstElementChild.textContent = `PORT ACTIVE (${baudRate} BAUD)`;

      // Update active protocol badge
      this.el.activeProtocolBadge.textContent = `PROTOCOL ${this.protocolVersion.toUpperCase()}`;

      // Adjust slider limits for Protocol 1.0 vs Protocol 2.0
      this.applyProtocolLimits();

      // Launch non-blocking read stream
      this.readLoopPromise = this.readLoop();

      // Start Telemetry Polling
      this.updatePollingRate();

      this.logPacket("SYS", [], `Connected to serial port at ${baudRate} baud with Protocol ${this.protocolVersion}`);

      // Query active actuator Operating Mode and Torque State from hardware
      if (this.protocolVersion === "2.0") {
        setTimeout(async () => {
          try {
            const modeBlock = await this.readMemoryBlock(11, 1);
            if (modeBlock && modeBlock.params && modeBlock.params.length > 0) {
              const currentMode = modeBlock.params[0];
              this.currentOperatingMode = currentMode;
              this.updateModeBadge(currentMode);
              this.updateDynamicModePanels(currentMode);
              this.logPacket("SYS", [], `Actuator Active ID ${this.activeId} initial Operating Mode: ${currentMode}`);
            }
            const torqueBlock = await this.readMemoryBlock(64, 1);
            if (torqueBlock && torqueBlock.params && torqueBlock.params.length > 0) {
              this.torqueEnabled = torqueBlock.params[0] === 1;
              if (this.el.toggleTorque) this.el.toggleTorque.checked = this.torqueEnabled;
              if (this.el.torqueStateLabel) {
                this.el.torqueStateLabel.textContent = this.torqueEnabled ? "TORQUE ARMED" : "TORQUE DISABLED";
                this.el.torqueStateLabel.style.color = this.torqueEnabled ? "var(--accent-emerald)" : "var(--accent-rose)";
              }
            }
          } catch (e) {
            console.warn("Could not query initial operating mode:", e);
          }
        }, 150);
      }
    } catch (err) {
      console.error("Connection failed:", err);
      alert(`Could not open port: ${err.message}`);
    }
  }

  async disconnect() {
    this.stopPolling();
    this.stopSweep();
    this.keepReading = false;

    if (this.reader) {
      try {
        await this.reader.cancel();
      } catch (e) {}
    }

    if (this.readLoopPromise) {
      await this.readLoopPromise;
    }

    if (this.port) {
      try {
        await this.port.close();
      } catch (e) {}
    }

    this.port = null;
    this.reader = null;
    this.writer = null;
    this.isConnected = false;

    this.el.connStatusBadge.classList.remove("connected");
    this.el.connStatusText.textContent = "DISCONNECTED";
    this.el.btnConnect.textContent = "CONNECT HARDWARE";
    this.el.btnConnect.classList.replace("btn-danger", "btn-primary");
    this.el.btnScanBus.disabled = true;
    this.el.btnPingBroadcast.disabled = true;
    this.logPacket("SYS", [], "Serial port disconnected.");
  }

  applyProtocolLimits() {
    if (this.protocolVersion === "1.0") {
      // AX-12A range: 0-1023 (0° to 300°)
      this.el.sliderPosition.max = "1023";
      if (this.goalPosition > 1023) this.goalPosition = 512;
      this.el.sliderPosition.value = this.goalPosition;
    } else {
      // Protocol 2.0 range: 0-4095 (0° to 360°)
      this.el.sliderPosition.max = "4095";
      if (this.goalPosition > 4095) this.goalPosition = 2048;
      this.el.sliderPosition.value = this.goalPosition;
    }
    this.updateDialDisplay(this.goalPosition);
  }

  // --------------------------------------------------------------------------
  // Asynchronous Serial Reader Loop
  // --------------------------------------------------------------------------
  async readLoop() {
    while (this.port && this.port.readable && this.keepReading) {
      try {
        this.reader = this.port.readable.getReader();
        while (this.keepReading) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value && value.length > 0) {
            this.handleIncomingBytes(value);
          }
        }
      } catch (err) {
        if (this.keepReading) {
          console.warn("Read stream error:", err);
        }
      } finally {
        if (this.reader) {
          try {
            this.reader.releaseLock();
          } catch (e) {}
          this.reader = null;
        }
      }
    }
  }

  handleIncomingBytes(bytes) {
    for (let i = 0; i < bytes.length; i++) {
      this.rxBuffer.push(bytes[i]);
    }

    // Packet parsing for Protocol 1.0 or 2.0
    if (this.protocolVersion === "2.0") {
      this.parseProtocol2Stream();
    } else if (this.protocolVersion === "1.0") {
      this.parseProtocol1Stream();
    } else {
      this.parseAsciiStream();
    }
  }

  parseProtocol2Stream() {
    // Protocol 2.0 Header: 0xFF, 0xFF, 0xFD, 0x00
    while (this.rxBuffer.length >= 11) {
      // Find header
      if (
        this.rxBuffer[0] !== 0xff ||
        this.rxBuffer[1] !== 0xff ||
        this.rxBuffer[2] !== 0xfd ||
        this.rxBuffer[3] !== 0x00
      ) {
        this.rxBuffer.shift();
        continue;
      }

      const id = this.rxBuffer[4];
      const length = this.rxBuffer[5] | (this.rxBuffer[6] << 8);
      const totalLen = 7 + length;

      if (this.rxBuffer.length < totalLen) {
        return; // Wait for full packet
      }

      const packetBytes = this.rxBuffer.splice(0, totalLen);
      const crcExpected = packetBytes[totalLen - 2] | (packetBytes[totalLen - 1] << 8);
      const crcActual = DXL.calculateCRC2(packetBytes, totalLen - 2);

      if (crcActual !== crcExpected) {
        this.logPacket("RX", packetBytes, `ID ${id} CRC MISMATCH (Exp: ${crcExpected.toString(16)}, Act: ${crcActual.toString(16)})`);
        continue;
      }

      const instruction = packetBytes[7];
      const errorByte = packetBytes[8];
      const params = packetBytes.slice(9, totalLen - 2);

      this.logPacket("RX", packetBytes, `ID ${id} [STATUS 0x55] Err: 0x${errorByte.toString(16)} DataLen: ${params.length}`);
      this.processStatusPacket(id, instruction, errorByte, params);
    }
  }

  parseProtocol1Stream() {
    // Protocol 1.0 Header: 0xFF, 0xFF
    while (this.rxBuffer.length >= 6) {
      if (this.rxBuffer[0] !== 0xff || this.rxBuffer[1] !== 0xff) {
        this.rxBuffer.shift();
        continue;
      }

      const id = this.rxBuffer[2];
      const length = this.rxBuffer[3];
      const totalLen = 4 + length;

      if (this.rxBuffer.length < totalLen) {
        return; // Wait for full packet
      }

      const packetBytes = this.rxBuffer.splice(0, totalLen);
      const checksumActual = packetBytes[totalLen - 1];
      const checksumCalculated = DXL.calculateChecksum1(packetBytes.slice(0, totalLen - 1));

      if (checksumActual !== checksumCalculated) {
        this.logPacket("RX", packetBytes, `ID ${id} CHECKSUM MISMATCH`);
        continue;
      }

      const errorByte = packetBytes[4];
      const params = packetBytes.slice(5, totalLen - 1);

      this.logPacket("RX", packetBytes, `ID ${id} [STATUS] Err: 0x${errorByte.toString(16)} DataLen: ${params.length}`);
      this.processStatusPacket(id, 0x55, errorByte, params);
    }
  }

  parseAsciiStream() {
    // In ASCII mode, decode as text line
    const text = String.fromCharCode(...this.rxBuffer);
    const nlIdx = text.indexOf('\n');
    if (nlIdx !== -1) {
      const line = text.substring(0, nlIdx).trim();
      this.rxBuffer.splice(0, nlIdx + 1);
      if (line) {
        this.logPacket("RX", [], `[ASCII] ${line}`);
      }
    }
  }

  processStatusPacket(id, instruction, errorByte, params) {
    // Check if an explicit memory block read is waiting for this response
    if (this.activeReadResolver && id === this.activeId) {
      const res = this.activeReadResolver;
      this.activeReadResolver = null;
      res.resolve({ addr: res.addr, error: errorByte, params });
      return;
    }

    // Notify any pending resolver (e.g. Ping or Bus Scan)
    const resolverKey = `ID_${id}`;
    if (this.pendingResolvers.has(resolverKey)) {
      const cb = this.pendingResolvers.get(resolverKey);
      this.pendingResolvers.delete(resolverKey);
      cb({ id, error: errorByte, params });
    }

    // Update active telemetry if packet is from active servo
    if (id === this.activeId) {
      this.hardwareError = errorByte;
      this.el.hudErrorVal.textContent = errorByte === 0 ? "NORMAL" : `ERR: 0x${errorByte.toString(16).toUpperCase()}`;
      this.el.hudErrorVal.style.color = errorByte === 0 ? "var(--accent-emerald)" : "var(--accent-rose)";

      if (this.protocolVersion === "2.0") {
        this.decodeTelemetryP2(params);
      } else {
        this.decodeTelemetryP1(params);
      }
    }
  }

  decodeTelemetryP2(params) {
    if (params.length >= 8) {
      // Address 128: Present Velocity (4 bytes signed int32)
      this.presentVelocity = DXL.bytesToInt(params.slice(0, 4), true);
      const rpm = (this.presentVelocity * 0.229).toFixed(1);
      this.el.hudVelVal.textContent = rpm;
      if (this.el.velLiveReadout) {
        this.el.velLiveReadout.textContent = `${rpm} RPM`;
      }
      if (this.el.testSpeedActualReadout) {
        this.el.testSpeedActualReadout.textContent = `${rpm} RPM`;
      }

      // Address 132: Present Position (4 bytes signed int32, supports ±256 multi-turn)
      this.presentPosition = DXL.bytesToInt(params.slice(4, 8), true);
      const absPos = Math.abs(this.presentPosition);
      const angleDeg = ((absPos % 4096) / 4095.0) * 360.0;
      this.el.hudPosVal.textContent = this.presentPosition;
      this.el.hudPosDeg.textContent = `${angleDeg.toFixed(1)}°`;

      // Update arm on SVG dial
      this.el.svgPresentArm.setAttribute("transform", `rotate(${angleDeg} 130 130)`);

      // Update multiturn live readout
      const turns = this.presentPosition / 4096.0;
      const turnStr = `${turns >= 0 ? '+' : ''}${turns.toFixed(2)} Revs (${this.presentPosition} ticks)`;
      if (this.el.multiturnLiveReadout) {
        this.el.multiturnLiveReadout.textContent = turnStr;
      }
      if (this.el.testMultiturnActualReadout) {
        this.el.testMultiturnActualReadout.textContent = turnStr;
      }

      // Record point for Scope
      this.recordScopePoint();
    } else if (params.length >= 4) {
      this.presentPosition = DXL.bytesToInt(params.slice(0, 4), true);
      const absPos = Math.abs(this.presentPosition);
      const angleDeg = ((absPos % 4096) / 4095.0) * 360.0;
      this.el.hudPosVal.textContent = this.presentPosition;
      this.el.hudPosDeg.textContent = `${angleDeg.toFixed(1)}°`;
      this.el.svgPresentArm.setAttribute("transform", `rotate(${angleDeg} 130 130)`);
      const turns = this.presentPosition / 4096.0;
      const turnStr = `${turns >= 0 ? '+' : ''}${turns.toFixed(2)} Revs (${this.presentPosition} ticks)`;
      if (this.el.multiturnLiveReadout) this.el.multiturnLiveReadout.textContent = turnStr;
      if (this.el.testMultiturnActualReadout) this.el.testMultiturnActualReadout.textContent = turnStr;
      this.recordScopePoint();
    }
  }

  decodeTelemetryP1(params) {
    if (params.length >= 2) {
      this.presentPosition = DXL.bytesToInt(params.slice(0, 2), false);
      const angleDeg = (this.presentPosition / 1023.0) * 300.0;
      this.el.hudPosVal.textContent = this.presentPosition;
      this.el.hudPosDeg.textContent = `${angleDeg.toFixed(1)}°`;

      this.el.svgPresentArm.setAttribute("transform", `rotate(${angleDeg} 130 130)`);

      if (params.length >= 4) {
        this.presentVelocity = DXL.bytesToInt(params.slice(2, 4), true);
        this.el.hudVelVal.textContent = (this.presentVelocity * 0.111).toFixed(1); // RPM
      }

      if (params.length >= 6) {
        this.presentLoad = DXL.bytesToInt(params.slice(4, 6), true);
        this.el.hudLoadVal.textContent = `${((this.presentLoad & 0x3ff) * 0.1).toFixed(1)} %`;
      }

      this.recordScopePoint();
    }
  }

  // --------------------------------------------------------------------------
  // Transmission Helper
  // --------------------------------------------------------------------------
  async sendPacket(packetBytes, description = "") {
    if (!this.port || !this.port.writable || !this.isConnected) {
      console.warn("Cannot send: Port not connected.");
      return false;
    }

    try {
      this.writer = this.port.writable.getWriter();
      await this.writer.write(packetBytes);
      this.writer.releaseLock();
      this.writer = null;

      this.logPacket("TX", packetBytes, description);
      return true;
    } catch (err) {
      console.error("Write error:", err);
      if (this.writer) {
        try { this.writer.releaseLock(); } catch (e) {}
        this.writer = null;
      }
      return false;
    }
  }

  // --------------------------------------------------------------------------
  // Bus Scanner
  // --------------------------------------------------------------------------
  async scanBus() {
    if (!this.isConnected) return;
    const startId = parseInt(this.el.scanStartId.value, 10);
    const endId = parseInt(this.el.scanEndId.value, 10);

    if (startId > endId || startId < 0 || endId > 253) {
      alert("Invalid ID range (0 to 253).");
      return;
    }

    this.el.btnScanBus.disabled = true;
    this.el.scanProgressBar.style.width = "0%";
    this.discoveredServos.clear();
    this.renderDiscoveredList();

    const total = endId - startId + 1;
    let scanned = 0;

    for (let id = startId; id <= endId; id++) {
      if (!this.isConnected) break;

      const progress = ((scanned + 1) / total) * 100;
      this.el.scanProgressBar.style.width = `${progress}%`;

      await this.pingId(id);
      scanned++;
      await new Promise(r => setTimeout(r, 20)); // Small yield between pings
    }

    this.el.btnScanBus.disabled = false;
    this.el.scanProgressBar.style.width = "100%";
    setTimeout(() => { this.el.scanProgressBar.style.width = "0%"; }, 800);
  }

  async pingId(id) {
    const packet = this.protocolVersion === "2.0"
      ? DXL.buildPacketP2(id, DXL.INST.PING, [])
      : DXL.buildPacketP1(id, DXL.INST.PING, []);

    const key = `ID_${id}`;
    const p = new Promise(resolve => {
      this.pendingResolvers.set(key, resolve);
      // 50ms ping timeout
      setTimeout(() => {
        if (this.pendingResolvers.has(key)) {
          this.pendingResolvers.delete(key);
          resolve(null);
        }
      }, 55);
    });

    await this.sendPacket(packet, `PING ID ${id}`);
    const res = await p;

    if (res) {
      let modelName = "Unknown Dynamixel";
      let modelId = null;

      if (this.protocolVersion === "2.0" && res.params.length >= 2) {
        modelId = DXL.bytesToInt(res.params.slice(0, 2), false);
        modelName = DXL.MODELS[modelId] || `Model 0x${modelId.toString(16).toUpperCase()}`;
      } else if (this.protocolVersion === "1.0") {
        modelName = "AX / MX Series";
      }

      this.discoveredServos.set(id, { id, model: modelName, modelId });
      this.renderDiscoveredList();

      // If active servo not yet selected, set to first found
      if (this.discoveredServos.size === 1) {
        this.selectActiveServo(id);
      }
    }
  }

  renderDiscoveredList() {
    this.el.discoveredCount.textContent = this.discoveredServos.size;
    this.el.discoveredServoList.innerHTML = "";

    if (this.discoveredServos.size === 0) {
      this.el.discoveredServoList.innerHTML = `
        <div style="font-size: 0.78rem; color: var(--text-dim); text-align: center; padding: 20px 0;">
          No servos discovered yet.<br>Click "SCAN BUS" to search IDs.
        </div>`;
      return;
    }

    this.discoveredServos.forEach(servo => {
      const item = document.createElement("div");
      item.className = `servo-item ${servo.id === this.activeId ? 'active' : ''}`;
      item.innerHTML = `
        <div class="servo-item-id">
          <span style="font-size: 1rem;">⚡</span>
          <span>ID ${servo.id}</span>
        </div>
        <div class="servo-item-meta">
          <span class="servo-model-badge">${servo.model}</span>
        </div>
      `;
      item.addEventListener("click", () => this.selectActiveServo(servo.id));
      this.el.discoveredServoList.appendChild(item);
    });
  }

  selectActiveServo(id) {
    this.activeId = id;
    const servo = this.discoveredServos.get(id);
    this.el.activeIdDisplay.textContent = id;
    this.el.activeModelDisplay.textContent = servo ? servo.model : "Generic DXL";
    this.renderDiscoveredList();
    this.queryServoState();

    if (this.protocolVersion === "2.0") {
      setTimeout(async () => {
        try {
          const modeBlock = await this.readMemoryBlock(11, 1);
          if (modeBlock && modeBlock.params && modeBlock.params.length > 0) {
            const currentMode = modeBlock.params[0];
            this.currentOperatingMode = currentMode;
            this.updateModeBadge(currentMode);
            this.updateDynamicModePanels(currentMode);
          }
          const torqueBlock = await this.readMemoryBlock(64, 1);
          if (torqueBlock && torqueBlock.params && torqueBlock.params.length > 0) {
            this.torqueEnabled = torqueBlock.params[0] === 1;
            if (this.el.toggleTorque) this.el.toggleTorque.checked = this.torqueEnabled;
            if (this.el.torqueStateLabel) {
              this.el.torqueStateLabel.textContent = this.torqueEnabled ? "Motor Armed (Torque ON)" : "Motor Free (Torque OFF)";
            }
            if (this.el.torqueAlertBanner) {
              this.el.torqueAlertBanner.style.display = this.torqueEnabled ? "none" : "flex";
            }
          }
        } catch (e) {}
      }, 50);
    }
  }

  // --------------------------------------------------------------------------
  // Actuator Control Operations
  // --------------------------------------------------------------------------
  async setTorque(enable) {
    this.torqueEnabled = enable;
    if (this.el.toggleTorque) this.el.toggleTorque.checked = enable;
    this.el.torqueStateLabel.textContent = enable ? "Motor Armed (Torque ON)" : "Motor Free (Torque OFF)";
    if (this.el.torqueAlertBanner) {
      this.el.torqueAlertBanner.style.display = enable ? "none" : "flex";
    }

    const addr = this.protocolVersion === "2.0" ? 64 : 24;
    const packet = this.protocolVersion === "2.0"
      ? DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [addr & 0xff, (addr >> 8) & 0xff, enable ? 1 : 0])
      : DXL.buildPacketP1(this.activeId, DXL.INST.WRITE, [addr, enable ? 1 : 0]);

    await this.sendPacket(packet, `TORQUE ENABLE = ${enable ? 1 : 0} (ID ${this.activeId})`);

    // On Protocol 1.0 (AX-12A), ensure Torque Limit (34) is max (1023)
    if (enable && this.protocolVersion === "1.0") {
      const limitPkt = DXL.buildPacketP1(this.activeId, DXL.INST.WRITE, [34, 0xff, 0x03]); // 1023
      await this.sendPacket(limitPkt, `TORQUE LIMIT = 1023 (ID ${this.activeId})`);
    }
  }

  async setOperatingMode(mode) {
    const modeVal = parseInt(mode, 10);

    // Robotis Hardware Rule: Operating Mode CANNOT be changed while Torque is ON!
    // Unconditionally ensure torque is OFF on physical actuator first
    this.logPacket("SYS", [], `Disabling Torque on Actuator (ID ${this.activeId}) to safely change Operating Mode to ${modeVal}...`);
    await this.setTorque(false);
    await new Promise(r => setTimeout(r, 40));

    if (this.protocolVersion === "2.0") {
      // Address 11 (Operating Mode, 1 byte)
      const addr = 11;
      const packet = DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [addr & 0xff, (addr >> 8) & 0xff, modeVal & 0xff]);
      await this.sendPacket(packet, `WRITE OPERATING MODE = ${modeVal} (ID ${this.activeId})`);
      await new Promise(r => setTimeout(r, 40));

      // For Position Mode (3) or Extended Position Mode (4), set Profile Velocity (112) = 200 (~45.8 RPM)
      // to ensure the actuator has an active velocity trajectory when commanded
      if (modeVal === 3 || modeVal === 4) {
        const pvBytes = DXL.intToBytes(200, 4);
        const pvPkt = DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [112, 0, ...pvBytes]);
        await this.sendPacket(pvPkt, `SET PROFILE VELOCITY = 200 (ID ${this.activeId})`);
        await new Promise(r => setTimeout(r, 30));
      }
    } else if (this.protocolVersion === "1.0") {
      // In Protocol 1.0 (AX-12A):
      // Joint Mode: CW Limit (6) = 0, CCW Limit (8) = 1023
      // Wheel Mode: CW Limit (6) = 0, CCW Limit (8) = 0
      const isJoint = (modeVal === 3 || modeVal === 4 || modeVal === 5);
      const ccw = isJoint ? 1023 : 0;
      const packet = DXL.buildPacketP1(this.activeId, DXL.INST.WRITE, [6, 0, 0, ccw & 0xff, (ccw >> 8) & 0xff]);
      await this.sendPacket(packet, `WRITE CW/CCW LIMITS (ID ${this.activeId}, CCW=${ccw})`);
      await new Promise(r => setTimeout(r, 40));
    }

    // Re-enable torque so the motor is energized and responsive in the new mode
    await this.setTorque(true);
    await new Promise(r => setTimeout(r, 40));

    this.currentOperatingMode = modeVal;
    this.updateModeBadge(modeVal);
    this.updateDynamicModePanels(modeVal);
    this.logPacket("SYS", [], `Operating Mode successfully updated to Mode ${modeVal} (Torque Armed)`);
  }

  updateModeBadge(modeVal) {
    this.currentOperatingMode = modeVal;
    const modeNames = {
      0: "CURRENT (0)",
      1: "VELOCITY / SPEED (1)",
      3: "POSITION (3)",
      4: "EXT MULTITURN (4)",
      5: "CURRENT-POS (5)",
      16: "PWM (16)",
    };
    const name = modeNames[modeVal] || `MODE (${modeVal})`;
    if (this.el.activeModeBadge) {
      this.el.activeModeBadge.textContent = `MODE: ${name}`;
      this.el.activeModeBadge.style.color = (modeVal === 3 || modeVal === 4 || modeVal === 5) ? "#34d399" : "#f59e0b";
    }
    if (this.el.operatingModeSelect) {
      this.el.operatingModeSelect.value = modeVal;
    }
  }

  updateDynamicModePanels(modeVal) {
    if (this.el.velocityControlBox) {
      this.el.velocityControlBox.style.display = (modeVal === 1) ? "flex" : "none";
    }
    if (this.el.multiturnControlBox) {
      this.el.multiturnControlBox.style.display = (modeVal === 4) ? "flex" : "none";
    }
  }

  async setGoalVelocity(rpm) {
    if (!this.isConnected) return;

    // Automatically ensure motor is in Velocity Control Mode (Mode 1)
    if (this.protocolVersion === "2.0" && this.currentOperatingMode !== 1) {
      this.logPacket("SYS", [], `Actuator not in Velocity Mode (Mode 1). Auto-switching to Mode 1...`);
      await this.setOperatingMode(1);
      await new Promise(r => setTimeout(r, 50));
    } else if (!this.torqueEnabled) {
      await this.setTorque(true);
      await new Promise(r => setTimeout(r, 30));
    }

    if (this.protocolVersion === "2.0") {
      // 0.229 rpm / unit in Protocol 2.0 (Address 104, 4 bytes, signed int32)
      const rawUnits = Math.round(rpm / 0.229);
      const bytes = DXL.intToBytes(rawUnits, 4);
      const packet = DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [104, 0, ...bytes]);
      await this.sendPacket(packet, `GOAL VELOCITY = ${rpm} RPM (${rawUnits}) (ID ${this.activeId})`);
    } else if (this.protocolVersion === "1.0") {
      const absRpm = Math.abs(rpm);
      let rawUnits = Math.min(1023, Math.round(absRpm / 0.111));
      if (rpm < 0) {
        rawUnits |= 1024; // Direction bit 10
      }
      const bytes = DXL.intToBytes(rawUnits, 2);
      const packet = DXL.buildPacketP1(this.activeId, DXL.INST.WRITE, [32, ...bytes]);
      await this.sendPacket(packet, `MOVING SPEED = ${rpm} RPM (${rawUnits}) (ID ${this.activeId})`);
    }

    this.goalVelocityRpm = rpm;
    if (this.el.testSpeedValReadout) {
      this.el.testSpeedValReadout.textContent = `${rpm.toFixed(1)} RPM`;
    }
  }

  async setMultiturnRevolutions(turns) {
    if (!this.isConnected) return;

    if (this.protocolVersion !== "2.0") {
      alert("Multiturn (Extended Position) mode is supported on Protocol 2.0 actuators.");
      return;
    }

    this.multiturnGoalTurns = turns;

    // Automatically ensure motor is in Extended Position Control Mode (Mode 4)
    if (this.currentOperatingMode !== 4) {
      this.logPacket("SYS", [], `Actuator not in Extended Position Mode (Mode 4). Auto-switching to Mode 4...`);
      await this.setOperatingMode(4);
      await new Promise(r => setTimeout(r, 50));
    } else if (!this.torqueEnabled) {
      await this.setTorque(true);
      await new Promise(r => setTimeout(r, 30));
    }

    const ticks = Math.round(turns * 4096);
    this.goalPosition = ticks;
    const bytes = DXL.intToBytes(ticks, 4);
    const packet = DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [116, 0, ...bytes]);
    await this.sendPacket(packet, `MULTITURN GOAL = ${turns.toFixed(2)} Revs (${ticks} ticks) (ID ${this.activeId})`);
  }

  async readMemoryBlock(addr, length) {
    if (!this.isConnected) return null;
    let packet;
    if (this.protocolVersion === "2.0") {
      packet = DXL.buildPacketP2(this.activeId, DXL.INST.READ, [
        addr & 0xff, (addr >> 8) & 0xff,
        length & 0xff, (length >> 8) & 0xff
      ]);
    } else {
      packet = DXL.buildPacketP1(this.activeId, DXL.INST.READ, [addr, length]);
    }

    return new Promise(async resolve => {
      this.activeReadResolver = { addr, length, resolve };
      const timeout = setTimeout(() => {
        if (this.activeReadResolver) {
          this.activeReadResolver = null;
          resolve(null);
        }
      }, 220);

      await this.sendPacket(packet, `READ ADDR ${addr} LEN ${length} (ID ${this.activeId})`);
    });
  }

  async readAllMotorParameters() {
    if (!this.isConnected) {
      alert("Please connect hardware first.");
      return;
    }

    if (this.el.paramReadStatus) {
      this.el.paramReadStatus.textContent = "Querying EEPROM & RAM from actuator...";
    }
    if (this.el.btnReadAllParams) this.el.btnReadAllParams.disabled = true;

    try {
      const paramValues = new Map();

      if (this.protocolVersion === "2.0") {
        // Block 1: Model & Config (Addr 0, Len 14)
        const b1 = await this.readMemoryBlock(0, 14);
        if (b1 && b1.params) {
          const p = b1.params;
          paramValues.set(0, DXL.bytesToInt(p.slice(0, 2), false));
          paramValues.set(6, p[6]);
          paramValues.set(7, p[7]);
          paramValues.set(8, p[8]);
          paramValues.set(9, p[9]);
          paramValues.set(10, p[10]);
          paramValues.set(11, p[11]);
          paramValues.set(12, p[12]);
          paramValues.set(13, p[13]);
        }

        await new Promise(r => setTimeout(r, 25));

        // Block 2: Limits (Addr 20, Len 36)
        const b2 = await this.readMemoryBlock(20, 36);
        if (b2 && b2.params) {
          const p = b2.params;
          paramValues.set(20, DXL.bytesToInt(p.slice(0, 4), true));
          paramValues.set(24, DXL.bytesToInt(p.slice(4, 8), false));
          paramValues.set(31, p[11]);
          paramValues.set(32, DXL.bytesToInt(p.slice(12, 14), false));
          paramValues.set(34, DXL.bytesToInt(p.slice(14, 16), false));
          paramValues.set(36, DXL.bytesToInt(p.slice(16, 18), false));
          paramValues.set(38, DXL.bytesToInt(p.slice(18, 20), false));
          paramValues.set(40, DXL.bytesToInt(p.slice(20, 24), false));
          paramValues.set(44, DXL.bytesToInt(p.slice(24, 28), false));
          paramValues.set(48, DXL.bytesToInt(p.slice(28, 32), false));
          paramValues.set(52, DXL.bytesToInt(p.slice(32, 36), false));
        }

        await new Promise(r => setTimeout(r, 25));

        // Block 3: State & Control Loop Gains (Addr 63, Len 30)
        const b3 = await this.readMemoryBlock(63, 30);
        if (b3 && b3.params) {
          const p = b3.params;
          paramValues.set(63, p[0]);
          paramValues.set(64, p[1]);
          paramValues.set(65, p[2]);
          paramValues.set(70, p[7]);
          paramValues.set(76, DXL.bytesToInt(p.slice(13, 15), false));
          paramValues.set(78, DXL.bytesToInt(p.slice(15, 17), false));
          paramValues.set(80, DXL.bytesToInt(p.slice(17, 19), false));
          paramValues.set(82, DXL.bytesToInt(p.slice(19, 21), false));
          paramValues.set(84, DXL.bytesToInt(p.slice(21, 23), false));
          paramValues.set(88, DXL.bytesToInt(p.slice(25, 27), false));
          paramValues.set(90, DXL.bytesToInt(p.slice(27, 29), false));
        }
      } else {
        // Protocol 1.0 (AX-12A)
        const b1 = await this.readMemoryBlock(0, 15);
        if (b1 && b1.params) {
          const p = b1.params;
          paramValues.set(0, DXL.bytesToInt(p.slice(0, 2), false));
          paramValues.set(2, p[2]);
          paramValues.set(3, p[3]);
          paramValues.set(4, p[4]);
          paramValues.set(5, p[5]);
          paramValues.set(6, DXL.bytesToInt(p.slice(6, 8), false));
          paramValues.set(8, DXL.bytesToInt(p.slice(8, 10), false));
          paramValues.set(11, p[11]);
          paramValues.set(12, p[12]);
          paramValues.set(13, p[13]);
          paramValues.set(14, DXL.bytesToInt(p.slice(14, 16), false));
        }
      }

      this.currentMotorParams = paramValues;
      this.renderMotorParametersTable(paramValues);
      if (this.el.paramReadStatus) {
        this.el.paramReadStatus.textContent = `Completed (${paramValues.size} parameters retrieved successfully)`;
      }
    } finally {
      if (this.el.btnReadAllParams) this.el.btnReadAllParams.disabled = false;
    }
  }

  renderMotorParametersTable(paramValues) {
    const modelNum = paramValues.get(0) || 0;
    const modelInfo = DXL.getModelInfo(modelNum);
    const fwVer = paramValues.get(6) !== undefined ? paramValues.get(6) : paramValues.get(2);

    // Update Identity Cards
    if (this.el.paramModelName) this.el.paramModelName.textContent = modelInfo.name;
    if (this.el.paramModelNum) this.el.paramModelNum.textContent = `Model ID: ${modelNum} (0x${modelNum.toString(16).toUpperCase()})`;
    if (this.el.paramSeries) this.el.paramSeries.textContent = modelInfo.series;
    if (this.el.paramBus) this.el.paramBus.textContent = `Bus: ${modelInfo.bus}`;
    if (this.el.paramFwVer) this.el.paramFwVer.textContent = fwVer !== undefined ? `v${fwVer} (0x${fwVer.toString(16)})` : "--";
    if (this.el.paramProtocolVer) this.el.paramProtocolVer.textContent = `Protocol ${this.protocolVersion}`;

    if (!this.el.paramTableBody) return;
    this.el.paramTableBody.innerHTML = "";

    const registry = (this.protocolVersion === "2.0") ? DXL.PARAM_REGISTRY_P2 : [];

    registry.forEach(entry => {
      const val = paramValues.get(entry.addr);
      if (val === undefined) return;

      let displayVal = `${val}`;
      if (entry.unit) {
        if (entry.unit === "0.229 rpm") {
          displayVal = `${(val * 0.229).toFixed(1)} RPM (${val})`;
        } else if (entry.unit === "0.1 V") {
          displayVal = `${(val * 0.1).toFixed(1)} V (${val})`;
        } else if (entry.unit === "2 µs") {
          displayVal = `${val * 2} µs`;
        } else if (entry.addr === 11) {
          const modeMap = { 0: "Current", 1: "Velocity", 3: "Position", 4: "Ext Position", 5: "Current-Pos", 16: "PWM" };
          displayVal = `${val} [${modeMap[val] || "Custom"}]`;
        } else if (entry.addr === 8) {
          const baudMap = { 0: "9600", 1: "57600", 2: "115200", 3: "1 Mbps", 4: "2 Mbps", 5: "3 Mbps", 6: "4 Mbps", 7: "4.5 Mbps" };
          displayVal = `${val} [${baudMap[val] || `${val} index`}]`;
        } else {
          displayVal = `${val} ${entry.unit}`;
        }
      }

      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td class="param-addr">${entry.addr} (0x${entry.addr.toString(16).padStart(2, '0').toUpperCase()})</td>
        <td class="param-name">${entry.name}</td>
        <td class="param-val">${displayVal}</td>
        <td><span class="param-chip ${entry.eeprom ? 'eeprom' : 'ram'}">${entry.eeprom ? 'EEPROM' : 'RAM'}</span></td>
        <td style="color: var(--text-dim); font-size: 0.75rem;">${entry.desc}</td>
      `;
      this.el.paramTableBody.appendChild(tr);
    });
  }

  exportParamsJson() {
    if (!this.currentMotorParams || this.currentMotorParams.size === 0) {
      alert("Please click 'READ ALL MOTOR PARAMETERS' first.");
      return;
    }

    const obj = {
      timestamp: new Date().toISOString(),
      servoId: this.activeId,
      protocol: this.protocolVersion,
      parameters: {}
    };

    const registry = (this.protocolVersion === "2.0") ? DXL.PARAM_REGISTRY_P2 : [];
    registry.forEach(entry => {
      const val = this.currentMotorParams.get(entry.addr);
      if (val !== undefined) {
        obj.parameters[entry.name] = {
          address: entry.addr,
          value: val,
          unit: entry.unit,
          type: entry.eeprom ? "EEPROM" : "RAM",
          description: entry.desc
        };
      }
    });

    const jsonStr = JSON.stringify(obj, null, 2);
    const blob = new Blob([jsonStr], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `dynamixel_id${this.activeId}_params_${new Date().toISOString().slice(0, 10)}.json`;
    a.click();
    URL.revokeObjectURL(url);
  }

  async startRampSpeedTest() {
    if (!this.isConnected) return;
    this.logPacket("SYS", [], "Starting Ramp Speed Test: 0 -> 50 RPM -> 0 RPM");
    await this.setOperatingMode(1);
    await this.setTorque(true);

    const steps = 20;
    for (let i = 0; i <= steps; i++) {
      const rpm = (50.0 / steps) * i;
      await this.setGoalVelocity(rpm);
      if (this.el.testSpeedSlider) this.el.testSpeedSlider.value = Math.round(rpm);
      if (this.el.testSpeedValReadout) this.el.testSpeedValReadout.textContent = `${rpm.toFixed(1)} RPM`;
      await new Promise(r => setTimeout(r, 100));
    }
    await new Promise(r => setTimeout(r, 1000));
    for (let i = steps; i >= 0; i--) {
      const rpm = (50.0 / steps) * i;
      await this.setGoalVelocity(rpm);
      if (this.el.testSpeedSlider) this.el.testSpeedSlider.value = Math.round(rpm);
      if (this.el.testSpeedValReadout) this.el.testSpeedValReadout.textContent = `${rpm.toFixed(1)} RPM`;
      await new Promise(r => setTimeout(r, 100));
    }
    this.logPacket("SYS", [], "Ramp Speed Test completed.");
  }

  async startOscillateSpeedTest() {
    if (!this.isConnected) return;
    this.logPacket("SYS", [], "Starting Oscillate Speed Test: ±30 RPM");
    await this.setOperatingMode(1);
    await this.setTorque(true);

    const speeds = [30, -30, 30, -30, 0];
    for (const spd of speeds) {
      await this.setGoalVelocity(spd);
      if (this.el.testSpeedSlider) this.el.testSpeedSlider.value = spd;
      if (this.el.testSpeedValReadout) this.el.testSpeedValReadout.textContent = `${spd} RPM`;
      await new Promise(r => setTimeout(r, 1200));
    }
    this.logPacket("SYS", [], "Oscillate Speed Test completed.");
  }

  async startMultiturnSweepTest() {
    if (!this.isConnected) return;
    this.logPacket("SYS", [], "Starting Multiturn Sweep Test: -3 Revs -> +3 Revs -> 0 Revs");
    await this.setOperatingMode(4);
    await this.setTorque(true);

    const targets = [-3.0, 3.0, 0.0];
    for (const t of targets) {
      await this.setMultiturnRevolutions(t);
      if (this.el.testTurnsInput) this.el.testTurnsInput.value = t;
      if (this.el.testTurnsTicksReadout) this.el.testTurnsTicksReadout.value = `${Math.round(t * 4096)} ticks`;
      await new Promise(r => setTimeout(r, 2500));
    }
    this.logPacket("SYS", [], "Multiturn Sweep Test completed.");
  }

  async setLed(enable) {
    this.ledEnabled = enable;
    this.el.ledStateLabel.textContent = enable ? "LED ON" : "LED OFF";

    const addr = this.protocolVersion === "2.0" ? 65 : 25;
    const packet = this.protocolVersion === "2.0"
      ? DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [addr & 0xff, (addr >> 8) & 0xff, enable ? 1 : 0])
      : DXL.buildPacketP1(this.activeId, DXL.INST.WRITE, [addr, enable ? 1 : 0]);

    await this.sendPacket(packet, `LED = ${enable ? 1 : 0} (ID ${this.activeId})`);
  }

  async setGoalPosition(ticks) {
    this.goalPosition = ticks;
    this.multiturnGoalTurns = ticks / 4096.0;
    this.updateDialDisplay(ticks);

    // Auto-enable torque and ensure motor is in a position mode so it moves
    if (this.protocolVersion === "2.0" && this.currentOperatingMode === 1) {
      this.logPacket("SYS", [], `Actuator in Velocity Mode (Mode 1). Switching to Position Mode (Mode 3) for position target...`);
      await this.setOperatingMode(3);
      await new Promise(r => setTimeout(r, 50));
    } else if (!this.torqueEnabled) {
      await this.setTorque(true);
      await new Promise(r => setTimeout(r, 30));
    }

    let packet;
    if (this.protocolVersion === "2.0") {
      const addr = 116; // Goal Position
      const bytes = DXL.intToBytes(ticks, 4);
      packet = DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [addr & 0xff, (addr >> 8) & 0xff, ...bytes]);
    } else {
      const addr = 30; // Goal Position (2 bytes)
      const bytes = DXL.intToBytes(ticks, 2);
      packet = DXL.buildPacketP1(this.activeId, DXL.INST.WRITE, [addr, ...bytes]);
    }

    await this.sendPacket(packet, `GOAL POS = ${ticks} (ID ${this.activeId})`);
  }

  updateDialDisplay(ticks) {
    const maxTicks = this.protocolVersion === "1.0" ? 1023 : 4095;
    const maxAngle = this.protocolVersion === "1.0" ? 300.0 : 360.0;
    const angleDeg = (ticks / maxTicks) * maxAngle;

    this.el.dialAngleText.textContent = `${angleDeg.toFixed(1)}°`;
    this.el.dialTicksText.textContent = `${ticks} TICKS`;
    this.el.sliderPosReadout.textContent = `${ticks} (${angleDeg.toFixed(1)}°)`;
    this.el.sliderPosition.value = ticks;

    // Rotate goal pointer
    this.el.svgGoalArm.setAttribute("transform", `rotate(${angleDeg} 130 130)`);

    // Draw active arc from 0 to angleDeg
    this.drawSvgArc(angleDeg);
  }

  drawSvgArc(angleDeg) {
    const cx = 130, cy = 130, r = 105;
    const startAngle = -90; // Top
    const endAngle = startAngle + Math.min(359.9, angleDeg);

    const rad1 = startAngle * Math.PI / 180;
    const rad2 = endAngle * Math.PI / 180;

    const x1 = cx + r * Math.cos(rad1);
    const y1 = cy + r * Math.sin(rad1);
    const x2 = cx + r * Math.cos(rad2);
    const y2 = cy + r * Math.sin(rad2);

    const largeArc = angleDeg > 180 ? 1 : 0;
    const d = `M ${x1} ${y1} A ${r} ${r} 0 ${largeArc} 1 ${x2} ${y2}`;
    this.el.svgArcActive.setAttribute("d", d);
  }

  async rebootServo() {
    if (!this.isConnected) return;
    if (this.protocolVersion === "2.0") {
      const packet = DXL.buildPacketP2(this.activeId, DXL.INST.REBOOT, []);
      await this.sendPacket(packet, `REBOOT ID ${this.activeId}`);
    } else {
      alert("Reboot instruction is supported on Protocol 2.0. Power cycle hardware for Protocol 1.0.");
    }
  }

  async emergencyStop() {
    this.stopSweep();
    this.el.toggleTorque.checked = false;
    await this.setTorque(false);
    this.logPacket("SYS", [], `EMERGENCY STOP executed on ID ${this.activeId}`);
  }

  // --------------------------------------------------------------------------
  // Periodic Telemetry Polling Loop
  // --------------------------------------------------------------------------
  updatePollingRate() {
    this.stopPolling();
    const rate = parseInt(this.el.pollRateSelect.value, 10);
    this.pollIntervalMs = rate;

    if (rate === 0) {
      this.el.pollFpsLabel.textContent = "OFF";
      return;
    }

    const fps = Math.round(1000 / rate);
    this.el.pollFpsLabel.textContent = `${fps} Hz`;

    this.pollTimer = setInterval(() => {
      if (this.isConnected && !this.isPollingBusy) {
        this.queryServoState();
      }
    }, rate);
  }

  stopPolling() {
    if (this.pollTimer) {
      clearInterval(this.pollTimer);
      this.pollTimer = null;
    }
  }

  async queryServoState() {
    this.isPollingBusy = true;
    try {
      if (this.protocolVersion === "2.0") {
        // Read 8 bytes starting from Present Velocity (128):
        // 128..131: Present Velocity (4 bytes signed int32)
        // 132..135: Present Position (4 bytes signed int32)
        const addr = 128;
        const length = 8;
        const packet = DXL.buildPacketP2(this.activeId, DXL.INST.READ, [
          addr & 0xff, (addr >> 8) & 0xff,
          length & 0xff, (length >> 8) & 0xff
        ]);
        await this.sendPacket(packet, `READ VEL+POS (ID ${this.activeId})`);
      } else if (this.protocolVersion === "1.0") {
        // Read 6 bytes starting from Present Position (36): Pos (2), Speed (2), Load (2)
        const addr = 36;
        const length = 6;
        const packet = DXL.buildPacketP1(this.activeId, DXL.INST.READ, [addr, length]);
        await this.sendPacket(packet, `READ TELEMETRY (ID ${this.activeId})`);
      }
    } finally {
      this.isPollingBusy = false;
    }
  }

  // --------------------------------------------------------------------------
  // Scope Canvas Plotter (60 FPS)
  // --------------------------------------------------------------------------
  initScopeCanvas() {
    this.scopeCanvas = this.el.scopeCanvas;
    if (!this.scopeCanvas) return;
    this.scopeCtx = this.scopeCanvas.getContext('2d');
    this.resizeCanvas();
    window.addEventListener('resize', () => this.resizeCanvas());
    this.setScopePlotMode(this.scopePlotMode);
    this.renderScope();
  }

  resizeCanvas() {
    if (!this.scopeCanvas) return;
    const rect = this.scopeCanvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.scopeCanvas.width = rect.width * dpr;
    this.scopeCanvas.height = rect.height * dpr;
  }

  setScopePlotMode(mode) {
    this.scopePlotMode = mode; // "single" | "multiturn" | "speed"
    document.querySelectorAll(".scope-mode-btn").forEach(b => {
      b.classList.toggle("active", b.dataset.scopeMode === mode);
    });

    if (this.el.scopeModeWatermark) {
      if (mode === "single") {
        this.el.scopeModeWatermark.textContent = "SIGNAL: SINGLE TURN POSITION (0 - 4095 TICKS / 360°)";
      } else if (mode === "multiturn") {
        this.el.scopeModeWatermark.textContent = "SIGNAL: MULTITURN EXTENDED POSITION (REVOLUTIONS)";
      } else if (mode === "speed") {
        this.el.scopeModeWatermark.textContent = "SIGNAL: SPEED / VELOCITY (RPM)";
      }
    }

    if (this.el.scopeLegendTarget && this.el.scopeLegendActual) {
      if (mode === "single") {
        this.el.scopeLegendTarget.textContent = "--- Target Position";
        this.el.scopeLegendTarget.style.color = "#f43f5e";
        this.el.scopeLegendActual.textContent = "― Actual Position";
        this.el.scopeLegendActual.style.color = "#38bdf8";
      } else if (mode === "multiturn") {
        this.el.scopeLegendTarget.textContent = "--- Target Revs";
        this.el.scopeLegendTarget.style.color = "#f43f5e";
        this.el.scopeLegendActual.textContent = "― Actual Revs";
        this.el.scopeLegendActual.style.color = "#c084fc";
      } else if (mode === "speed") {
        this.el.scopeLegendTarget.textContent = "--- Target Speed";
        this.el.scopeLegendTarget.style.color = "#f59e0b";
        this.el.scopeLegendActual.textContent = "― Actual Speed";
        this.el.scopeLegendActual.style.color = "#10b981";
      }
    }
  }

  recordScopePoint() {
    if (this.isScopePaused) return;
    const t = (performance.now() - this.startTime) / 1000.0;

    // Single Turn Target & Actual (0 to 4095 ticks)
    const maxTicks = this.protocolVersion === "1.0" ? 1024 : 4096;
    const rawGoal = Math.round(this.goalPosition || 0);
    const rawActual = Math.round(this.presentPosition || 0);
    const singleTarget = ((rawGoal % maxTicks) + maxTicks) % maxTicks;
    const singleActual = ((rawActual % maxTicks) + maxTicks) % maxTicks;

    // Multiturn Target & Actual (Revolutions)
    const multiTargetTurns = this.multiturnGoalTurns !== undefined ? this.multiturnGoalTurns : (this.goalPosition / 4096.0);
    const multiActualTurns = this.presentPosition / 4096.0;

    // Speed / Velocity Target & Actual (RPM)
    const speedScale = this.protocolVersion === "1.0" ? 0.111 : 0.229;
    const speedTarget = this.goalVelocityRpm !== undefined ? this.goalVelocityRpm : 0;
    const speedActual = (this.presentVelocity || 0) * speedScale;

    this.scopeData.push({
      t,
      singleTarget,
      singleActual,
      multiTargetTurns,
      multiActualTurns,
      speedTarget,
      speedActual,
      load: this.presentLoad || 0,
    });

    if (this.scopeData.length > this.scopeMaxPoints) {
      this.scopeData.shift();
    }

    this.updateScopeBadges(singleTarget, singleActual, multiTargetTurns, multiActualTurns, speedTarget, speedActual);
  }

  updateScopeBadges(singleTarget, singleActual, multiTargetTurns, multiActualTurns, speedTarget, speedActual) {
    if (!this.el.scopeTargetBadge) return;

    if (this.scopePlotMode === "single") {
      const maxTicks = this.protocolVersion === "1.0" ? 1023 : 4095;
      const maxDeg = this.protocolVersion === "1.0" ? 300.0 : 360.0;
      const targetDeg = ((singleTarget / maxTicks) * maxDeg).toFixed(1);
      const actualDeg = ((singleActual / maxTicks) * maxDeg).toFixed(1);
      const err = singleActual - singleTarget;

      this.el.scopeTargetBadge.textContent = `TARGET: ${singleTarget} ticks (${targetDeg}°)`;
      this.el.scopeTargetBadge.style.color = "#f43f5e";
      this.el.scopeActualBadge.textContent = `ACTUAL: ${singleActual} ticks (${actualDeg}°)`;
      this.el.scopeActualBadge.style.color = "#38bdf8";
      this.el.scopeErrorBadge.textContent = `ERR: ${err >= 0 ? '+' : ''}${err} ticks`;
    } else if (this.scopePlotMode === "multiturn") {
      const err = multiActualTurns - multiTargetTurns;
      this.el.scopeTargetBadge.textContent = `TARGET: ${multiTargetTurns >= 0 ? '+' : ''}${multiTargetTurns.toFixed(2)} Revs`;
      this.el.scopeTargetBadge.style.color = "#f43f5e";
      this.el.scopeActualBadge.textContent = `ACTUAL: ${multiActualTurns >= 0 ? '+' : ''}${multiActualTurns.toFixed(2)} Revs`;
      this.el.scopeActualBadge.style.color = "#c084fc";
      this.el.scopeErrorBadge.textContent = `ERR: ${err >= 0 ? '+' : ''}${err.toFixed(2)} Revs`;
    } else if (this.scopePlotMode === "speed") {
      const err = speedActual - speedTarget;
      this.el.scopeTargetBadge.textContent = `TARGET: ${speedTarget.toFixed(1)} RPM`;
      this.el.scopeTargetBadge.style.color = "#f59e0b";
      this.el.scopeActualBadge.textContent = `ACTUAL: ${speedActual.toFixed(1)} RPM`;
      this.el.scopeActualBadge.style.color = "#10b981";
      this.el.scopeErrorBadge.textContent = `ERR: ${err >= 0 ? '+' : ''}${err.toFixed(1)} RPM`;
    }
  }

  renderScope() {
    if (!this.scopeCtx || !this.scopeCanvas) return;
    const ctx = this.scopeCtx;
    const w = this.scopeCanvas.width;
    const h = this.scopeCanvas.height;
    const dpr = window.devicePixelRatio || 1;

    // Clear background
    ctx.clearRect(0, 0, w, h);
    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, "#080c18");
    grad.addColorStop(1, "#04060c");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    const leftPad = 86 * dpr;
    const rightPad = 16 * dpr;
    const topPad = 24 * dpr;
    const bottomPad = 24 * dpr;
    const plotW = w - leftPad - rightPad;
    const plotH = h - topPad - bottomPad;

    ctx.font = `600 ${10 * dpr}px 'JetBrains Mono', monospace`;
    ctx.textBaseline = "middle";

    if (this.scopePlotMode === "single") {
      // -------------------------------------------------------------
      // Mode 1: Single Turn Position (0 to 4095 ticks / 0° to 360°)
      // -------------------------------------------------------------
      const maxTicks = this.protocolVersion === "1.0" ? 1023 : 4095;
      const maxDeg = this.protocolVersion === "1.0" ? 300 : 360;
      const divs = 4;

      for (let i = 0; i <= divs; i++) {
        const norm = i / divs;
        const y = topPad + plotH * (1 - norm);
        const tickVal = Math.round(norm * maxTicks);
        const degVal = Math.round(norm * maxDeg);

        // Horizontal Grid Line
        ctx.beginPath();
        ctx.strokeStyle = (i === 0 || i === divs) ? "rgba(255, 255, 255, 0.12)" : "rgba(255, 255, 255, 0.05)";
        ctx.lineWidth = 1 * dpr;
        ctx.moveTo(leftPad, y);
        ctx.lineTo(w - rightPad, y);
        ctx.stroke();

        // Left Label
        ctx.fillStyle = "rgba(148, 163, 184, 0.75)";
        ctx.textAlign = "right";
        ctx.fillText(`${degVal}° (${tickVal})`, leftPad - 8 * dpr, y);
      }

      // Vertical Time Grid Lines
      for (let v = 1; v < 5; v++) {
        const x = leftPad + (plotW / 5) * v;
        ctx.beginPath();
        ctx.strokeStyle = "rgba(255, 255, 255, 0.04)";
        ctx.moveTo(x, topPad);
        ctx.lineTo(x, h - bottomPad);
        ctx.stroke();
      }

      if (this.scopeData.length >= 2) {
        // Draw Target series (Red dashed)
        this.renderPlotCurve(
          this.scopeData.map(d => d.singleTarget),
          0, maxTicks,
          leftPad, topPad, plotW, plotH, dpr,
          "#f43f5e", true
        );

        // Draw Actual series (Cyan glowing solid)
        this.renderPlotCurve(
          this.scopeData.map(d => d.singleActual),
          0, maxTicks,
          leftPad, topPad, plotW, plotH, dpr,
          "#38bdf8", false
        );
      }

    } else if (this.scopePlotMode === "multiturn") {
      // -------------------------------------------------------------
      // Mode 2: Multiturn Position (Revolutions: ±Revs)
      // -------------------------------------------------------------
      let maxAbsTurns = 1.5;
      this.scopeData.forEach(d => {
        maxAbsTurns = Math.max(maxAbsTurns, Math.abs(d.multiTargetTurns || 0), Math.abs(d.multiActualTurns || 0));
      });
      const yLimit = Math.max(2.0, Math.ceil(maxAbsTurns * 1.25));

      const divisions = [-yLimit, -yLimit / 2, 0, yLimit / 2, yLimit];
      divisions.forEach(val => {
        const norm = (val - (-yLimit)) / (2 * yLimit);
        const y = topPad + plotH * (1 - norm);

        // Grid line
        ctx.beginPath();
        if (val === 0) {
          ctx.strokeStyle = "rgba(192, 132, 252, 0.35)"; // Highlighted origin line
          ctx.lineWidth = 1.5 * dpr;
        } else {
          ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
          ctx.lineWidth = 1 * dpr;
        }
        ctx.moveTo(leftPad, y);
        ctx.lineTo(w - rightPad, y);
        ctx.stroke();

        // Label
        ctx.fillStyle = val === 0 ? "#c084fc" : "rgba(148, 163, 184, 0.75)";
        ctx.textAlign = "right";
        const sign = val > 0 ? "+" : "";
        const labelText = val === 0 ? "0 (Origin)" : `${sign}${val.toFixed(1)} Rev`;
        ctx.fillText(labelText, leftPad - 8 * dpr, y);
      });

      // Vertical Time Grid Lines
      for (let v = 1; v < 5; v++) {
        const x = leftPad + (plotW / 5) * v;
        ctx.beginPath();
        ctx.strokeStyle = "rgba(255, 255, 255, 0.04)";
        ctx.moveTo(x, topPad);
        ctx.lineTo(x, h - bottomPad);
        ctx.stroke();
      }

      if (this.scopeData.length >= 2) {
        // Target series (Red dashed)
        this.renderPlotCurve(
          this.scopeData.map(d => d.multiTargetTurns),
          -yLimit, yLimit,
          leftPad, topPad, plotW, plotH, dpr,
          "#f43f5e", true
        );

        // Actual series (Vivid Purple glowing solid)
        this.renderPlotCurve(
          this.scopeData.map(d => d.multiActualTurns),
          -yLimit, yLimit,
          leftPad, topPad, plotW, plotH, dpr,
          "#c084fc", false
        );
      }

    } else if (this.scopePlotMode === "speed") {
      // -------------------------------------------------------------
      // Mode 3: Speed / Velocity (RPM: ±RPM)
      // -------------------------------------------------------------
      let maxAbsSpeed = 25.0;
      this.scopeData.forEach(d => {
        maxAbsSpeed = Math.max(maxAbsSpeed, Math.abs(d.speedTarget || 0), Math.abs(d.speedActual || 0));
      });
      const yLimit = Math.max(30.0, Math.ceil(maxAbsSpeed / 10) * 10 + 10);

      const divisions = [-yLimit, -yLimit / 2, 0, yLimit / 2, yLimit];
      divisions.forEach(val => {
        const norm = (val - (-yLimit)) / (2 * yLimit);
        const y = topPad + plotH * (1 - norm);

        // Grid line
        ctx.beginPath();
        if (val === 0) {
          ctx.strokeStyle = "rgba(16, 185, 129, 0.35)"; // Highlighted stopped line
          ctx.lineWidth = 1.5 * dpr;
        } else {
          ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
          ctx.lineWidth = 1 * dpr;
        }
        ctx.moveTo(leftPad, y);
        ctx.lineTo(w - rightPad, y);
        ctx.stroke();

        // Label
        ctx.fillStyle = val === 0 ? "#10b981" : "rgba(148, 163, 184, 0.75)";
        ctx.textAlign = "right";
        const sign = val > 0 ? "+" : "";
        const labelText = val === 0 ? "0 RPM (Stop)" : `${sign}${val.toFixed(0)} RPM`;
        ctx.fillText(labelText, leftPad - 8 * dpr, y);
      });

      // Vertical Time Grid Lines
      for (let v = 1; v < 5; v++) {
        const x = leftPad + (plotW / 5) * v;
        ctx.beginPath();
        ctx.strokeStyle = "rgba(255, 255, 255, 0.04)";
        ctx.moveTo(x, topPad);
        ctx.lineTo(x, h - bottomPad);
        ctx.stroke();
      }

      if (this.scopeData.length >= 2) {
        // Target series (Amber dashed)
        this.renderPlotCurve(
          this.scopeData.map(d => d.speedTarget),
          -yLimit, yLimit,
          leftPad, topPad, plotW, plotH, dpr,
          "#f59e0b", true
        );

        // Actual series (Emerald green glowing solid)
        this.renderPlotCurve(
          this.scopeData.map(d => d.speedActual),
          -yLimit, yLimit,
          leftPad, topPad, plotW, plotH, dpr,
          "#10b981", false
        );
      }
    }

    this.scopeAnimFrame = requestAnimationFrame(() => this.renderScope());
  }

  renderPlotCurve(values, minVal, maxVal, leftPad, topPad, plotW, plotH, dpr, color, isDashed = false) {
    const ctx = this.scopeCtx;
    const n = values.length;
    if (n < 2) return;

    ctx.save();
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = (isDashed ? 2 : 2.5) * dpr;

    if (isDashed) {
      ctx.setLineDash([6 * dpr, 4 * dpr]);
    } else {
      ctx.setLineDash([]);
      ctx.shadowColor = color;
      ctx.shadowBlur = 8 * dpr;
    }

    const range = (maxVal - minVal) || 1;
    let lastX = 0;
    let lastY = 0;

    for (let i = 0; i < n; i++) {
      const x = leftPad + (i / (this.scopeMaxPoints - 1)) * plotW;
      const v = values[i] !== undefined && !isNaN(values[i]) ? values[i] : 0;
      const norm = Math.max(0, Math.min(1, (v - minVal) / range));
      const y = topPad + plotH * (1 - norm);

      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);

      lastX = x;
      lastY = y;
    }
    ctx.stroke();

    // Draw indicator dot on latest actual point
    if (!isDashed) {
      ctx.beginPath();
      ctx.fillStyle = color;
      ctx.arc(lastX, lastY, 4 * dpr, 0, Math.PI * 2);
      ctx.fill();
    }

    ctx.restore();
  }

  exportScopeCsv() {
    if (this.scopeData.length === 0) {
      alert("No scope data recorded yet.");
      return;
    }
    let csv = "Timestamp_s,SingleTurn_Target_Ticks,SingleTurn_Actual_Ticks,Multiturn_Target_Revs,Multiturn_Actual_Revs,Speed_Target_RPM,Speed_Actual_RPM,Load_Raw\n";
    this.scopeData.forEach(d => {
      csv += `${d.t.toFixed(3)},${d.singleTarget},${d.singleActual},${d.multiTargetTurns.toFixed(3)},${d.multiActualTurns.toFixed(3)},${d.speedTarget.toFixed(2)},${d.speedActual.toFixed(2)},${d.load}\n`;
    });
    const blob = new Blob([csv], { type: "text/csv" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `dynamixel_scope_${this.scopePlotMode}_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "_")}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }

  // --------------------------------------------------------------------------
  // Sine / Sweep Sequencer
  // --------------------------------------------------------------------------
  startSweep() {
    if (!this.isConnected) {
      alert("Please connect hardware first.");
      return;
    }
    if (!this.torqueEnabled) {
      this.el.toggleTorque.checked = true;
      this.setTorque(true);
    }

    const minAngle = parseFloat(this.el.sweepMinAngle.value);
    const maxAngle = parseFloat(this.el.sweepMaxAngle.value);
    const freq = parseFloat(this.el.sweepFreq.value);

    this.isSweeping = true;
    this.sweepStartTime = performance.now();
    this.el.btnStartSweep.disabled = true;
    this.el.btnStopSweep.disabled = false;

    const maxTicks = this.protocolVersion === "1.0" ? 1023 : 4095;
    const maxDeg = this.protocolVersion === "1.0" ? 300.0 : 360.0;

    const minTicks = (minAngle / maxDeg) * maxTicks;
    const maxTicksTarget = (maxAngle / maxDeg) * maxTicks;
    const center = (minTicks + maxTicksTarget) / 2.0;
    const amp = (maxTicksTarget - minTicks) / 2.0;

    this.sweepTimer = setInterval(() => {
      const elapsed = (performance.now() - this.sweepStartTime) / 1000.0;
      const targetTicks = Math.round(center + amp * Math.sin(2 * Math.PI * freq * elapsed));
      this.setGoalPosition(targetTicks);
    }, 50); // 20 Hz updates
  }

  stopSweep() {
    this.isSweeping = false;
    if (this.sweepTimer) {
      clearInterval(this.sweepTimer);
      this.sweepTimer = null;
    }
    this.el.btnStartSweep.disabled = false;
    this.el.btnStopSweep.disabled = true;
  }

  // --------------------------------------------------------------------------
  // Packet Inspector Logging
  // --------------------------------------------------------------------------
  logPacket(direction, bytes, desc = "") {
    const logBox = this.el.packetLogBox;
    if (!logBox) return;

    const timeStr = new Date().toTimeString().split(' ')[0] + '.' + performance.now().toFixed(0).slice(-3);
    const hexStr = bytes && bytes.length > 0 ? DXL.bytesToHex(bytes) : "";

    const line = document.createElement("div");
    line.className = "packet-line";

    let dirClass = "pkt-dir-tx";
    if (direction === "RX") dirClass = "pkt-dir-rx";
    else if (direction === "SYS") dirClass = "pkt-desc";

    line.innerHTML = `
      <span class="pkt-time">[${timeStr}]</span>
      <span class="${dirClass}">[${direction}]</span>
      <span class="pkt-bytes">${hexStr}</span>
      <span class="pkt-desc">${desc ? `// ${desc}` : ""}</span>
    `;

    logBox.appendChild(line);

    // Limit log lines to 200
    if (logBox.children.length > 200) {
      logBox.removeChild(logBox.firstElementChild);
    }
    logBox.scrollTop = logBox.scrollHeight;
  }

  // --------------------------------------------------------------------------
  // Event Listeners
  // --------------------------------------------------------------------------
  attachEventListeners() {
    this.el.btnRequestPort.addEventListener("click", () => this.requestNewPort());
    this.el.btnConnect.addEventListener("click", () => this.toggleConnection());
    this.el.baudSelect.addEventListener("change", () => {
      this.saveCurrentSettings();
    });
    this.el.protocolSelect.addEventListener("change", () => {
      this.protocolVersion = this.el.protocolSelect.value;
      this.el.activeProtocolBadge.textContent = `PROTOCOL ${this.protocolVersion.toUpperCase()}`;
      this.applyProtocolLimits();
      this.saveCurrentSettings();
    });

    // Scanner
    this.el.btnScanBus.addEventListener("click", () => this.scanBus());
    this.el.btnAddManualId.addEventListener("click", () => {
      const idStr = prompt("Enter Servo ID to add (1-253):", "1");
      if (idStr) {
        const id = parseInt(idStr, 10);
        if (!isNaN(id) && id >= 0 && id <= 253) {
          this.discoveredServos.set(id, { id, model: "Manual ID" });
          this.selectActiveServo(id);
        }
      }
    });

    // Operating Mode & Quick Arm
    if (this.el.btnSetOperatingMode) {
      this.el.btnSetOperatingMode.addEventListener("click", () => {
        const mode = this.el.operatingModeSelect.value;
        this.setOperatingMode(mode);
      });
    }
    if (this.el.btnQuickArmTorque) {
      this.el.btnQuickArmTorque.addEventListener("click", () => {
        this.setTorque(true);
      });
    }

    // Switches
    this.el.toggleTorque.addEventListener("change", e => this.setTorque(e.target.checked));
    this.el.toggleLed.addEventListener("change", e => this.setLed(e.target.checked));

    // Sliders
    this.el.sliderPosition.addEventListener("input", e => {
      const val = parseInt(e.target.value, 10);
      this.setGoalPosition(val);
    });

    this.el.sliderVelocity.addEventListener("input", e => {
      this.el.sliderVelReadout.textContent = `${e.target.value} ticks/s`;
    });

    this.el.sliderTorqueLimit.addEventListener("input", e => {
      this.el.sliderTorqueLimitReadout.textContent = `${e.target.value} ticks`;
    });

    // Quick Angles
    document.querySelectorAll(".quick-angle-btn").forEach(btn => {
      btn.addEventListener("click", () => {
        const angle = parseFloat(btn.dataset.angle);
        const maxAngle = this.protocolVersion === "1.0" ? 300.0 : 360.0;
        const maxTicks = this.protocolVersion === "1.0" ? 1023 : 4095;
        const ticks = Math.round((angle / maxAngle) * maxTicks);
        this.setGoalPosition(ticks);
      });
    });

    // Rotary Horn Drag Interaction
    const svg = this.el.rotarySvg;
    const handleDialEvent = e => {
      const rect = svg.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      const clientX = e.clientX || (e.touches && e.touches[0].clientX);
      const clientY = e.clientY || (e.touches && e.touches[0].clientY);

      let rad = Math.atan2(clientY - cy, clientX - cx) + Math.PI / 2;
      if (rad < 0) rad += 2 * Math.PI;

      let deg = (rad * 180) / Math.PI;
      const maxDeg = this.protocolVersion === "1.0" ? 300.0 : 360.0;
      deg = Math.min(maxDeg, Math.max(0, deg));

      const maxTicks = this.protocolVersion === "1.0" ? 1023 : 4095;
      const ticks = Math.round((deg / maxDeg) * maxTicks);
      this.setGoalPosition(ticks);
    };

    svg.addEventListener("mousedown", e => {
      this.isDialDragging = true;
      handleDialEvent(e);
    });

    window.addEventListener("mousemove", e => {
      if (this.isDialDragging) handleDialEvent(e);
    });

    window.addEventListener("mouseup", () => {
      this.isDialDragging = false;
    });

    // Touch events for mobile/tablet
    svg.addEventListener("touchstart", e => {
      this.isDialDragging = true;
      handleDialEvent(e);
    });
    window.addEventListener("touchmove", e => {
      if (this.isDialDragging) handleDialEvent(e);
    });
    window.addEventListener("touchend", () => {
      this.isDialDragging = false;
    });

    // Maintenance
    this.el.btnRebootServo.addEventListener("click", () => this.rebootServo());
    this.el.btnEmergencyStop.addEventListener("click", () => this.emergencyStop());

    // Tabs
    document.querySelectorAll(".tab-btn").forEach(btn => {
      btn.addEventListener("click", () => {
        document.querySelectorAll(".tab-btn").forEach(b => b.classList.remove("active"));
        document.querySelectorAll(".tab-content").forEach(c => c.classList.remove("active"));
        btn.classList.add("active");
        const targetTab = document.getElementById(btn.dataset.tab);
        if (targetTab) targetTab.classList.add("active");
      });
    });

    // Scope controls
    document.querySelectorAll(".scope-mode-btn").forEach(btn => {
      btn.addEventListener("click", () => {
        const mode = btn.dataset.scopeMode;
        this.setScopePlotMode(mode);
      });
    });

    if (this.el.scopeWindowSelect) {
      this.el.scopeWindowSelect.addEventListener("change", e => {
        this.scopeMaxPoints = parseInt(e.target.value, 10) || 200;
        if (this.el.scopePointsWatermark) {
          this.el.scopePointsWatermark.textContent = `${this.scopeMaxPoints} SAMPLES`;
        }
      });
    }

    this.el.btnScopePause.addEventListener("click", () => {
      this.isScopePaused = !this.isScopePaused;
      this.el.btnScopePause.textContent = this.isScopePaused ? "RESUME" : "PAUSE";
    });
    this.el.btnScopeClear.addEventListener("click", () => {
      this.scopeData = [];
    });
    this.el.btnScopeExportCsv.addEventListener("click", () => this.exportScopeCsv());
    this.el.pollRateSelect.addEventListener("change", () => this.updatePollingRate());

    // Sequencer
    this.el.sweepFreq.addEventListener("input", e => {
      this.el.sweepFreqReadout.textContent = `${parseFloat(e.target.value).toFixed(2)} Hz`;
    });
    this.el.btnStartSweep.addEventListener("click", () => this.startSweep());
    this.el.btnStopSweep.addEventListener("click", () => this.stopSweep());

    // Packet Inspector
    this.el.btnClearPackets.addEventListener("click", () => {
      this.el.packetLogBox.innerHTML = "";
    });

    this.el.btnSendCustomPkt.addEventListener("click", () => {
      const id = parseInt(this.el.pktIdInput.value, 10);
      const inst = parseInt(this.el.pktInstSelect.value, 10);
      const addr = parseInt(this.el.pktAddrInput.value, 10);
      const val = parseInt(this.el.pktValInput.value, 10);

      let packet;
      if (this.protocolVersion === "2.0") {
        if (inst === DXL.INST.WRITE) {
          packet = DXL.buildPacketP2(id, inst, [addr & 0xff, (addr >> 8) & 0xff, val & 0xff]);
        } else if (inst === DXL.INST.READ) {
          packet = DXL.buildPacketP2(id, inst, [addr & 0xff, (addr >> 8) & 0xff, 1, 0]);
        } else {
          packet = DXL.buildPacketP2(id, inst, []);
        }
      } else {
        if (inst === DXL.INST.WRITE) {
          packet = DXL.buildPacketP1(id, inst, [addr, val & 0xff]);
        } else if (inst === DXL.INST.READ) {
          packet = DXL.buildPacketP1(id, inst, [addr, 1]);
        } else {
          packet = DXL.buildPacketP1(id, inst, []);
        }
      }

      this.sendPacket(packet, `CUSTOM PACKET INST 0x${inst.toString(16)}`);
    });

    this.el.btnSendRawHex.addEventListener("click", () => {
      const hex = this.el.hexRawInput.value.trim();
      if (!hex) return;
      const bytes = DXL.hexToBytes(hex);
      if (!bytes) {
        alert("Invalid HEX string. Provide even number of hex characters (e.g. FF FF 01 04 03 18 01 DE).");
        return;
      }
      this.sendPacket(bytes, "RAW HEX TRANSMISSION");
    });

    // Dynamic Speed Controller
    if (this.el.sliderLiveVelocity) {
      this.el.sliderLiveVelocity.addEventListener("input", e => {
        const rpm = parseFloat(e.target.value);
        if (this.el.velLiveReadout) this.el.velLiveReadout.textContent = `${rpm.toFixed(1)} RPM`;
        this.setGoalVelocity(rpm);
      });
    }

    if (this.el.btnLiveStopVelocity) {
      this.el.btnLiveStopVelocity.addEventListener("click", () => {
        if (this.el.sliderLiveVelocity) this.el.sliderLiveVelocity.value = 0;
        if (this.el.velLiveReadout) this.el.velLiveReadout.textContent = "0.0 RPM";
        this.setGoalVelocity(0);
      });
    }

    document.querySelectorAll("[data-live-speed]").forEach(btn => {
      btn.addEventListener("click", () => {
        const spd = parseFloat(btn.dataset.liveSpeed);
        if (this.el.sliderLiveVelocity) this.el.sliderLiveVelocity.value = spd;
        if (this.el.velLiveReadout) this.el.velLiveReadout.textContent = `${spd.toFixed(1)} RPM`;
        this.setGoalVelocity(spd);
      });
    });

    // Dynamic Multiturn Controller
    if (this.el.btnApplyLiveTurns) {
      this.el.btnApplyLiveTurns.addEventListener("click", () => {
        const turns = parseFloat(this.el.inputLiveTurns.value) || 0;
        this.setMultiturnRevolutions(turns);
      });
    }

    if (this.el.btnZeroLiveTurns) {
      this.el.btnZeroLiveTurns.addEventListener("click", () => {
        if (this.el.inputLiveTurns) this.el.inputLiveTurns.value = 0;
        this.setMultiturnRevolutions(0);
      });
    }

    document.querySelectorAll("[data-live-turn]").forEach(btn => {
      btn.addEventListener("click", () => {
        const turn = parseFloat(btn.dataset.liveTurn);
        if (this.el.inputLiveTurns) this.el.inputLiveTurns.value = turn;
        this.setMultiturnRevolutions(turn);
      });
    });

    // Mode Tests Tab
    if (this.el.btnSwitchSpeedMode) {
      this.el.btnSwitchSpeedMode.addEventListener("click", () => {
        this.setOperatingMode(1);
      });
    }

    if (this.el.testSpeedSlider) {
      this.el.testSpeedSlider.addEventListener("input", e => {
        const rpm = parseFloat(e.target.value);
        if (this.el.testSpeedValReadout) this.el.testSpeedValReadout.textContent = `${rpm.toFixed(1)} RPM`;
        this.setGoalVelocity(rpm);
      });
    }

    if (this.el.btnTestStopSpeed) {
      this.el.btnTestStopSpeed.addEventListener("click", () => {
        if (this.el.testSpeedSlider) this.el.testSpeedSlider.value = 0;
        if (this.el.testSpeedValReadout) this.el.testSpeedValReadout.textContent = "0.0 RPM";
        this.setGoalVelocity(0);
      });
    }

    document.querySelectorAll("[data-speed]").forEach(btn => {
      btn.addEventListener("click", () => {
        const spd = parseFloat(btn.dataset.speed);
        if (this.el.testSpeedSlider) this.el.testSpeedSlider.value = spd;
        if (this.el.testSpeedValReadout) this.el.testSpeedValReadout.textContent = `${spd.toFixed(1)} RPM`;
        this.setGoalVelocity(spd);
      });
    });

    if (this.el.btnRampSpeedTest) {
      this.el.btnRampSpeedTest.addEventListener("click", () => this.startRampSpeedTest());
    }

    if (this.el.btnOscillateSpeedTest) {
      this.el.btnOscillateSpeedTest.addEventListener("click", () => this.startOscillateSpeedTest());
    }

    if (this.el.btnSwitchMultiturnMode) {
      this.el.btnSwitchMultiturnMode.addEventListener("click", () => {
        this.setOperatingMode(4);
      });
    }

    if (this.el.testTurnsInput) {
      this.el.testTurnsInput.addEventListener("input", e => {
        const turns = parseFloat(e.target.value) || 0;
        if (this.el.testTurnsTicksReadout) {
          this.el.testTurnsTicksReadout.value = `${Math.round(turns * 4096)} ticks`;
        }
      });
    }

    document.querySelectorAll("[data-turn]").forEach(btn => {
      btn.addEventListener("click", () => {
        const turn = parseFloat(btn.dataset.turn);
        if (this.el.testTurnsInput) {
          this.el.testTurnsInput.value = turn;
          if (this.el.testTurnsTicksReadout) {
            this.el.testTurnsTicksReadout.value = `${Math.round(turn * 4096)} ticks`;
          }
        }
        this.setMultiturnRevolutions(turn);
      });
    });

    if (this.el.btnExecuteMultiturnMove) {
      this.el.btnExecuteMultiturnMove.addEventListener("click", () => {
        const turns = parseFloat(this.el.testTurnsInput.value) || 0;
        this.setMultiturnRevolutions(turns);
      });
    }

    if (this.el.btnMultiturnSweepTest) {
      this.el.btnMultiturnSweepTest.addEventListener("click", () => this.startMultiturnSweepTest());
    }

    if (this.el.btnReturnToJointMode) {
      this.el.btnReturnToJointMode.addEventListener("click", () => {
        this.setOperatingMode(3);
      });
    }

    // Parameters Tab & Actions
    if (this.el.btnInspectParamsDirect) {
      this.el.btnInspectParamsDirect.addEventListener("click", () => {
        document.querySelectorAll(".tab-btn").forEach(b => b.classList.remove("active"));
        document.querySelectorAll(".tab-content").forEach(c => c.classList.remove("active"));
        const tabBtn = document.querySelector('[data-tab="tab-params"]');
        const tabContent = document.getElementById("tab-params");
        if (tabBtn) tabBtn.classList.add("active");
        if (tabContent) tabContent.classList.add("active");
        this.readAllMotorParameters();
      });
    }

    if (this.el.btnReadAllParams) {
      this.el.btnReadAllParams.addEventListener("click", () => this.readAllMotorParameters());
    }

    if (this.el.btnExportParamsJson) {
      this.el.btnExportParamsJson.addEventListener("click", () => this.exportParamsJson());
    }
  }
}

// Start application when DOM is ready
window.addEventListener("DOMContentLoaded", () => {
  window.dxlApp = new WebDynamixelApp();
});
