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
    this.scopeData = []; // { t, target, actual, load }
    this.scopeMaxPoints = 200;
    this.isScopePaused = false;
    this.scopeAnimFrame = null;
    this.startTime = performance.now();

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
    };
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
    // If response from reading Present Position & Telemetry block
    if (params.length >= 4) {
      this.presentPosition = DXL.bytesToInt(params.slice(0, 4), false);
      const angleDeg = (this.presentPosition / 4095.0) * 360.0;
      this.el.hudPosVal.textContent = this.presentPosition;
      this.el.hudPosDeg.textContent = `${angleDeg.toFixed(1)}°`;

      // Update arm on SVG dial
      this.el.svgPresentArm.setAttribute("transform", `rotate(${angleDeg} 130 130)`);

      if (params.length >= 8) {
        this.presentVelocity = DXL.bytesToInt(params.slice(4, 8), true);
        this.el.hudVelVal.textContent = (this.presentVelocity * 0.229).toFixed(1); // 0.229 RPM/unit
      }

      if (params.length >= 10) {
        this.presentLoad = DXL.bytesToInt(params.slice(8, 10), true);
        this.el.hudLoadVal.textContent = `${(this.presentLoad * 2.69).toFixed(1)} mA`; // 2.69 mA/unit
      }

      // Record point for Scope
      this.recordScopePoint(this.goalPosition, this.presentPosition, this.presentLoad);
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

      this.recordScopePoint(this.goalPosition, this.presentPosition, this.presentLoad);
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
  }

  // --------------------------------------------------------------------------
  // Actuator Control Operations
  // --------------------------------------------------------------------------
  async setTorque(enable) {
    this.torqueEnabled = enable;
    this.el.torqueStateLabel.textContent = enable ? "Motor Armed (Torque ON)" : "Motor Free (Torque OFF)";

    const addr = this.protocolVersion === "2.0" ? 64 : 24;
    const packet = this.protocolVersion === "2.0"
      ? DXL.buildPacketP2(this.activeId, DXL.INST.WRITE, [addr & 0xff, (addr >> 8) & 0xff, enable ? 1 : 0])
      : DXL.buildPacketP1(this.activeId, DXL.INST.WRITE, [addr, enable ? 1 : 0]);

    await this.sendPacket(packet, `TORQUE ENABLE = ${enable ? 1 : 0} (ID ${this.activeId})`);
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
    this.updateDialDisplay(ticks);

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
        // Read 10 bytes starting from Present Position (132) or Velocity/Position block
        const addr = 132; // Present Position (4 bytes)
        const length = 4;
        const packet = DXL.buildPacketP2(this.activeId, DXL.INST.READ, [
          addr & 0xff, (addr >> 8) & 0xff,
          length & 0xff, (length >> 8) & 0xff
        ]);
        await this.sendPacket(packet, `READ PRESENT POS (ID ${this.activeId})`);
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
    this.scopeCtx = this.scopeCanvas.getContext('2d');
    this.resizeCanvas();
    window.addEventListener('resize', () => this.resizeCanvas());
    this.renderScope();
  }

  resizeCanvas() {
    if (!this.scopeCanvas) return;
    const rect = this.scopeCanvas.getBoundingClientRect();
    this.scopeCanvas.width = rect.width * window.devicePixelRatio;
    this.scopeCanvas.height = rect.height * window.devicePixelRatio;
  }

  recordScopePoint(target, actual, load) {
    if (this.isScopePaused) return;
    const t = (performance.now() - this.startTime) / 1000.0;
    this.scopeData.push({ t, target, actual, load });
    if (this.scopeData.length > this.scopeMaxPoints) {
      this.scopeData.shift();
    }
  }

  renderScope() {
    const ctx = this.scopeCtx;
    const w = this.scopeCanvas.width;
    const h = this.scopeCanvas.height;

    ctx.clearRect(0, 0, w, h);

    // Draw Grid
    ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
    ctx.lineWidth = 1;
    const gridY = 5;
    for (let i = 0; i <= gridY; i++) {
      const y = (h / gridY) * i;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    if (this.scopeData.length >= 2) {
      const maxTicks = this.protocolVersion === "1.0" ? 1023 : 4095;

      // Draw Target (Red dashed line)
      this.drawScopeSeries(this.scopeData.map(d => d.target), maxTicks, "#f43f5e", true);
      // Draw Actual (Cyan solid line)
      this.drawScopeSeries(this.scopeData.map(d => d.actual), maxTicks, "#38bdf8", false);
    }

    this.scopeAnimFrame = requestAnimationFrame(() => this.renderScope());
  }

  drawScopeSeries(values, maxVal, color, isDashed = false) {
    const ctx = this.scopeCtx;
    const w = this.scopeCanvas.width;
    const h = this.scopeCanvas.height;
    const pad = 20 * window.devicePixelRatio;

    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2 * window.devicePixelRatio;
    if (isDashed) {
      ctx.setLineDash([6 * window.devicePixelRatio, 4 * window.devicePixelRatio]);
    } else {
      ctx.setLineDash([]);
    }

    const n = values.length;
    for (let i = 0; i < n; i++) {
      const x = (i / (this.scopeMaxPoints - 1)) * w;
      const norm = Math.max(0, Math.min(1, values[i] / maxVal));
      const y = h - pad - norm * (h - pad * 2);

      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  exportScopeCsv() {
    if (this.scopeData.length === 0) {
      alert("No scope data recorded yet.");
      return;
    }
    let csv = "Time_s,Target_Ticks,Actual_Ticks,Load\n";
    this.scopeData.forEach(d => {
      csv += `${d.t.toFixed(3)},${d.target},${d.actual},${d.load}\n`;
    });
    const blob = new Blob([csv], { type: "text/csv" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `dynamixel_scope_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "_")}.csv`;
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
    this.el.protocolSelect.addEventListener("change", () => {
      this.protocolVersion = this.el.protocolSelect.value;
      this.el.activeProtocolBadge.textContent = `PROTOCOL ${this.protocolVersion.toUpperCase()}`;
      this.applyProtocolLimits();
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
  }
}

// Start application when DOM is ready
window.addEventListener("DOMContentLoaded", () => {
  window.dxlApp = new WebDynamixelApp();
});
