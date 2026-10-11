/**
 * Web Force Bench - Serial Monitor & Real-time Plotter
 * Handles Web Serial API connections, port selection & pairing,
 * USB vs Bluetooth filtering via checkboxes, configurable regex data extraction,
 * and smooth 60fps canvas plotting.
 */

// Well-known USB Vendor IDs for identification
const KNOWN_VENDORS = {
  0x2341: "Arduino LLC",
  0x2a03: "Arduino srl",
  0x10c4: "Silicon Labs (CP210x)",
  0x1a86: "QinHeng / WCH (CH340/CH341)",
  0x0403: "FTDI (Future Technology Devices)",
  0x2e8a: "Raspberry Pi (RP2040 / Pico)",
  0x303a: "Espressif Systems (ESP32-S2/S3/C3)",
  0x0483: "STMicroelectronics (STM32)",
  0x16c0: "Van Ooijen Technische Informatica (Teensy / CDC)",
  0x0d28: "ARM Ltd. (DAPLink / micro:bit)",
};

// Known common USB VID list for filter matching
const COMMON_USB_VIDS = [
  0x1a86, // WCH CH340 / CH341
  0x10c4, // Silicon Labs CP210x
  0x0403, // FTDI
  0x2341, // Arduino
  0x2a03, // Arduino
  0x2e8a, // Raspberry Pi Pico
  0x303a, // Espressif ESP32
  0x0483, // STM32
  0x16c0, // Teensy
  0x0d28, // micro:bit
  0x1366, // Segger J-Link
  0x04d8, // Microchip PIC
];

// Distinct color palette for real-time plot series
const CHANNEL_COLORS = [
  '#38bdf8', // Cyan
  '#f43f5e', // Rose
  '#10b981', // Emerald
  '#f59e0b', // Amber
  '#a855f7', // Purple
  '#06b6d4', // Teal
  '#ec4899', // Pink
  '#84cc16', // Lime
];

class SerialMonitorApp {
  constructor() {
    this.port = null;
    this.reader = null;
    this.writer = null;

    // Authorized ports list: Array of SerialPort
    this.availablePorts = [];
    this.selectedPortIndex = -1;

    this.isConnected = false;
    this.rxBytes = 0;
    this.txBytes = 0;
    this.packets = 0;
    this.plotPoints = 0;
    this.commandHistory = [];
    this.historyIndex = -1;

    // Incoming line buffer for clean newline-delimited stream decoding
    this.incomingBuffer = '';

    // Plotter data buffers: Map of channelIndex -> Array of numbers
    this.channelData = new Map();
    this.channelNames = new Map();
    this.maxPlotWindow = 100;
    this.isPlotPaused = false;

    // Initialize UI and listeners
    this.initElements();
    this.initCanvas();
    this.checkApiSupport();
    this.attachEventListeners();
    this.listenForDeviceEvents();

    // Populate authorized ports
    this.refreshPortList();

    // Start 60fps render loop for canvas plotter
    this.startPlotterLoop();
  }

  initElements() {
    // Badges & Status
    this.apiSupportBadge = document.getElementById('apiSupportBadge');
    this.apiSupportText = document.getElementById('apiSupportText');
    this.connectionStatusBadge = document.getElementById('connectionStatusBadge');
    this.connectionStatusText = document.getElementById('connectionStatusText');

    // Port Selection & Filter Checkboxes
    this.comPortSelect = document.getElementById('comPortSelect');
    this.btnPairNewPort = document.getElementById('btnPairNewPort');
    this.filterUsbCheck = document.getElementById('filterUsbCheck');
    this.filterBluetoothCheck = document.getElementById('filterBluetoothCheck');
    this.portListHint = document.getElementById('portListHint');

    // Controls
    this.baudRateSelect = document.getElementById('baudRateSelect');
    this.customBaudGroup = document.getElementById('customBaudGroup');
    this.customBaudInput = document.getElementById('customBaudInput');
    this.btnConnect = document.getElementById('btnConnect');
    this.btnDisconnect = document.getElementById('btnDisconnect');

    // Serial parameters
    this.dataBitsSelect = document.getElementById('dataBitsSelect');
    this.stopBitsSelect = document.getElementById('stopBitsSelect');
    this.paritySelect = document.getElementById('paritySelect');
    this.flowControlSelect = document.getElementById('flowControlSelect');

    // Device Info
    this.vidValue = document.getElementById('vidValue');
    this.pidValue = document.getElementById('pidValue');
    this.portTypeBadge = document.getElementById('portTypeBadge');
    this.detectedPortCategory = document.getElementById('detectedPortCategory');
    this.deviceGuess = document.getElementById('deviceGuess');
    this.portState = document.getElementById('portState');

    // Regex & Plotter controls
    this.regexPresetSelect = document.getElementById('regexPresetSelect');
    this.customRegexGroup = document.getElementById('customRegexGroup');
    this.customRegexInput = document.getElementById('customRegexInput');
    this.plotWindowSelect = document.getElementById('plotWindowSelect');
    this.plotYRangeSelect = document.getElementById('plotYRangeSelect');
    this.pausePlotCheck = document.getElementById('pausePlotCheck');
    this.btnClearPlot = document.getElementById('btnClearPlot');
    this.channelCountBadge = document.getElementById('channelCountBadge');
    this.latestValuesBadge = document.getElementById('latestValuesBadge');
    this.plotChannelsLegend = document.getElementById('plotChannelsLegend');

    // View Tabs
    this.viewsWrapper = document.getElementById('viewsWrapper');
    this.tabBtnPlotter = document.getElementById('tabBtnPlotter');
    this.tabBtnTerminal = document.getElementById('tabBtnTerminal');
    this.tabBtnSplit = document.getElementById('tabBtnSplit');

    // Terminal
    this.terminalContainer = document.getElementById('terminalContainer');
    this.serialInput = document.getElementById('serialInput');
    this.btnSend = document.getElementById('btnSend');
    this.lineEndingSelect = document.getElementById('lineEndingSelect');
    this.autoscrollCheck = document.getElementById('autoscrollCheck');
    this.timestampsCheck = document.getElementById('timestampsCheck');
    this.hexViewCheck = document.getElementById('hexViewCheck');
    this.btnClearTerminal = document.getElementById('btnClearTerminal');
    this.btnExportLog = document.getElementById('btnExportLog');

    // Stats
    this.rxBytesCount = document.getElementById('rxBytesCount');
    this.txBytesCount = document.getElementById('txBytesCount');
    this.plotPointsCount = document.getElementById('plotPointsCount');
    this.packetsCount = document.getElementById('packetsCount');

    // Canvas
    this.plotterCanvas = document.getElementById('plotterCanvas');
    this.ctx = this.plotterCanvas.getContext('2d');
  }

  initCanvas() {
    this.resizeCanvas();
    window.addEventListener('resize', () => this.resizeCanvas());
  }

  resizeCanvas() {
    const container = this.plotterCanvas.parentElement;
    if (!container) return;
    const rect = container.getBoundingClientRect();
    if (rect.width > 0 && rect.height > 0) {
      const dpr = window.devicePixelRatio || 1;
      this.plotterCanvas.width = rect.width * dpr;
      this.plotterCanvas.height = rect.height * dpr;
      this.ctx.resetTransform?.();
      this.ctx.scale(dpr, dpr);
      this.canvasWidth = rect.width;
      this.canvasHeight = rect.height;
    }
  }

  checkApiSupport() {
    if ('serial' in navigator) {
      this.apiSupportBadge.className = 'badge-status supported';
      this.apiSupportText.textContent = 'Web Serial Supported';
      this.logSystem('Web Serial API is available.');
    } else {
      this.apiSupportBadge.className = 'badge-status unsupported';
      this.apiSupportText.textContent = 'Web Serial Unsupported';
      this.btnPairNewPort.disabled = true;
      this.btnConnect.disabled = true;
      this.logError('Web Serial API is not supported. Use Google Chrome, Edge, or Opera on HTTPS/localhost.');
    }
  }

  attachEventListeners() {
    // Port selection change
    this.comPortSelect.addEventListener('change', () => {
      const val = this.comPortSelect.value;
      if (val !== '') {
        this.selectedPortIndex = parseInt(val, 10);
        this.port = this.availablePorts[this.selectedPortIndex];
        this.btnConnect.disabled = false;
        this.updateDeviceInfo(this.port);
      } else {
        this.selectedPortIndex = -1;
        this.port = null;
        this.btnConnect.disabled = true;
      }
    });

    // Pair/request new port button
    this.btnPairNewPort.addEventListener('click', () => this.handlePairNewPort());

    // Filter checkboxes (USB / Bluetooth)
    this.filterUsbCheck.addEventListener('change', () => this.refreshPortList());
    this.filterBluetoothCheck.addEventListener('change', () => this.refreshPortList());

    // Baud rate custom
    this.baudRateSelect.addEventListener('change', () => {
      if (this.baudRateSelect.value === 'custom') {
        this.customBaudGroup.classList.remove('hidden');
        this.customBaudInput.focus();
      } else {
        this.customBaudGroup.classList.add('hidden');
      }
    });

    // Connection buttons
    this.btnConnect.addEventListener('click', () => this.handleConnect());
    this.btnDisconnect.addEventListener('click', () => this.handleDisconnect());

    // Regex preset switcher
    this.regexPresetSelect.addEventListener('change', () => {
      const preset = this.regexPresetSelect.value;
      if (preset === 'numbers') {
        this.customRegexInput.value = '[-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?';
      } else if (preset === 'keyvalue') {
        this.customRegexInput.value = '(?:[a-zA-Z0-9_-]+)\\s*[:=]\\s*([-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?)';
      }
    });

    // Buffer window size
    this.plotWindowSelect.addEventListener('change', () => {
      this.maxPlotWindow = parseInt(this.plotWindowSelect.value, 10);
    });

    // Pause plot
    this.pausePlotCheck.addEventListener('change', () => {
      this.isPlotPaused = this.pausePlotCheck.checked;
    });

    // Clear plot
    this.btnClearPlot.addEventListener('click', () => {
      this.channelData.clear();
      this.channelNames.clear();
      this.plotPoints = 0;
      this.updateLegend();
      this.updateStats();
    });

    // View tab switching
    this.tabBtnPlotter.addEventListener('click', () => this.switchView('mode-plotter', this.tabBtnPlotter));
    this.tabBtnTerminal.addEventListener('click', () => this.switchView('mode-terminal', this.tabBtnTerminal));
    this.tabBtnSplit.addEventListener('click', () => this.switchView('mode-split', this.tabBtnSplit));

    // Terminal command send
    this.btnSend.addEventListener('click', () => this.sendCurrentInput());
    this.serialInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        e.preventDefault();
        this.sendCurrentInput();
      } else if (e.key === 'ArrowUp') {
        this.navigateHistory(1);
      } else if (e.key === 'ArrowDown') {
        this.navigateHistory(-1);
      }
    });

    this.btnClearTerminal.addEventListener('click', () => {
      this.terminalContainer.innerHTML = '';
      this.logSystem('Terminal cleared.');
    });

    this.btnExportLog.addEventListener('click', () => this.exportTerminalLogs());
  }

  async refreshPortList() {
    if (!('serial' in navigator)) return;

    try {
      // Retrieve already granted ports
      const authorizedPorts = await navigator.serial.getPorts();
      this.availablePorts = authorizedPorts;

      const showUsb = this.filterUsbCheck.checked;
      const showBt = this.filterBluetoothCheck.checked;

      this.comPortSelect.innerHTML = '';

      let matchingCount = 0;
      const currentSelected = this.port;

      authorizedPorts.forEach((p, idx) => {
        const info = p.getInfo ? p.getInfo() : {};
        const isUsb = info.usbVendorId !== undefined && info.usbProductId !== undefined;
        const isBt = !isUsb;

        // Apply checkbox filters
        if ((isUsb && !showUsb) || (isBt && !showBt)) {
          return;
        }

        matchingCount++;
        const option = document.createElement('option');
        option.value = idx;

        if (isUsb) {
          const vidHex = '0x' + info.usbVendorId.toString(16).toUpperCase().padStart(4, '0');
          const pidHex = '0x' + info.usbProductId.toString(16).toUpperCase().padStart(4, '0');
          const vendor = KNOWN_VENDORS[info.usbVendorId] || 'USB Serial';
          option.textContent = `[USB] ${vendor} (${vidHex}:${pidHex})`;
        } else {
          option.textContent = `[Bluetooth] Standard Serial Link #${idx + 1}`;
        }

        if (currentSelected === p) {
          option.selected = true;
        }

        this.comPortSelect.appendChild(option);
      });

      if (matchingCount === 0) {
        const opt = document.createElement('option');
        opt.value = '';
        opt.disabled = true;
        opt.selected = true;
        if (authorizedPorts.length === 0) {
          opt.textContent = 'No paired ports (Click "Pair" to add)';
          this.portListHint.textContent = 'Click "Pair" to grant access to a COM port.';
        } else {
          opt.textContent = 'No ports match current checkbox filters';
          this.portListHint.textContent = 'Check USB or Bluetooth boxes to view paired ports.';
        }
        this.comPortSelect.appendChild(opt);
        this.btnConnect.disabled = true;
      } else {
        this.portListHint.textContent = `${matchingCount} port(s) available. Select one and click Connect.`;
        if (this.comPortSelect.value !== '') {
          this.selectedPortIndex = parseInt(this.comPortSelect.value, 10);
          this.port = this.availablePorts[this.selectedPortIndex];
          if (!this.isConnected) {
            this.btnConnect.disabled = false;
            this.updateDeviceInfo(this.port);
          }
        }
      }
    } catch (err) {
      console.error('Error refreshing port list:', err);
    }
  }

  async handlePairNewPort() {
    if (!('serial' in navigator)) return;

    try {
      const showUsb = this.filterUsbCheck.checked;
      const showBt = this.filterBluetoothCheck.checked;
      const requestOptions = {};

      // If user checked USB only and unchecked Bluetooth, apply USB filter to browser picker
      if (showUsb && !showBt) {
        requestOptions.filters = COMMON_USB_VIDS.map(vid => ({ usbVendorId: vid }));
        this.logSystem('Opening browser picker with USB hardware filters...');
      } else {
        this.logSystem('Opening browser port selector prompt...');
      }

      const newlyPairedPort = await navigator.serial.requestPort(requestOptions);
      this.logSystem('Port permission granted.');

      // Refresh list and select this newly paired port
      await this.refreshPortList();

      const newIdx = this.availablePorts.indexOf(newlyPairedPort);
      if (newIdx >= 0) {
        this.comPortSelect.value = newIdx;
        this.selectedPortIndex = newIdx;
        this.port = newlyPairedPort;
        this.btnConnect.disabled = false;
        this.updateDeviceInfo(this.port);
      }
    } catch (err) {
      if (err.name === 'NotFoundError') {
        this.logSystem('Port selection cancelled by user.');
      } else {
        this.logError(`Pairing error: ${err.message}`);
      }
    }
  }

  switchView(modeClass, activeBtn) {
    this.viewsWrapper.className = `views-wrapper ${modeClass}`;
    [this.tabBtnPlotter, this.tabBtnTerminal, this.tabBtnSplit].forEach(btn => btn.classList.remove('active'));
    activeBtn.classList.add('active');
    setTimeout(() => this.resizeCanvas(), 50);
  }

  listenForDeviceEvents() {
    if (!('serial' in navigator)) return;

    navigator.serial.addEventListener('connect', (event) => {
      this.logSystem('Hardware device plugged in.');
      this.refreshPortList();
    });

    navigator.serial.addEventListener('disconnect', (event) => {
      this.logSystem('Hardware device unplugged.');
      if (this.port === event.port) {
        this.handleDisconnect(true);
      }
      this.refreshPortList();
    });
  }

  getBaudRate() {
    if (this.baudRateSelect.value === 'custom') {
      const customVal = parseInt(this.customBaudInput.value, 10);
      return customVal && customVal > 0 ? customVal : 115200;
    }
    return parseInt(this.baudRateSelect.value, 10);
  }

  async handleConnect() {
    if (!this.port) {
      this.logError('Please select a port from the list first (or click Pair to add one).');
      return;
    }

    try {
      this.updateDeviceInfo(this.port);

      const baudRate = this.getBaudRate();
      const dataBits = parseInt(this.dataBitsSelect.value, 10);
      const stopBits = parseInt(this.stopBitsSelect.value, 10);
      const parity = this.paritySelect.value;
      const flowControl = this.flowControlSelect.value;

      this.logSystem(`Opening port at ${baudRate} baud (${dataBits}-${parity.toUpperCase()[0]}-${stopBits})...`);

      await this.port.open({
        baudRate,
        dataBits,
        stopBits,
        parity,
        flowControl,
        bufferSize: 16384
      });

      this.setConnectedState(true);
      this.portState.textContent = 'Open';
      this.logSystem(`Connected to port successfully at ${baudRate} baud.`);

      // Reset incoming stream buffer
      this.incomingBuffer = '';

      // Start reading loop
      this.startReadingLoop();
    } catch (err) {
      this.logError(`Failed to open port: ${err.message}`);
    }
  }

  updateDeviceInfo(port) {
    if (!port || !port.getInfo) return;
    const info = port.getInfo();

    const vid = info.usbVendorId;
    const pid = info.usbProductId;

    if (vid !== undefined && pid !== undefined) {
      const vidHex = '0x' + vid.toString(16).toUpperCase().padStart(4, '0');
      const pidHex = '0x' + pid.toString(16).toUpperCase().padStart(4, '0');
      this.vidValue.textContent = vidHex;
      this.pidValue.textContent = pidHex;

      const vendorName = KNOWN_VENDORS[vid] || 'USB Hardware Serial';
      this.deviceGuess.textContent = vendorName;

      // Identify as USB Hardware Serial Port
      this.detectedPortCategory.textContent = 'USB Serial Hardware';
      this.portTypeBadge.className = 'info-tag tag-usb';
      this.portTypeBadge.textContent = 'USB Port';
    } else {
      // Ports without USB VID/PID (e.g. Standard Serial over Bluetooth link COM3/5/6)
      this.vidValue.textContent = 'None';
      this.pidValue.textContent = 'None';
      this.deviceGuess.textContent = 'Bluetooth RFCOMM / Emulated';
      this.detectedPortCategory.textContent = 'Bluetooth Serial Link';
      this.portTypeBadge.className = 'info-tag tag-bt';
      this.portTypeBadge.textContent = 'Bluetooth Port';
    }
  }

  setConnectedState(connected) {
    this.isConnected = connected;

    if (connected) {
      this.connectionStatusBadge.className = 'badge-status connected';
      this.connectionStatusText.textContent = 'Connected';
      this.comPortSelect.disabled = true;
      this.btnPairNewPort.disabled = true;
      this.filterUsbCheck.disabled = true;
      this.filterBluetoothCheck.disabled = true;
      this.btnConnect.disabled = true;
      this.btnDisconnect.disabled = false;
      this.serialInput.disabled = false;
      this.btnSend.disabled = false;
      this.baudRateSelect.disabled = true;
      this.customBaudInput.disabled = true;
      this.dataBitsSelect.disabled = true;
      this.stopBitsSelect.disabled = true;
      this.paritySelect.disabled = true;
      this.flowControlSelect.disabled = true;
      this.serialInput.focus();
    } else {
      this.connectionStatusBadge.className = 'badge-status disconnected';
      this.connectionStatusText.textContent = 'Disconnected';
      this.comPortSelect.disabled = false;
      this.btnPairNewPort.disabled = false;
      this.filterUsbCheck.disabled = false;
      this.filterBluetoothCheck.disabled = false;
      this.btnConnect.disabled = this.selectedPortIndex < 0;
      this.btnDisconnect.disabled = true;
      this.serialInput.disabled = true;
      this.btnSend.disabled = true;
      this.baudRateSelect.disabled = false;
      this.customBaudInput.disabled = false;
      this.dataBitsSelect.disabled = false;
      this.stopBitsSelect.disabled = false;
      this.paritySelect.disabled = false;
      this.flowControlSelect.disabled = false;
      this.portState.textContent = 'Closed';
    }
  }

  async startReadingLoop() {
    const textDecoder = new TextDecoder();

    while (this.port && this.port.readable && this.isConnected) {
      try {
        this.reader = this.port.readable.getReader();
        while (true) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value) {
            this.handleIncomingChunk(value, textDecoder);
          }
        }
      } catch (err) {
        this.logError(`Serial Read Error: ${err.message}`);
        break;
      } finally {
        if (this.reader) {
          try { this.reader.releaseLock(); } catch (e) {}
          this.reader = null;
        }
      }
    }

    if (this.isConnected) {
      this.handleDisconnect();
    }
  }

  handleIncomingChunk(uint8Array, decoder) {
    this.rxBytes += uint8Array.byteLength;
    this.packets += 1;
    this.updateStats();

    if (this.hexViewCheck.checked) {
      const hexStr = Array.from(uint8Array)
        .map(b => b.toString(16).toUpperCase().padStart(2, '0'))
        .join(' ');
      this.appendTerminalLine('RX', hexStr, 'line-rx');
      return;
    }

    // Decode incoming bytes to UTF-8
    const chunkText = decoder.decode(uint8Array, { stream: true });
    this.incomingBuffer += chunkText;

    // Process line-by-line when newline is encountered
    let newlineIndex;
    while ((newlineIndex = this.incomingBuffer.indexOf('\n')) >= 0) {
      const line = this.incomingBuffer.slice(0, newlineIndex).replace(/\r$/, '');
      this.incomingBuffer = this.incomingBuffer.slice(newlineIndex + 1);

      if (line.length > 0) {
        // Output to terminal
        this.appendTerminalLine('RX', line, 'line-rx');

        // Extract plot data via Regex parser
        if (!this.isPlotPaused) {
          this.parseLineForPlotter(line);
        }
      }
    }
  }

  parseLineForPlotter(line) {
    let regex;
    try {
      regex = new RegExp(this.customRegexInput.value, 'g');
    } catch (e) {
      regex = /[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?/g;
    }

    const matches = [];
    let match;

    // Check if regex has capture groups (like key-value regex)
    while ((match = regex.exec(line)) !== null) {
      // Use first capture group if exists, else full match
      const valStr = match[1] !== undefined ? match[1] : match[0];
      const num = parseFloat(valStr);
      if (!isNaN(num) && isFinite(num)) {
        matches.push(num);
      }
    }

    if (matches.length > 0) {
      this.plotPoints += matches.length;
      let hasNewChannel = false;

      matches.forEach((val, index) => {
        if (!this.channelData.has(index)) {
          this.channelData.set(index, []);
          this.channelNames.set(index, `CH${index + 1}`);
          hasNewChannel = true;
        }
        const series = this.channelData.get(index);
        series.push(val);

        // Keep buffer trimmed to window size
        if (series.length > this.maxPlotWindow) {
          series.shift();
        }
      });

      if (hasNewChannel) {
        this.updateLegend();
      }

      this.channelCountBadge.textContent = this.channelData.size;
      this.latestValuesBadge.textContent = matches.map((v, i) => `CH${i + 1}: ${v}`).join('  |  ');
      this.updateStats();
    }
  }

  updateLegend() {
    this.plotChannelsLegend.innerHTML = '';
    this.channelData.forEach((_, index) => {
      const chip = document.createElement('div');
      chip.className = 'legend-chip';
      const color = CHANNEL_COLORS[index % CHANNEL_COLORS.length];

      chip.innerHTML = `
        <span class="legend-color" style="background: ${color}; box-shadow: 0 0 6px ${color};"></span>
        <span style="color: ${color}; font-weight: 600;">${this.channelNames.get(index)}</span>
      `;
      this.plotChannelsLegend.appendChild(chip);
    });
  }

  startPlotterLoop() {
    const render = () => {
      this.renderPlot();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  renderPlot() {
    if (!this.ctx || !this.canvasWidth || !this.canvasHeight) return;

    const ctx = this.ctx;
    const width = this.canvasWidth;
    const height = this.canvasHeight;
    const padding = { top: 20, right: 25, bottom: 35, left: 55 };
    const plotW = width - padding.left - padding.right;
    const plotH = height - padding.top - padding.bottom;

    // Clear background
    ctx.fillStyle = '#06090e';
    ctx.fillRect(0, 0, width, height);

    if (plotW <= 0 || plotH <= 0) return;

    // Determine Y min & max
    let minY = Infinity;
    let maxY = -Infinity;

    const scaleMode = this.plotYRangeSelect.value;
    if (scaleMode === 'fixed_100') {
      minY = 0;
      maxY = 100;
    } else if (scaleMode === 'fixed_1000') {
      minY = 0;
      maxY = 1000;
    } else if (scaleMode === 'fixed_5000') {
      minY = 0;
      maxY = 5000;
    } else {
      // Auto-scaling across all current series
      this.channelData.forEach(series => {
        for (let i = 0; i < series.length; i++) {
          const val = series[i];
          if (val < minY) minY = val;
          if (val > maxY) maxY = val;
        }
      });

      if (!isFinite(minY) || !isFinite(maxY)) {
        minY = 0;
        maxY = 100;
      } else if (minY === maxY) {
        minY -= 1;
        maxY += 1;
      } else {
        // Add 10% headroom
        const span = maxY - minY;
        minY -= span * 0.08;
        maxY += span * 0.08;
      }
    }

    const yRange = maxY - minY || 1;

    // Draw Grid Lines (Horizontal)
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.07)';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#64748b';
    ctx.font = '11px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const ySteps = 5;
    for (let i = 0; i <= ySteps; i++) {
      const ratio = i / ySteps;
      const yVal = maxY - ratio * yRange;
      const yPos = padding.top + ratio * plotH;

      ctx.beginPath();
      ctx.moveTo(padding.left, yPos);
      ctx.lineTo(padding.left + plotW, yPos);
      ctx.stroke();

      ctx.fillText(yVal.toFixed(1), padding.left - 8, yPos);
    }

    // Draw Grid Lines (Vertical)
    const xSteps = 5;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let i = 0; i <= xSteps; i++) {
      const xPos = padding.left + (i / xSteps) * plotW;
      ctx.beginPath();
      ctx.moveTo(xPos, padding.top);
      ctx.lineTo(xPos, padding.top + plotH);
      ctx.stroke();
    }

    // Outer Plot Border
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
    ctx.strokeRect(padding.left, padding.top, plotW, plotH);

    // Render Data Series Lines
    this.channelData.forEach((series, channelIndex) => {
      if (series.length < 2) return;

      const color = CHANNEL_COLORS[channelIndex % CHANNEL_COLORS.length];
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.2;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';

      ctx.beginPath();
      const stepX = plotW / (this.maxPlotWindow - 1);
      const startOffset = (this.maxPlotWindow - series.length) * stepX;

      series.forEach((val, i) => {
        const x = padding.left + startOffset + i * stepX;
        const normalizedY = (val - minY) / yRange;
        const y = padding.top + plotH - normalizedY * plotH;

        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();

      // Draw glowing head dot on latest point
      const lastIndex = series.length - 1;
      const lastVal = series[lastIndex];
      const lastX = padding.left + startOffset + lastIndex * stepX;
      const lastNormY = (lastVal - minY) / yRange;
      const lastY = padding.top + plotH - lastNormY * plotH;

      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(lastX, lastY, 4, 0, Math.PI * 2);
      ctx.fill();
    });

    // If no data points yet, draw clean placeholder hint
    if (this.channelData.size === 0) {
      ctx.fillStyle = '#475569';
      ctx.font = '13px "Outfit", sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Awaiting serial numeric telemetry stream to plot...', width / 2, height / 2);
    }
  }

  async sendCurrentInput() {
    const rawVal = this.serialInput.value;
    if (!rawVal && rawVal !== '') return;

    if (!this.isConnected || !this.port || !this.port.writable) {
      this.logError('Cannot send: port is not open.');
      return;
    }

    const lineEnding = this.lineEndingSelect.value.replace(/\\r/g, '\r').replace(/\\n/g, '\n');
    const fullPayload = rawVal + lineEnding;

    try {
      const encoder = new TextEncoder();
      const data = encoder.encode(fullPayload);

      this.writer = this.port.writable.getWriter();
      await this.writer.write(data);
      this.writer.releaseLock();
      this.writer = null;

      this.txBytes += data.byteLength;
      this.updateStats();

      this.appendTerminalLine('TX', rawVal, 'line-tx');

      if (rawVal.trim()) {
        this.commandHistory.push(rawVal);
        this.historyIndex = this.commandHistory.length;
      }

      this.serialInput.value = '';
    } catch (err) {
      this.logError(`Serial Write Error: ${err.message}`);
      if (this.writer) {
        try { this.writer.releaseLock(); } catch (e) {}
        this.writer = null;
      }
    }
  }

  navigateHistory(direction) {
    if (this.commandHistory.length === 0) return;

    if (direction === 1) { // Up
      if (this.historyIndex > 0) {
        this.historyIndex--;
        this.serialInput.value = this.commandHistory[this.historyIndex];
      }
    } else if (direction === -1) { // Down
      if (this.historyIndex < this.commandHistory.length - 1) {
        this.historyIndex++;
        this.serialInput.value = this.commandHistory[this.historyIndex];
      } else {
        this.historyIndex = this.commandHistory.length;
        this.serialInput.value = '';
      }
    }
  }

  async handleDisconnect(wasForced = false) {
    if (!this.isConnected && !this.port) return;

    this.setConnectedState(false);

    try {
      if (this.reader) {
        await this.reader.cancel();
        try { this.reader.releaseLock(); } catch (e) {}
        this.reader = null;
      }
      if (this.writer) {
        try { this.writer.releaseLock(); } catch (e) {}
        this.writer = null;
      }
      if (this.port) {
        await this.port.close();
      }
      this.logSystem(wasForced ? 'Device disconnected unexpectedly.' : 'Disconnected from port.');
    } catch (err) {
      this.logError(`Error during disconnection: ${err.message}`);
    }
  }

  appendTerminalLine(type, text, className) {
    const line = document.createElement('div');
    line.className = `terminal-line ${className}`;

    if (this.timestampsCheck.checked) {
      const tsSpan = document.createElement('span');
      tsSpan.className = 'terminal-ts';
      const now = new Date();
      tsSpan.textContent = now.toTimeString().split(' ')[0] + '.' + String(now.getMilliseconds()).padStart(3, '0');
      line.appendChild(tsSpan);
    }

    const typeSpan = document.createElement('span');
    typeSpan.className = 'terminal-type';
    typeSpan.textContent = type;
    line.appendChild(typeSpan);

    const msgSpan = document.createElement('span');
    msgSpan.className = 'terminal-msg';
    msgSpan.textContent = text;
    line.appendChild(msgSpan);

    this.terminalContainer.appendChild(line);

    if (this.autoscrollCheck.checked) {
      this.terminalContainer.scrollTop = this.terminalContainer.scrollHeight;
    }
  }

  logSystem(text) {
    this.appendTerminalLine('SYS', text, 'line-sys');
  }

  logError(text) {
    this.appendTerminalLine('ERR', text, 'line-err');
  }

  formatBytes(bytes) {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
  }

  updateStats() {
    this.rxBytesCount.textContent = this.formatBytes(this.rxBytes);
    this.txBytesCount.textContent = this.formatBytes(this.txBytes);
    this.plotPointsCount.textContent = this.plotPoints.toLocaleString();
    this.packetsCount.textContent = this.packets.toLocaleString();
  }

  exportTerminalLogs() {
    const textLines = [];
    const lines = this.terminalContainer.querySelectorAll('.terminal-line');
    lines.forEach(l => {
      textLines.push(l.textContent);
    });

    const blob = new Blob([textLines.join('\n')], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `serial-log-${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`;
    a.click();
    URL.revokeObjectURL(url);
  }
}

// Instantiate once DOM content is ready
window.addEventListener('DOMContentLoaded', () => {
  new SerialMonitorApp();
});
