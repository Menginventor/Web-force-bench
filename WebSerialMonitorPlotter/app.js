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

    // Plotter data buffers: Map of channelKey -> Array of numbers
    this.channelOrder = [];
    this.channelData = new Map();
    this.channelNames = new Map();
    this.channelColors = new Map();
    this.customChannelNames = new Set();
    this.channelVisibility = new Map();
    this.latestChannelValues = new Map();
    this.maxPlotWindow = 100;
    this.isPlotPaused = false;

    // Split Y-Axis (Dual Axis) & CSV Recording State
    this.isSplitYAxis = false;
    this.isRecordingCsv = false;
    this.csvRecordedRows = [];
    this.csvRecordingStartTime = null;

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

    // Split Y-Axis & CSV Recording Controls
    this.splitYAxisWrapper = document.getElementById('splitYAxisWrapper');
    this.splitYAxisCheck = document.getElementById('splitYAxisCheck');
    this.splitYAxisLabel = document.getElementById('splitYAxisLabel');
    this.btnRecordCsv = document.getElementById('btnRecordCsv');
    this.btnRecordCsvText = document.getElementById('btnRecordCsvText');
    this.btnExportPlotCsv = document.getElementById('btnExportPlotCsv');

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
    this.filterPlotInTerminalCheck = document.getElementById('filterPlotInTerminalCheck');
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

    // Restore previously used connection settings (baud rate)
    this.restoreSavedSettings();
  }

  restoreSavedSettings() {
    try {
      const savedBaud = localStorage.getItem('serial_monitor_baud_rate');
      const savedCustom = localStorage.getItem('serial_monitor_custom_baud');

      if (savedCustom && this.customBaudInput) {
        this.customBaudInput.value = savedCustom;
      }

      if (savedBaud && this.baudRateSelect) {
        if (savedBaud === 'custom') {
          this.baudRateSelect.value = 'custom';
          if (this.customBaudGroup) {
            this.customBaudGroup.classList.remove('hidden');
          }
        } else {
          let exists = false;
          for (let i = 0; i < this.baudRateSelect.options.length; i++) {
            if (this.baudRateSelect.options[i].value === savedBaud) {
              exists = true;
              break;
            }
          }
          if (!exists) {
            const opt = document.createElement('option');
            opt.value = savedBaud;
            opt.textContent = `${Number(savedBaud).toLocaleString()} (Saved)`;
            const customOpt = this.baudRateSelect.querySelector('option[value="custom"]');
            if (customOpt) {
              this.baudRateSelect.insertBefore(opt, customOpt);
            } else {
              this.baudRateSelect.appendChild(opt);
            }
          }
          this.baudRateSelect.value = savedBaud;
          if (this.customBaudGroup) {
            this.customBaudGroup.classList.add('hidden');
          }
        }
      }
    } catch (e) {
      console.warn('Could not access localStorage for baud rate:', e);
    }
  }

  saveCurrentSettings() {
    try {
      if (this.baudRateSelect) {
        localStorage.setItem('serial_monitor_baud_rate', this.baudRateSelect.value);
      }
      if (this.customBaudInput && this.customBaudInput.value) {
        localStorage.setItem('serial_monitor_custom_baud', this.customBaudInput.value);
      }
    } catch (e) {
      console.warn('Could not save to localStorage:', e);
    }
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
      this.saveCurrentSettings();
    });

    if (this.customBaudInput) {
      this.customBaudInput.addEventListener('input', () => {
        this.saveCurrentSettings();
      });
    }

    // Connection buttons
    this.btnConnect.addEventListener('click', () => this.handleConnect());
    this.btnDisconnect.addEventListener('click', () => this.handleDisconnect());

    // Regex preset switcher
    this.regexPresetSelect.addEventListener('change', () => {
      const preset = this.regexPresetSelect.value;
      if (preset === 'telemetry' || preset === 'keyvalue') {
        this.customRegexInput.value = '([a-zA-Z0-9_\\-\\.]+)\\s*[:=]\\s*([-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?)';
      } else if (preset === 'numbers') {
        this.customRegexInput.value = '[-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?';
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

    // Clear plot - clears data points while preserving channel names and visibility ("keep the label")
    this.btnClearPlot.addEventListener('click', () => {
      this.channelData.forEach(series => {
        series.length = 0;
      });
      this.latestChannelValues.clear();
      this.plotPoints = 0;
      this.updateStats();
      this.updateLegend();
      this.updateLatestValuesDisplay();
      this.updateSplitYAxisAvailability();
    });

    // Split Y-Axis Toggle (Dual Y-Axis for 2 selected signals)
    this.splitYAxisCheck.addEventListener('change', () => {
      this.isSplitYAxis = this.splitYAxisCheck.checked;
    });

    // Record live CSV toggle
    this.btnRecordCsv.addEventListener('click', () => {
      this.toggleCsvRecording();
    });

    // Export current plot buffers as CSV
    this.btnExportPlotCsv.addEventListener('click', () => {
      this.exportCurrentPlotCsv();
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
      this.saveCurrentSettings();
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
        const trimmedLine = line.trim();

        // ONLY lines starting with "$DAT:" tag are plotted
        if (/^\$DAT:/i.test(trimmedLine)) {
          // Extract telemetry payload (strip "$DAT:" tag)
          const cleanData = trimmedLine.replace(/^\$DAT:\s*/i, '');

          // Feed exclusively into plotter & CSV recorder
          if (!this.isPlotPaused) {
            this.parseLineForPlotter(cleanData);
          }

          // Output to terminal only if user hasn't enabled "Filter Plots"
          const filterPlot = this.filterPlotInTerminalCheck && this.filterPlotInTerminalCheck.checked;
          if (!filterPlot) {
            this.appendTerminalLine('DAT', cleanData, 'line-telemetry');
          }
        } else {
          // All other lines are pure console logs (NEVER plotted)
          this.appendTerminalLine('RX', line, 'line-rx');
        }
      }
    }
  }

  getChannelColor(key) {
    if (this.channelColors.has(key)) {
      return this.channelColors.get(key);
    }
    const idx = this.channelOrder.indexOf(key);
    const colorIndex = (idx >= 0 ? idx : this.channelColors.size) % CHANNEL_COLORS.length;
    const color = CHANNEL_COLORS[colorIndex];
    this.channelColors.set(key, color);
    return color;
  }

  resolveChannelKey(rawLabel) {
    if (!rawLabel) return null;
    const label = rawLabel.trim();
    if (!label) return null;
    const lowerLabel = label.toLowerCase();

    // Map common aliases to canonical keys
    if (lowerLabel === 'temp' || lowerLabel === 'temperature') {
      for (const key of this.channelOrder) {
        if (key.toLowerCase() === 'temp' || key.toLowerCase() === 'temperature') return key;
      }
      return 'Temp';
    }
    if (lowerLabel === 'pwm') {
      for (const key of this.channelOrder) {
        if (key.toLowerCase() === 'pwm') return key;
      }
      return 'PWM';
    }

    // 1. Exact match with existing channel keys
    for (const key of this.channelOrder) {
      if (key.toLowerCase() === lowerLabel) return key;
    }

    // 2. Exact match with display names
    for (const [key, name] of this.channelNames.entries()) {
      if (name.toLowerCase() === lowerLabel) return key;
    }

    // 3. Strip trailing filler like 'set to', 'is', 'value'
    // e.g. "PWM set to" -> "PWM"
    const cleaned = label.replace(/\s+(?:set\s+to|is|val|value)$/i, '').trim();
    if (cleaned && cleaned.toLowerCase() !== lowerLabel) {
      for (const key of this.channelOrder) {
        if (key.toLowerCase() === cleaned.toLowerCase()) return key;
      }
    }

    // 4. Prefix match with word boundary (e.g. "PWM set to" starts with "PWM")
    for (const key of this.channelOrder) {
      const lowerKey = key.toLowerCase();
      if (lowerLabel.startsWith(lowerKey + ' ') || lowerLabel.startsWith(lowerKey + ':') || lowerLabel.startsWith(lowerKey + '_')) {
        return key;
      }
    }

    return cleaned || label;
  }

  getOrCreateChannel(key, displayName) {
    if (!this.channelData.has(key)) {
      this.channelOrder.push(key);
      this.channelData.set(key, []);
      this.channelNames.set(key, displayName || key);
      this.channelVisibility.set(key, true);
      this.getChannelColor(key);
      return true;
    }
    return false;
  }

  isChannelVisible(key) {
    return this.channelVisibility.get(key) !== false;
  }

  toggleChannelVisibility(key) {
    const nextState = !this.isChannelVisible(key);
    this.channelVisibility.set(key, nextState);
    this.updateLegend();
    this.updateLatestValuesDisplay();
    this.updateSplitYAxisAvailability();
  }

  setAllChannelsVisibility(visible) {
    this.channelOrder.forEach(key => {
      this.channelVisibility.set(key, visible);
    });
    this.updateLegend();
    this.updateLatestValuesDisplay();
    this.updateSplitYAxisAvailability();
  }

  updateSplitYAxisAvailability() {
    if (!this.splitYAxisCheck || !this.splitYAxisWrapper) return;
    const visibleKeys = this.channelOrder.filter(key => this.isChannelVisible(key));
    const isAvailable = (visibleKeys.length === 2);

    if (isAvailable) {
      this.splitYAxisCheck.disabled = false;
      this.splitYAxisWrapper.classList.remove('disabled');
      const name1 = this.channelNames.get(visibleKeys[0]) || visibleKeys[0];
      const name2 = this.channelNames.get(visibleKeys[1]) || visibleKeys[1];
      this.splitYAxisLabel.textContent = 'Split Y-Axis (2 Ch)';
      this.splitYAxisWrapper.title = `Split Y-Axis active: Left axis = ${name1}, Right axis = ${name2}`;
    } else {
      this.splitYAxisCheck.disabled = true;
      this.splitYAxisWrapper.classList.add('disabled');
      this.splitYAxisLabel.textContent = `Split Y-Axis (${visibleKeys.length}/2)`;
      this.splitYAxisWrapper.title = `Split Y-Axis requires exactly 2 signals selected (currently ${visibleKeys.length} active)`;
    }
  }

  toggleCsvRecording() {
    if (!this.isRecordingCsv) {
      this.startCsvRecording();
    } else {
      this.stopCsvRecording();
    }
  }

  startCsvRecording() {
    this.isRecordingCsv = true;
    this.csvRecordedRows = [];
    this.csvRecordingStartTime = Date.now();
    this.btnRecordCsv.classList.add('recording');
    this.btnRecordCsvText.textContent = 'Stop (0 rec)';
    this.logSystem('Live CSV recording of plotter data started...');
  }

  stopCsvRecording() {
    if (!this.isRecordingCsv) return;
    this.isRecordingCsv = false;
    this.btnRecordCsv.classList.remove('recording');
    this.btnRecordCsvText.textContent = 'Record CSV';

    const rowCount = this.csvRecordedRows.length;
    if (rowCount === 0) {
      this.logSystem('CSV recording stopped (no data rows recorded).');
      return;
    }

    const channelHeaders = this.channelOrder.map(key => {
      const name = this.channelNames.get(key) || key;
      return `"${name.replace(/"/g, '""')}"`;
    });

    const headers = ['Timestamp_ms', 'Elapsed_sec', 'ISO_Time', ...channelHeaders].join(',');
    const rows = [headers];

    this.csvRecordedRows.forEach(row => {
      const lineVals = [
        row.timeMs,
        row.elapsedSec,
        `"${row.isoTime}"`
      ];
      this.channelOrder.forEach(key => {
        const val = row.values.get(key);
        lineVals.push(val !== undefined && val !== null ? val : '');
      });
      rows.push(lineVals.join(','));
    });

    const csvContent = rows.join('\r\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const timestampStr = new Date().toISOString().slice(0, 19).replace(/:/g, '-');
    a.href = url;
    a.download = `plot-recording-${timestampStr}.csv`;
    a.click();
    URL.revokeObjectURL(url);

    this.logSystem(`CSV recording saved: ${rowCount} rows downloaded to plot-recording-${timestampStr}.csv`);
  }

  exportCurrentPlotCsv() {
    if (this.channelOrder.length === 0) {
      this.logError('Cannot export CSV: no channel data in plot buffers.');
      return;
    }

    const channelHeaders = this.channelOrder.map(key => {
      const name = this.channelNames.get(key) || key;
      return `"${name.replace(/"/g, '""')}"`;
    });

    let maxLen = 0;
    this.channelOrder.forEach(key => {
      const s = this.channelData.get(key);
      if (s && s.length > maxLen) maxLen = s.length;
    });

    if (maxLen === 0) {
      this.logError('Plot buffers are currently empty.');
      return;
    }

    const headers = ['Sample_Index', ...channelHeaders].join(',');
    const rows = [headers];

    for (let i = 0; i < maxLen; i++) {
      const lineVals = [i + 1];
      this.channelOrder.forEach(key => {
        const s = this.channelData.get(key);
        const offset = maxLen - s.length;
        const valIndex = i - offset;
        const val = (valIndex >= 0 && valIndex < s.length) ? s[valIndex] : '';
        lineVals.push(val);
      });
      rows.push(lineVals.join(','));
    }

    const csvContent = rows.join('\r\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const timestampStr = new Date().toISOString().slice(0, 19).replace(/:/g, '-');
    a.href = url;
    a.download = `plot-buffer-${timestampStr}.csv`;
    a.click();
    URL.revokeObjectURL(url);

    this.logSystem(`Exported ${maxLen} buffered points across ${this.channelOrder.length} channel(s) to CSV.`);
  }

  updateLatestValuesDisplay() {
    if (this.channelOrder.length === 0) {
      this.channelCountBadge.textContent = '0';
      this.latestValuesBadge.textContent = '--';
      return;
    }

    let visibleCount = 0;
    const items = [];
    this.channelOrder.forEach(key => {
      const isVisible = this.isChannelVisible(key);
      if (isVisible) visibleCount++;
      const name = this.channelNames.get(key) || key;
      const color = this.getChannelColor(key);
      const val = this.latestChannelValues.has(key) ? this.latestChannelValues.get(key) : '--';

      if (isVisible) {
        items.push(`<span class="latest-val-item" style="color: ${color}"><span class="val-name">${name}:</span> ${val}</span>`);
      } else {
        items.push(`<span class="latest-val-item is-hidden" style="color: var(--text-dim)" title="Channel hidden on plot (click legend label to show)"><span class="val-name">${name}:</span> ${val} <span class="val-tag">(hidden)</span></span>`);
      }
    });

    this.channelCountBadge.textContent = `${visibleCount}/${this.channelOrder.length}`;
    this.channelCountBadge.title = `${visibleCount} active of ${this.channelOrder.length} channels`;
    this.latestValuesBadge.innerHTML = items.join('<span class="meta-separator">|</span>');
  }

  updateLegendValuesOnly() {
    this.channelOrder.forEach(key => {
      if (!this.isChannelVisible(key)) return;
      const chip = this.plotChannelsLegend.querySelector(`[data-channel="${CSS.escape(key)}"]`);
      if (chip) {
        const valBadge = chip.querySelector('.legend-val-badge');
        if (valBadge && this.latestChannelValues.has(key)) {
          valBadge.textContent = this.latestChannelValues.get(key);
        }
      }
    });
  }

  parseLineForPlotter(line) {
    const pairs = [];

    // 1. Extract labeled pairs (label)[:=](value)
    // Matches labels before colons/equals, e.g. "Temperature: 25.40", "[MANUAL] PWM set to: 64"
    const labeledRegex = /([a-zA-Z0-9_\-\.\s]+?)\s*[:=]\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)/g;
    let match;

    while ((match = labeledRegex.exec(line)) !== null) {
      let rawLabel = match[1].trim();
      // Remove square bracket tags like [MANUAL], [TUNE], etc.
      rawLabel = rawLabel.replace(/\[.*?\]/g, '').replace(/^[;,\|\s]+/, '').trim();
      // Ignore if label looks like a log level or state tag
      if (!rawLabel || /^(?:CONFIG|TUNE|ERROR|WARN|INFO|STATUS|MANUAL|REPORT|CYCLE|PERIOD|PEAK|VALLEY)$/i.test(rawLabel)) {
        continue;
      }
      const val = parseFloat(match[2]);
      if (!isNaN(val) && isFinite(val) && rawLabel.length > 0) {
        pairs.push({ rawLabel, val });
      }
    }

    if (pairs.length > 0) {
      let legendNeedsUpdate = false;

      for (const item of pairs) {
        // Resolve channel key by label (NOT by line index!)
        const key = this.resolveChannelKey(item.rawLabel);
        const displayName = item.rawLabel.replace(/\s+(?:set\s+to|is)$/i, '').trim() || key;
        const isNew = this.getOrCreateChannel(key, displayName);
        if (isNew) legendNeedsUpdate = true;

        const series = this.channelData.get(key);
        series.push(item.val);
        this.latestChannelValues.set(key, item.val);
        this.plotPoints++;

        if (series.length > this.maxPlotWindow) {
          series.shift();
        }
      }

      // Record live CSV row if recording is active
      if (this.isRecordingCsv) {
        const now = Date.now();
        const elapsedSec = ((now - this.csvRecordingStartTime) / 1000).toFixed(3);
        const row = {
          timeMs: now,
          elapsedSec,
          isoTime: new Date(now).toISOString(),
          values: new Map()
        };
        this.channelOrder.forEach(key => {
          row.values.set(key, this.latestChannelValues.get(key));
        });
        this.csvRecordedRows.push(row);
        this.btnRecordCsvText.textContent = `Stop (${this.csvRecordedRows.length} rec)`;
      }

      if (legendNeedsUpdate) {
        this.updateLegend();
      } else {
        this.updateLegendValuesOnly();
      }

      this.updateLatestValuesDisplay();
      this.updateSplitYAxisAvailability();
      this.updateStats();
      return;
    }

    // 2. Unlabeled numbers fallback (only if no labels found on line)
    let numRegex;
    try {
      numRegex = new RegExp(this.customRegexInput.value, 'g');
    } catch (e) {
      numRegex = /[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?/g;
    }

    const numbers = [];
    while ((match = numRegex.exec(line)) !== null) {
      const valStr = match[1] !== undefined ? match[1] : match[0];
      const val = parseFloat(valStr);
      if (!isNaN(val) && isFinite(val)) {
        numbers.push(val);
      }
    }

    if (numbers.length > 0) {
      let legendNeedsUpdate = false;

      numbers.forEach((val, idx) => {
        const key = `CH${idx + 1}`;
        const isNew = this.getOrCreateChannel(key, key);
        if (isNew) legendNeedsUpdate = true;

        const series = this.channelData.get(key);
        series.push(val);
        this.latestChannelValues.set(key, val);
        this.plotPoints++;

        if (series.length > this.maxPlotWindow) {
          series.shift();
        }
      });

      if (this.isRecordingCsv) {
        const now = Date.now();
        const elapsedSec = ((now - this.csvRecordingStartTime) / 1000).toFixed(3);
        const row = {
          timeMs: now,
          elapsedSec,
          isoTime: new Date(now).toISOString(),
          values: new Map()
        };
        this.channelOrder.forEach(key => {
          row.values.set(key, this.latestChannelValues.get(key));
        });
        this.csvRecordedRows.push(row);
        this.btnRecordCsvText.textContent = `Stop (${this.csvRecordedRows.length} rec)`;
      }

      if (legendNeedsUpdate) {
        this.updateLegend();
      } else {
        this.updateLegendValuesOnly();
      }

      this.updateLatestValuesDisplay();
      this.updateSplitYAxisAvailability();
      this.updateStats();
    }
  }

  updateLegend() {
    this.plotChannelsLegend.innerHTML = '';
    if (this.channelOrder.length === 0) {
      this.updateSplitYAxisAvailability();
      return;
    }

    this.channelOrder.forEach(key => {
      const isVisible = this.isChannelVisible(key);
      const color = this.getChannelColor(key);
      const name = this.channelNames.get(key) || key;
      const latestVal = this.latestChannelValues.has(key) ? this.latestChannelValues.get(key) : null;

      const chip = document.createElement('button');
      chip.type = 'button';
      chip.className = `legend-chip ${isVisible ? 'is-visible' : 'is-hidden'}`;
      chip.setAttribute('data-channel', key);
      chip.setAttribute('aria-pressed', isVisible ? 'true' : 'false');
      chip.title = `Click to ${isVisible ? 'hide' : 'show'} "${name}" plot line\nDouble-click to rename label`;

      const eyeIcon = isVisible
        ? `<svg class="legend-eye-icon" width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"></path><circle cx="12" cy="12" r="3"></circle></svg>`
        : `<svg class="legend-eye-icon is-slashed" width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"></path><line x1="1" y1="1" x2="23" y2="23"></line></svg>`;

      chip.innerHTML = `
        <span class="legend-eye-wrap">${eyeIcon}</span>
        <span class="legend-color ${isVisible ? '' : 'is-muted'}" style="${isVisible ? `background: ${color}; box-shadow: 0 0 7px ${color};` : `border: 2px solid ${color}; background: transparent;`}"></span>
        <span class="legend-label ${isVisible ? '' : 'is-muted'}" style="color: ${isVisible ? color : 'var(--text-muted)'};">${name}</span>
        <span class="legend-val-badge ${isVisible ? '' : 'is-hidden-badge'}">${isVisible ? (latestVal !== null ? latestVal : '') : 'Hidden'}</span>
      `;

      chip.addEventListener('click', (e) => {
        e.preventDefault();
        this.toggleChannelVisibility(key);
      });

      chip.addEventListener('dblclick', (e) => {
        e.preventDefault();
        e.stopPropagation();
        const currentName = this.channelNames.get(key) || key;
        const newName = window.prompt(`Rename channel "${currentName}" label:`, currentName);
        if (newName && newName.trim()) {
          this.customChannelNames.add(key);
          this.channelNames.set(key, newName.trim());
          this.updateLegend();
          this.updateLatestValuesDisplay();
          this.updateSplitYAxisAvailability();
        }
      });

      this.plotChannelsLegend.appendChild(chip);
    });

    if (this.channelOrder.length > 1) {
      const allVisible = this.channelOrder.every(key => this.isChannelVisible(key));
      const toggleAllBtn = document.createElement('button');
      toggleAllBtn.type = 'button';
      toggleAllBtn.className = 'legend-toggle-all-btn';
      toggleAllBtn.textContent = allVisible ? 'Hide All' : 'Show All';
      toggleAllBtn.title = allVisible ? 'Hide all plot lines (labels remain)' : 'Show all plot lines';
      toggleAllBtn.addEventListener('click', (e) => {
        e.preventDefault();
        this.setAllChannelsVisibility(!allVisible);
      });
      this.plotChannelsLegend.appendChild(toggleAllBtn);
    }

    this.updateSplitYAxisAvailability();
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

    const visibleKeys = this.channelOrder.filter(key => this.isChannelVisible(key));
    const isSplitY = this.isSplitYAxis && (visibleKeys.length === 2);

    const padding = {
      top: 25,
      right: isSplitY ? 65 : 25,
      bottom: 35,
      left: 60
    };
    const plotW = width - padding.left - padding.right;
    const plotH = height - padding.top - padding.bottom;

    // Clear background
    ctx.fillStyle = '#06090e';
    ctx.fillRect(0, 0, width, height);

    if (plotW <= 0 || plotH <= 0) return;

    if (isSplitY) {
      // DUAL Y-AXIS MODE: Left Y-Axis for Channel 1, Right Y-Axis for Channel 2
      const ch1Key = visibleKeys[0];
      const ch2Key = visibleKeys[1];
      const series1 = this.channelData.get(ch1Key) || [];
      const series2 = this.channelData.get(ch2Key) || [];
      const color1 = this.getChannelColor(ch1Key);
      const color2 = this.getChannelColor(ch2Key);
      const name1 = this.channelNames.get(ch1Key) || ch1Key;
      const name2 = this.channelNames.get(ch2Key) || ch2Key;

      // Calculate range for Channel 1 (Left Y-Axis)
      let min1 = Infinity, max1 = -Infinity;
      for (let i = 0; i < series1.length; i++) {
        const v = series1[i];
        if (v < min1) min1 = v;
        if (v > max1) max1 = v;
      }
      if (!isFinite(min1) || !isFinite(max1)) { min1 = 0; max1 = 100; }
      else if (min1 === max1) { min1 -= 1; max1 += 1; }
      else {
        const span = max1 - min1;
        min1 -= span * 0.08;
        max1 += span * 0.08;
      }
      const range1 = max1 - min1 || 1;

      // Calculate range for Channel 2 (Right Y-Axis)
      let min2 = Infinity, max2 = -Infinity;
      for (let i = 0; i < series2.length; i++) {
        const v = series2[i];
        if (v < min2) min2 = v;
        if (v > max2) max2 = v;
      }
      if (!isFinite(min2) || !isFinite(max2)) { min2 = 0; max2 = 100; }
      else if (min2 === max2) { min2 -= 1; max2 += 1; }
      else {
        const span = max2 - min2;
        min2 -= span * 0.08;
        max2 += span * 0.08;
      }
      const range2 = max2 - min2 || 1;

      // Horizontal Grid Lines with Dual Labels
      const ySteps = 5;
      for (let i = 0; i <= ySteps; i++) {
        const ratio = i / ySteps;
        const yPos = padding.top + ratio * plotH;

        ctx.strokeStyle = 'rgba(255, 255, 255, 0.07)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(padding.left, yPos);
        ctx.lineTo(padding.left + plotW, yPos);
        ctx.stroke();

        // Left Y-Axis Label (Value 1 / Channel 1)
        const val1 = max1 - ratio * range1;
        ctx.font = 'bold 11px "JetBrains Mono", monospace';
        ctx.fillStyle = color1;
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';
        ctx.fillText(val1.toFixed(1), padding.left - 8, yPos);

        // Right Y-Axis Label (Value 2 / Channel 2)
        const val2 = max2 - ratio * range2;
        ctx.fillStyle = color2;
        ctx.textAlign = 'left';
        ctx.fillText(val2.toFixed(1), padding.left + plotW + 8, yPos);
      }

      // Vertical Grid Lines
      const xSteps = 5;
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.07)';
      for (let i = 0; i <= xSteps; i++) {
        const xPos = padding.left + (i / xSteps) * plotW;
        ctx.beginPath();
        ctx.moveTo(xPos, padding.top);
        ctx.lineTo(xPos, padding.top + plotH);
        ctx.stroke();
      }

      // Axis Headers on Canvas
      ctx.font = 'bold 11px "Outfit", sans-serif';
      ctx.textBaseline = 'bottom';
      ctx.textAlign = 'right';
      ctx.fillStyle = color1;
      ctx.fillText(`${name1} (Left Y)`, padding.left - 8, padding.top - 6);

      ctx.textAlign = 'left';
      ctx.fillStyle = color2;
      ctx.fillText(`${name2} (Right Y)`, padding.left + plotW + 8, padding.top - 6);

      // Outer Plot Border
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
      ctx.strokeRect(padding.left, padding.top, plotW, plotH);

      // Render Channel 1 Line (Normalized to range1)
      if (series1.length >= 2) {
        ctx.strokeStyle = color1;
        ctx.lineWidth = 2.2;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.beginPath();
        const stepX = plotW / (this.maxPlotWindow - 1);
        const startOffset = (this.maxPlotWindow - series1.length) * stepX;
        series1.forEach((val, i) => {
          const x = padding.left + startOffset + i * stepX;
          const normalizedY = (val - min1) / range1;
          const y = padding.top + plotH - normalizedY * plotH;
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();

        const lastIndex = series1.length - 1;
        const lastVal = series1[lastIndex];
        const lastX = padding.left + startOffset + lastIndex * stepX;
        const lastNormY = (lastVal - min1) / range1;
        const lastY = padding.top + plotH - lastNormY * plotH;
        ctx.fillStyle = color1;
        ctx.beginPath();
        ctx.arc(lastX, lastY, 4, 0, Math.PI * 2);
        ctx.fill();
      }

      // Render Channel 2 Line (Normalized to range2)
      if (series2.length >= 2) {
        ctx.strokeStyle = color2;
        ctx.lineWidth = 2.2;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.beginPath();
        const stepX = plotW / (this.maxPlotWindow - 1);
        const startOffset = (this.maxPlotWindow - series2.length) * stepX;
        series2.forEach((val, i) => {
          const x = padding.left + startOffset + i * stepX;
          const normalizedY = (val - min2) / range2;
          const y = padding.top + plotH - normalizedY * plotH;
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();

        const lastIndex = series2.length - 1;
        const lastVal = series2[lastIndex];
        const lastX = padding.left + startOffset + lastIndex * stepX;
        const lastNormY = (lastVal - min2) / range2;
        const lastY = padding.top + plotH - lastNormY * plotH;
        ctx.fillStyle = color2;
        ctx.beginPath();
        ctx.arc(lastX, lastY, 4, 0, Math.PI * 2);
        ctx.fill();
      }
    } else {
      // STANDARD SINGLE Y-AXIS MODE
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
        let hasVisibleSeries = false;
        this.channelOrder.forEach(key => {
          if (!this.isChannelVisible(key)) return;
          const series = this.channelData.get(key) || [];
          for (let i = 0; i < series.length; i++) {
            const val = series[i];
            if (val < minY) minY = val;
            if (val > maxY) maxY = val;
            hasVisibleSeries = true;
          }
        });

        if (!hasVisibleSeries || !isFinite(minY) || !isFinite(maxY)) {
          minY = 0;
          maxY = 100;
        } else if (minY === maxY) {
          minY -= 1;
          maxY += 1;
        } else {
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

      // Render Data Series Lines (visible only)
      this.channelOrder.forEach(key => {
        if (!this.isChannelVisible(key)) return;
        const series = this.channelData.get(key) || [];
        if (series.length < 2) return;

        const color = this.getChannelColor(key);
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
    }

    // Handle empty or all-hidden series placeholder hints
    const allHidden = this.channelOrder.length > 0 && this.channelOrder.every(key => !this.isChannelVisible(key));
    if (allHidden) {
      ctx.fillStyle = '#64748b';
      ctx.font = '13px "Outfit", sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('All plot lines hidden — click channel labels in legend above to show', width / 2, height / 2);
    } else if (this.channelOrder.length === 0) {
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
  window.serialApp = new SerialMonitorApp();
});
