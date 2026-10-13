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
    this.MOTOR_BAUD_RATE = 57600;       // Fixed Robotis U2D2 / XL430 baud rate
    this.ARDUINO_BAUD_RATE = 115200;    // Fixed Arduino Force-Bench firmware baud rate

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
    this.commandedPositionTicks = 0;    // Discrete incremental target coordinate tracker
    this.softwareZeroOffsetTicks = 0;   // Software zero offset for display and logging (no DXL write)
    this.motorRxBuffer = [];
    this.motorActiveResolver = null;
    this.motorPollTimer = null;
    this.motorIsPollingBusy = false;
    this.motorIsContinuousMoving = false;
    this.continuousJogDirection = 0;       // 0: idle, +1: forward, -1: reverse
    this.continuousJogPressStartTime = 0;  // Track press duration for hold vs click latch
    this.motorTxQueue = [];             // Mutex FIFO queue for serial packet transmission
    this.isMotorTxBusy = false;
    this.savedMotorVid = localStorage.getItem('unified_dxl_vid') || '0403';
    this.savedMotorPid = localStorage.getItem('unified_dxl_pid') || '6014';

    // ------------------------------------------------------------------------
    // Port 2: Arduino Heated Loadcell State
    // ------------------------------------------------------------------------
    this.arduinoPort = null;
    this.arduinoReader = null;
    this.arduinoWriter = null;
    this.arduinoKeepReading = false;
    this.isArduinoConnected = false;
    this.arduinoBuffer = "";
    this.savedArduinoVid = localStorage.getItem('unified_arduino_vid') || '1A86';
    this.savedArduinoPid = localStorage.getItem('unified_arduino_pid') || '7523';
    this.arduinoReportWatchdog = null;
    this.lastArduinoDatTimestamp = 0;
    this.liveTempC = null;
    this.liveWeightKg = 0.0;
    this.liveForceN = 0.0;
    this.livePwm = 0;
    this.liveSetpoint = 50.0;
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
    this.testAutoReturn = false;
    this.testStartMm = 0.0;
    this.testPeakForceN = 0.0;
    this.testDispAtPeakMm = 0.0;
    this.testStiffnessNPerMm = 0.0;
    this.testEnergyMJ = 0.0;
    this.testPollInterval = null;

    this.testConfigs = {
      push: {
        travelMm: parseFloat(localStorage.getItem('push_travel_mm')) || 10.0,
        speedMmS: parseFloat(localStorage.getItem('push_speed_mms')) || 1.0,
        maxForceLimitN: parseFloat(localStorage.getItem('push_force_limit')) || 50.0,
        autoReturn: localStorage.getItem('push_auto_return') === 'true',
        autoTare: localStorage.getItem('push_auto_tare') !== 'false'
      },
      pull: {
        travelMm: parseFloat(localStorage.getItem('pull_travel_mm')) || 10.0,
        speedMmS: parseFloat(localStorage.getItem('pull_speed_mms')) || 1.0,
        maxForceLimitN: parseFloat(localStorage.getItem('pull_force_limit')) || 50.0,
        autoReturn: localStorage.getItem('pull_auto_return') === 'true',
        autoTare: localStorage.getItem('pull_auto_tare') !== 'false'
      }
    };
    this.modalActiveTestDirection = 'push';

    // ------------------------------------------------------------------------
    // Initialization
    // ------------------------------------------------------------------------
    this.cacheDOMElements();
    this.bindEvents();
    this.initCanvasPlots();
    this.startPlotAnimationLoop();
    this.updateUsbPillDisplays();
    this.log("SYS", "Heated Force Bench initialized. Ready to connect ports.", "bench");
  }

  // --------------------------------------------------------------------------
  // DOM Cache
  // --------------------------------------------------------------------------
  cacheDOMElements() {
    this.el = {
      // Ports
      cardMotorPort: document.getElementById('cardMotorPort'),
      indMotor: document.getElementById('indMotor'),
      pillMotorUsb: document.getElementById('pillMotorUsb'),
      motorIdInput: document.getElementById('motorIdInput'),
      btnConnectMotor: document.getElementById('btnConnectMotor'),
      btnPairMotor: document.getElementById('btnPairMotor'),

      cardArduinoPort: document.getElementById('cardArduinoPort'),
      indArduino: document.getElementById('indArduino'),
      pillArduinoUsb: document.getElementById('pillArduinoUsb'),
      btnConnectArduino: document.getElementById('btnConnectArduino'),
      btnPairArduino: document.getElementById('btnPairArduino'),
      btnEmergencyStop: document.getElementById('btnEmergencyStop'),

      // Header System Config Modal & Status Indicators
      btnSystemConfig: document.getElementById('btnSystemConfig'),
      modalSystemConfig: document.getElementById('modalSystemConfig'),
      btnCloseSystemConfigModal: document.getElementById('btnCloseSystemConfigModal'),
      btnDoneSystemConfigModal: document.getElementById('btnDoneSystemConfigModal'),
      pillNavMotor: document.getElementById('pillNavMotor'),
      pillNavArduino: document.getElementById('pillNavArduino'),
      dotNavMotor: document.getElementById('dotNavMotor'),
      dotNavArduino: document.getElementById('dotNavArduino'),

      // Motion Axis
      badgeTorqueState: document.getElementById('badgeTorqueState'),
      dispActiveId: document.getElementById('dispActiveId'),
      dispLinearMm: document.getElementById('dispLinearMm'),
      dispRevs: document.getElementById('dispRevs'),
      dispTicks: document.getElementById('dispTicks'),
      btnSoftwareZero: document.getElementById('btnSoftwareZero'),
      btnClearSoftwareZero: document.getElementById('btnClearSoftwareZero'),
      dispSoftwareOffsetNote: document.getElementById('dispSoftwareOffsetNote'),
      btnSyncMotor: document.getElementById('btnSyncMotor'),
      btnOpenSpeedModal: document.getElementById('btnOpenSpeedModal'),
      modalJogSpeed: document.getElementById('modalJogSpeed'),
      btnCloseSpeedModal: document.getElementById('btnCloseSpeedModal'),
      btnDoneSpeedModal: document.getElementById('btnDoneSpeedModal'),
      dispCurrentSpeed: document.getElementById('dispCurrentSpeed'),
      selectJogSpeed: document.getElementById('selectJogSpeed'),
      inputCustomSpeed: document.getElementById('inputCustomSpeed'),
      btnApplyCustomSpeed: document.getElementById('btnApplyCustomSpeed'),
      inputCustomJogStep: document.getElementById('inputCustomJogStep'),
      btnCustomJogRev: document.getElementById('btnCustomJogRev'),
      btnCustomJogFwd: document.getElementById('btnCustomJogFwd'),
      // Continuous Jog (At Vmax)
      dispContinuousState: document.getElementById('dispContinuousState'),
      btnContinuousRev: document.getElementById('btnContinuousRev'),
      btnContinuousStop: document.getElementById('btnContinuousStop'),
      btnContinuousFwd: document.getElementById('btnContinuousFwd'),
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
      btnClearPushPullPlot: document.getElementById('btnClearPushPullPlot'),
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
      dispLoadcellWeight: document.getElementById('dispLoadcellWeight'),
      dispLoadcellForce: document.getElementById('dispLoadcellForce'),
      btnToggleLoadcellSettings: document.getElementById('btnToggleLoadcellSettings'),
      drawerLoadcellSettings: document.getElementById('drawerLoadcellSettings'),
      iconLoadcellSettingsArrow: document.getElementById('iconLoadcellSettingsArrow'),
      btnTareLoadcell: document.getElementById('btnTareLoadcell'),
      btnCalibratePrompt: document.getElementById('btnCalibratePrompt'),

      // Automated Mechanical Test
      dispTestPhase: document.getElementById('dispTestPhase'),
      rowTestButtons: document.getElementById('rowTestButtons'),
      btnStartPushTest: document.getElementById('btnStartPushTest'),
      btnConfigPushTest: document.getElementById('btnConfigPushTest'),
      summaryPushParams: document.getElementById('summaryPushParams'),
      btnStartPullTest: document.getElementById('btnStartPullTest'),
      btnConfigPullTest: document.getElementById('btnConfigPullTest'),
      summaryPullParams: document.getElementById('summaryPullParams'),
      btnAbortTest: document.getElementById('btnAbortTest'),
      cardTestResults: document.getElementById('cardTestResults'),
      lblResPeakForce: document.getElementById('lblResPeakForce'),
      resPeakForce: document.getElementById('resPeakForce'),
      resPeakDisp: document.getElementById('resPeakDisp'),
      resStiffness: document.getElementById('resStiffness'),
      resEnergy: document.getElementById('resEnergy'),
      btnExportCsv: document.getElementById('btnExportCsv'),

      // Modal Test Config
      modalTestConfig: document.getElementById('modalTestConfig'),
      titleTestConfigModal: document.getElementById('titleTestConfigModal'),
      noticeTestConfigDirection: document.getElementById('noticeTestConfigDirection'),
      btnCloseTestConfigModal: document.getElementById('btnCloseTestConfigModal'),
      btnSaveTestConfigModal: document.getElementById('btnSaveTestConfigModal'),
      inputModalTestDistance: document.getElementById('inputModalTestDistance'),
      inputModalTestSpeed: document.getElementById('inputModalTestSpeed'),
      inputModalMaxForceLimit: document.getElementById('inputModalMaxForceLimit'),
      checkModalAutoReturn: document.getElementById('checkModalAutoReturn'),
      checkModalAutoTare: document.getElementById('checkModalAutoTare'),

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
    if (this.el.btnPairMotor) {
      this.el.btnPairMotor.addEventListener('click', () => this.pairNewMotorPort());
    }
    if (this.el.btnPairArduino) {
      this.el.btnPairArduino.addEventListener('click', () => this.pairNewArduinoPort());
    }

    // Modal: System Hardware Configuration & USB Port Connections
    const openSystemConfigModal = () => {
      if (this.el.modalSystemConfig) {
        this.el.modalSystemConfig.classList.add('is-open');
        this.el.modalSystemConfig.setAttribute('aria-hidden', 'false');
      }
    };
    const closeSystemConfigModal = () => {
      if (this.el.modalSystemConfig) {
        this.el.modalSystemConfig.classList.remove('is-open');
        this.el.modalSystemConfig.setAttribute('aria-hidden', 'true');
      }
    };

    if (this.el.btnSystemConfig) {
      this.el.btnSystemConfig.addEventListener('click', openSystemConfigModal);
    }
    if (this.el.pillNavMotor) {
      this.el.pillNavMotor.addEventListener('click', openSystemConfigModal);
    }
    if (this.el.pillNavArduino) {
      this.el.pillNavArduino.addEventListener('click', openSystemConfigModal);
    }
    if (this.el.btnCloseSystemConfigModal) {
      this.el.btnCloseSystemConfigModal.addEventListener('click', closeSystemConfigModal);
    }
    if (this.el.btnDoneSystemConfigModal) {
      this.el.btnDoneSystemConfigModal.addEventListener('click', closeSystemConfigModal);
    }
    if (this.el.modalSystemConfig) {
      this.el.modalSystemConfig.addEventListener('click', (e) => {
        if (e.target === this.el.modalSystemConfig) closeSystemConfigModal();
      });
    }
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.el.modalSystemConfig && this.el.modalSystemConfig.classList.contains('is-open')) {
        closeSystemConfigModal();
      }
    });

    // Motor ID Input change
    this.el.motorIdInput.addEventListener('change', (e) => {
      this.motorActiveId = parseInt(e.target.value, 10) || 1;
      if (this.el.dispActiveId) {
        this.el.dispActiveId.textContent = `ID ${this.motorActiveId} (XL430)`;
      }
      if (this.isMotorConnected) this.syncMotorHardware();
    });

    // Modal: Jog Speed Configuration Pop-up
    const openSpeedModal = () => {
      if (this.el.modalJogSpeed) {
        this.el.modalJogSpeed.classList.add('is-open');
        this.el.modalJogSpeed.setAttribute('aria-hidden', 'false');
      }
    };
    const closeSpeedModal = () => {
      if (this.el.modalJogSpeed) {
        this.el.modalJogSpeed.classList.remove('is-open');
        this.el.modalJogSpeed.setAttribute('aria-hidden', 'true');
      }
    };

    if (this.el.btnOpenSpeedModal) {
      this.el.btnOpenSpeedModal.addEventListener('click', (e) => {
        e.stopPropagation();
        openSpeedModal();
      });
    }
    if (this.el.dispCurrentSpeed) {
      this.el.dispCurrentSpeed.addEventListener('click', (e) => {
        e.stopPropagation();
        openSpeedModal();
      });
    }
    if (this.el.btnCloseSpeedModal) {
      this.el.btnCloseSpeedModal.addEventListener('click', closeSpeedModal);
    }
    if (this.el.btnDoneSpeedModal) {
      this.el.btnDoneSpeedModal.addEventListener('click', closeSpeedModal);
    }
    if (this.el.modalJogSpeed) {
      this.el.modalJogSpeed.addEventListener('click', (e) => {
        if (e.target === this.el.modalJogSpeed) closeSpeedModal();
      });
    }
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.el.modalJogSpeed && this.el.modalJogSpeed.classList.contains('is-open')) {
        closeSpeedModal();
      }
    });

    // Speed Preset Dropdown (inside modal)
    if (this.el.selectJogSpeed) {
      this.el.selectJogSpeed.addEventListener('change', (e) => {
        const val = e.target.value;
        if (val === 'custom') {
          if (this.el.inputCustomSpeed) {
            this.el.inputCustomSpeed.focus();
            this.el.inputCustomSpeed.select();
          }
        } else {
          const spd = parseFloat(val);
          if (spd > 0) this.setJogSpeed(spd);
        }
      });
    }

    if (this.el.btnApplyCustomSpeed) {
      this.el.btnApplyCustomSpeed.addEventListener('click', () => {
        const spd = parseFloat(this.el.inputCustomSpeed.value);
        if (spd > 0) this.setJogSpeed(spd);
      });
    }

    // Jog Distance Presets (0.1, 0.5, 1.0, 5.0, 10.0 mm)
    document.querySelectorAll('.jog-step-preset').forEach(btn => {
      btn.addEventListener('click', () => {
        const step = parseFloat(btn.dataset.step);
        if (this.el.inputCustomJogStep && step > 0) {
          this.el.inputCustomJogStep.value = step;
          document.querySelectorAll('.jog-step-preset').forEach(b => b.classList.remove('active'));
          btn.classList.add('active');
        }
      });
    });

    if (this.el.inputCustomJogStep) {
      this.el.inputCustomJogStep.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        document.querySelectorAll('.jog-step-preset').forEach(b => {
          b.classList.toggle('active', Math.abs(parseFloat(b.dataset.step) - val) < 0.001);
        });
      });
    }

    // Jog Step Movement (STEP - and STEP +)
    if (this.el.btnCustomJogRev) {
      this.el.btnCustomJogRev.addEventListener('click', () => {
        const step = parseFloat(this.el.inputCustomJogStep ? this.el.inputCustomJogStep.value : 1.0) || 1.0;
        this.jogRelativeMm(-Math.abs(step));
      });
    }
    if (this.el.btnCustomJogFwd) {
      this.el.btnCustomJogFwd.addEventListener('click', () => {
        const step = parseFloat(this.el.inputCustomJogStep ? this.el.inputCustomJogStep.value : 1.0) || 1.0;
        this.jogRelativeMm(Math.abs(step));
      });
    }

    // Continuous Jog Controls (Keep moving goal forward/reverse as long as button is held down)
    if (this.el.btnContinuousFwd) {
      this.el.btnContinuousFwd.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        this.startContinuousJogStream(1);
      });
      const endFwd = () => {
        if (this.continuousJogDirection === 1) {
          this.stopContinuousJogStream();
        }
      };
      this.el.btnContinuousFwd.addEventListener('pointerup', endFwd);
      this.el.btnContinuousFwd.addEventListener('pointercancel', endFwd);
      this.el.btnContinuousFwd.addEventListener('pointerleave', endFwd);
    }

    if (this.el.btnContinuousRev) {
      this.el.btnContinuousRev.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        this.startContinuousJogStream(-1);
      });
      const endRev = () => {
        if (this.continuousJogDirection === -1) {
          this.stopContinuousJogStream();
        }
      };
      this.el.btnContinuousRev.addEventListener('pointerup', endRev);
      this.el.btnContinuousRev.addEventListener('pointercancel', endRev);
      this.el.btnContinuousRev.addEventListener('pointerleave', endRev);
    }

    window.addEventListener('pointerup', () => {
      if (this.continuousJogDirection !== 0) {
        this.stopContinuousJogStream();
      }
    });

    if (this.el.btnContinuousStop) {
      this.el.btnContinuousStop.addEventListener('click', () => {
        this.stopContinuousJogStream();
      });
    }

    // Origin and Hardware Tools
    if (this.el.btnSoftwareZero) {
      this.el.btnSoftwareZero.addEventListener('click', () => this.setSoftwareZero());
    }
    if (this.el.btnClearSoftwareZero) {
      this.el.btnClearSoftwareZero.addEventListener('click', () => this.clearSoftwareZero());
    }
    this.el.btnResetEncoder.addEventListener('click', () => this.resetMultiTurnEncoder());
    this.el.btnGotoOrigin.addEventListener('click', () => this.gotoOriginMm());
    this.el.btnToggleTorque.addEventListener('click', () => this.toggleTorque());
    if (this.el.btnSyncMotor) {
      this.el.btnSyncMotor.addEventListener('click', () => this.syncMotorHardware(true));
    }

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

    // Load Cell Settings Drawer Toggle
    if (this.el.btnToggleLoadcellSettings && this.el.drawerLoadcellSettings) {
      this.el.btnToggleLoadcellSettings.addEventListener('click', () => {
        const isOpen = this.el.drawerLoadcellSettings.classList.toggle('open');
        if (this.el.iconLoadcellSettingsArrow) {
          this.el.iconLoadcellSettingsArrow.textContent = isOpen ? '▴' : '▾';
        }
      });
    }

    // Load Cell Actions
    this.el.btnTareLoadcell.addEventListener('click', () => this.tareLoadcell());
    this.el.btnCalibratePrompt.addEventListener('click', () => this.calibrateLoadcell());

    // Automated Push / Pull Test
    if (this.el.btnStartPushTest) {
      this.el.btnStartPushTest.addEventListener('click', () => this.startTestSequence('push'));
    }
    if (this.el.btnStartPullTest) {
      this.el.btnStartPullTest.addEventListener('click', () => this.startTestSequence('pull'));
    }
    if (this.el.btnAbortTest) {
      this.el.btnAbortTest.addEventListener('click', () => this.abortTestSequence());
    }
    if (this.el.btnClearPushPullPlot) {
      this.el.btnClearPushPullPlot.addEventListener('click', () => this.clearTestPlot(true));
    }

    if (this.el.btnConfigPushTest) {
      this.el.btnConfigPushTest.addEventListener('click', () => this.openTestConfigModal('push'));
    }
    if (this.el.btnConfigPullTest) {
      this.el.btnConfigPullTest.addEventListener('click', () => this.openTestConfigModal('pull'));
    }

    if (this.el.btnCloseTestConfigModal) {
      this.el.btnCloseTestConfigModal.addEventListener('click', () => this.closeTestConfigModal());
    }
    if (this.el.btnSaveTestConfigModal) {
      this.el.btnSaveTestConfigModal.addEventListener('click', () => this.saveTestConfigFromModal());
    }
    if (this.el.modalTestConfig) {
      this.el.modalTestConfig.addEventListener('click', (e) => {
        if (e.target === this.el.modalTestConfig) this.closeTestConfigModal();
      });
    }
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.el.modalTestConfig && this.el.modalTestConfig.classList.contains('is-open')) {
        this.closeTestConfigModal();
      }
    });

    this.updateTestParamSummaries();

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

    // Collapsible Cards & Subplots Event Delegation
    document.querySelectorAll('.btn-card-toggle').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const targetId = btn.dataset.target;
        let card = null;
        if (targetId) card = document.getElementById(targetId);
        if (!card) card = btn.closest('.collapsible-card') || btn.closest('.plot-card');
        if (card) {
          card.classList.toggle('is-collapsed');
          this.resizeCanvases();
        }
      });
    });

    document.querySelectorAll('.card-collapse-header, .plot-card-header').forEach(hdr => {
      hdr.addEventListener('click', (e) => {
        if (e.target.closest('button, input, select, a, .filter-tabs')) return;
        const card = hdr.closest('.collapsible-card') || hdr.closest('.plot-card');
        if (card) {
          card.classList.toggle('is-collapsed');
          this.resizeCanvases();
        }
      });
    });
  }

  // --------------------------------------------------------------------------
  // USB Port Persistence & Device Identification Helpers
  // --------------------------------------------------------------------------
  formatVidPid(vid, pid) {
    if (!vid || !pid) return "VID:-- PID:--";
    const v = (typeof vid === 'number' ? vid.toString(16) : vid.toString()).padStart(4, '0').toUpperCase();
    const p = (typeof pid === 'number' ? pid.toString(16) : pid.toString()).padStart(4, '0').toUpperCase();
    return `VID:${v} PID:${p}`;
  }

  updateUsbPillDisplays() {
    if (this.el.pillMotorUsb) {
      const text = this.formatVidPid(this.savedMotorVid, this.savedMotorPid);
      this.el.pillMotorUsb.textContent = text;
      this.el.pillMotorUsb.title = this.savedMotorVid
        ? `Remembered U2D2 Port: ${text}`
        : "No U2D2 port paired yet. Click connect or ⇄ to pair.";
    }
    if (this.el.pillArduinoUsb) {
      const text = this.formatVidPid(this.savedArduinoVid, this.savedArduinoPid);
      this.el.pillArduinoUsb.textContent = text;
      this.el.pillArduinoUsb.title = this.savedArduinoVid
        ? `Remembered Arduino Port: ${text}`
        : "No Arduino port paired yet. Click connect or ⇄ to pair.";
    }
  }

  /**
   * Find an authorized SerialPort from navigator.serial.getPorts()
   * with smart fallback so U2D2 connects automatically without prompt window
   */
  async findMatchingMotorPort(excludePort = null) {
    if (!navigator.serial) return null;
    try {
      const ports = await navigator.serial.getPorts();
      if (!ports || ports.length === 0) return null;

      const cleanV = (this.savedMotorVid || '0403').toLowerCase().replace(/^0x/, '');
      const cleanP = (this.savedMotorPid || '6014').toLowerCase().replace(/^0x/, '');

      // 1. Exact match with saved VID & PID
      for (const p of ports) {
        if (excludePort && p === excludePort) continue;
        const info = p.getInfo ? p.getInfo() : {};
        if (info.usbVendorId !== undefined && info.usbProductId !== undefined) {
          const v = info.usbVendorId.toString(16).toLowerCase();
          const pid = info.usbProductId.toString(16).toLowerCase();
          if (v === cleanV && pid === cleanP) return p;
        }
      }

      // 2. Known U2D2 / FTDI vendor (0x0403) with PID 0x6014 or 0x6001
      for (const p of ports) {
        if (excludePort && p === excludePort) continue;
        const info = p.getInfo ? p.getInfo() : {};
        if (info.usbVendorId !== undefined) {
          const v = info.usbVendorId.toString(16).toLowerCase();
          if (v === '0403' || v === '403') return p;
        }
      }

      // 3. Fallback: Any authorized port that is NOT Arduino (CH340: 0x1A86 or 0x2341)
      for (const p of ports) {
        if (excludePort && p === excludePort) continue;
        const info = p.getInfo ? p.getInfo() : {};
        if (info.usbVendorId !== undefined) {
          const v = info.usbVendorId.toString(16).toLowerCase();
          if (v !== '1a86' && v !== '2341') return p;
        } else if (ports.length === 1 && !excludePort) {
          return p;
        }
      }
    } catch (e) {
      console.warn("findMatchingMotorPort query error:", e);
    }
    return null;
  }

  async findMatchingArduinoPort(excludePort = null) {
    if (!navigator.serial) return null;
    try {
      const ports = await navigator.serial.getPorts();
      if (!ports || ports.length === 0) return null;

      const cleanV = (this.savedArduinoVid || '1A86').toLowerCase().replace(/^0x/, '');
      const cleanP = (this.savedArduinoPid || '7523').toLowerCase().replace(/^0x/, '');

      // 1. Exact match with saved VID & PID
      for (const p of ports) {
        if (excludePort && p === excludePort) continue;
        const info = p.getInfo ? p.getInfo() : {};
        if (info.usbVendorId !== undefined && info.usbProductId !== undefined) {
          const v = info.usbVendorId.toString(16).toLowerCase();
          const pid = info.usbProductId.toString(16).toLowerCase();
          if (v === cleanV && pid === cleanP) return p;
        }
      }

      // 2. Known Arduino / CH340 / CP210x (0x1A86, 0x2341, 0x10C4)
      for (const p of ports) {
        if (excludePort && p === excludePort) continue;
        const info = p.getInfo ? p.getInfo() : {};
        if (info.usbVendorId !== undefined) {
          const v = info.usbVendorId.toString(16).toLowerCase();
          if (v === '1a86' || v === '2341' || v === '10c4') return p;
        }
      }

      // 3. Any port that is NOT U2D2 (0x0403)
      for (const p of ports) {
        if (excludePort && p === excludePort) continue;
        const info = p.getInfo ? p.getInfo() : {};
        if (info.usbVendorId !== undefined) {
          const v = info.usbVendorId.toString(16).toLowerCase();
          if (v !== '0403' && v !== '403') return p;
        } else if (ports.length === 1 && !excludePort) {
          return p;
        }
      }
    } catch (e) {
      console.warn("findMatchingArduinoPort query error:", e);
    }
    return null;
  }

  async findMatchingPort(targetVid, targetPid, excludePort = null) {
    return this.findMatchingMotorPort(excludePort);
  }

  // --------------------------------------------------------------------------
  // Speed & Coordinate Conversion Helpers
  // --------------------------------------------------------------------------
  async setJogSpeed(speedMmS) {
    // Motor limit: Max 3.0 mm/s (180 RPM) for XL430 actuator on 1 mm pitch lead screw
    this.CURRENT_SPEED_MM_S = Math.min(3.0, Math.max(0.05, speedMmS));
    const rpm = (this.CURRENT_SPEED_MM_S * 60).toFixed(1);
    const velUnits = this.speedMmSToVelocityUnits(this.CURRENT_SPEED_MM_S);

    if (this.el.dispCurrentSpeed) {
      this.el.dispCurrentSpeed.textContent = `Vmax: ${this.CURRENT_SPEED_MM_S.toFixed(1)} mm/s`;
      this.el.dispCurrentSpeed.title = `Configured Jog Speed: ${this.CURRENT_SPEED_MM_S} mm/s - Click to configure`;
    }
    if (this.el.inputCustomSpeed) {
      this.el.inputCustomSpeed.value = this.CURRENT_SPEED_MM_S;
    }

    // Update select dropdown value
    if (this.el.selectJogSpeed) {
      let matched = false;
      for (const opt of this.el.selectJogSpeed.options) {
        if (Math.abs(parseFloat(opt.value) - this.CURRENT_SPEED_MM_S) < 0.001) {
          this.el.selectJogSpeed.value = opt.value;
          matched = true;
          break;
        }
      }
      if (!matched) {
        this.el.selectJogSpeed.value = "custom";
      }
    }

    // Update V_max (Profile Velocity, Addr 112) and Profile Acceleration (Addr 108) immediately in hardware if connected!
    if (this.isMotorConnected) {
      const velBytes = DXL.intToBytes(velUnits, 4);
      const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
      await this.sendMotorPacket(velPkt, `UPDATE VMAX = ${this.CURRENT_SPEED_MM_S} mm/s (${velUnits})`);

      // Scale acceleration with velocity so trapezoidal ramp remains ~0.18-0.25s (prevents counter-rotation jerk)
      const accelUnits = Math.min(32767, Math.max(80, Math.round(velUnits * 0.35)));
      const accelBytes = DXL.intToBytes(accelUnits, 4);
      const accelPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [108, 0, ...accelBytes]);
      await this.sendMotorPacket(accelPkt, `UPDATE PROFILE ACCEL = ${accelUnits}`);

      this.log("MOTOR", `Vmax updated: ${this.CURRENT_SPEED_MM_S} mm/s (${velUnits} units, Accel ${accelUnits})`, "motor");
    }
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

  async pairNewMotorPort() {
    if (this.isMotorConnected) {
      await this.disconnectMotor();
    }
    await this.connectMotor(true);
  }

  async connectMotor(forceSelectNew = false) {
    if (!navigator.serial) {
      alert("Web Serial API is not supported in this browser. Please use Google Chrome or Edge.");
      return;
    }

    try {
      let selectedPort = null;
      if (!forceSelectNew) {
        selectedPort = await this.findMatchingMotorPort(this.arduinoPort);
        if (selectedPort) {
          this.log("MOTOR", `Auto-selected remembered U2D2 port`, "motor");
        }
      }

      if (!selectedPort) {
        // Filter specifically for Robotis U2D2 (VID: 0x0403, PID: 0x6014 or 0x6001)
        const filters = [
          { usbVendorId: 0x0403, usbProductId: 0x6014 },
          { usbVendorId: 0x0403, usbProductId: 0x6001 },
          { usbVendorId: 0x0403 }
        ];
        try {
          selectedPort = await navigator.serial.requestPort({ filters });
        } catch (filterErr) {
          if (filterErr.name === 'NotFoundError') throw filterErr;
          selectedPort = await navigator.serial.requestPort();
        }
      }

      this.motorPort = selectedPort;
      const info = this.motorPort.getInfo ? this.motorPort.getInfo() : {};
      if (info && info.usbVendorId !== undefined && info.usbProductId !== undefined) {
        this.savedMotorVid = info.usbVendorId.toString(16).padStart(4, '0').toUpperCase();
        this.savedMotorPid = info.usbProductId.toString(16).padStart(4, '0').toUpperCase();
        localStorage.setItem('unified_dxl_vid', this.savedMotorVid);
        localStorage.setItem('unified_dxl_pid', this.savedMotorPid);
        this.updateUsbPillDisplays();
        this.log("MOTOR", `Paired U2D2 USB: ${this.formatVidPid(this.savedMotorVid, this.savedMotorPid)}`, "motor");
      }

      const baudRate = this.MOTOR_BAUD_RATE;
      await this.motorPort.open({ baudRate, bufferSize: 4096 });

      this.isMotorConnected = true;
      this.motorKeepReading = true;
      this.motorTxQueue = [];
      this.isMotorTxBusy = false;
      this.el.cardMotorPort.classList.add('connected');
      if (this.el.dotNavMotor) this.el.dotNavMotor.classList.add('connected');
      this.el.btnConnectMotor.textContent = "DISCONNECT";
      this.el.btnConnectMotor.classList.replace('btn-primary-sm', 'btn-danger-sm');

      // Start non-blocking reader loop
      this.startMotorReaderLoop();
      this.log("MOTOR", `Connected to U2D2 at ${baudRate} baud. Initializing actuator...`, "motor");

      // STEP 1: First explicitly disarm motor (Torque Enable = 0)
      this.log("MOTOR", `[INIT 1/4] Disarming torque on actuator ID ${this.motorActiveId}...`, "motor");
      await this.setMotorTorque(false);
      await new Promise(r => setTimeout(r, 50));

      // STEP 2: Set to Multi-Turn Position Mode (Operating Mode = 4)
      this.log("MOTOR", `[INIT 2/4] Setting Operating Mode = 4 (Extended Position / Multi-turn)...`, "motor");
      const modePkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [11, 0, 4]);
      await this.sendMotorPacket(modePkt, "SET OPERATING MODE = 4");
      this.motorOperatingMode = 4;
      await new Promise(r => setTimeout(r, 50));

      // STEP 3: Set V_max according to user UI (Profile Velocity, Addr 112)
      const userSpeed = parseFloat(this.el.inputCustomSpeed ? this.el.inputCustomSpeed.value : this.CURRENT_SPEED_MM_S) || 1.0;
      this.CURRENT_SPEED_MM_S = Math.max(0.05, userSpeed);
      const velUnits = this.speedMmSToVelocityUnits(this.CURRENT_SPEED_MM_S);
      const velBytes = DXL.intToBytes(velUnits, 4);
      this.log("MOTOR", `[INIT 3/4] Setting Vmax (Profile Velocity) = ${this.CURRENT_SPEED_MM_S} mm/s (${velUnits} units)...`, "motor");
      const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
      await this.sendMotorPacket(velPkt, `SET PROFILE VELOCITY = ${velUnits}`);
      if (this.el.dispCurrentSpeed) {
        this.el.dispCurrentSpeed.textContent = `Vmax: ${this.CURRENT_SPEED_MM_S.toFixed(1)} mm/s`;
      }
      await new Promise(r => setTimeout(r, 30));

      // Set Profile Acceleration (108) for smooth trapezoidal profile (eliminates stop bounce & counter-rotation)
      const accelUnits = Math.min(32767, Math.max(80, Math.round(velUnits * 0.35)));
      const accelBytes = DXL.intToBytes(accelUnits, 4);
      const accelPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [108, 0, ...accelBytes]);
      await this.sendMotorPacket(accelPkt, `SET PROFILE ACCEL = ${accelUnits}`);
      await new Promise(r => setTimeout(r, 40));

      // STEP 4: Re-arm torque (Torque Enable = 1)
      this.log("MOTOR", `[INIT 4/4] Arming torque on actuator ID ${this.motorActiveId}...`, "motor");
      await this.setMotorTorque(true);
      await new Promise(r => setTimeout(r, 50));

      // Read current motor position and align commanded coordinate
      await this.queryMotorTelemetry();
      this.commandedPositionTicks = this.motorPresentTicks;

      // Start telemetry polling at 10 Hz (100 ms) matching WebDynamixelInterface
      this.startMotorTelemetryPolling();
      this.log("MOTOR", `✓ Actuator initialized: Multi-turn Mode (4), Vmax=${this.CURRENT_SPEED_MM_S} mm/s, Position=${this.ticksToLinearMm(this.motorPresentTicks).toFixed(2)} mm. Ready for incremental jogging!`, "motor");
    } catch (err) {
      console.error("Motor connect failed:", err);
      this.log("MOTOR", `Connection failed: ${err.message}`, "err");
      this.isMotorConnected = false;
      this.el.cardMotorPort.classList.remove('connected');
      if (this.el.dotNavMotor) this.el.dotNavMotor.classList.remove('connected');
    }
  }

  async disconnectMotor() {
    this.stopMotorTelemetryPolling();
    this.motorKeepReading = false;
    this.motorTxQueue = [];
    this.isMotorTxBusy = false;

    if (this.motorReader) {
      try { await this.motorReader.cancel(); } catch (e) {}
    }

    if (this.motorPort) {
      try { await this.motorPort.close(); } catch (e) {}
      this.motorPort = null;
    }

    this.isMotorConnected = false;
    this.el.cardMotorPort.classList.remove('connected');
    if (this.el.dotNavMotor) this.el.dotNavMotor.classList.remove('connected');
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

  // --------------------------------------------------------------------------
  // Transmission Queue & Hardware Communication
  // --------------------------------------------------------------------------
  async sendMotorPacket(packetBytes, desc = "") {
    if (!this.motorPort || !this.motorPort.writable || !this.isMotorConnected) return false;

    // Push into FIFO queue to eliminate concurrent getWriter() stream locks
    return new Promise((resolve) => {
      this.motorTxQueue.push({
        bytes: packetBytes instanceof Uint8Array ? packetBytes : new Uint8Array(packetBytes),
        desc,
        resolve
      });
      this.processMotorTxQueue();
    });
  }

  async processMotorTxQueue() {
    if (this.isMotorTxBusy || this.motorTxQueue.length === 0) return;
    this.isMotorTxBusy = true;

    while (this.motorTxQueue.length > 0) {
      const item = this.motorTxQueue.shift();
      try {
        if (!this.motorPort || !this.motorPort.writable || !this.isMotorConnected) {
          item.resolve(false);
          continue;
        }

        const writer = this.motorPort.writable.getWriter();
        await writer.write(item.bytes);
        writer.releaseLock();

        // Half-duplex turnaround delay: 20ms pause for actuator response
        await new Promise(r => setTimeout(r, 20));
        item.resolve(true);
      } catch (err) {
        console.warn("Motor send packet error:", err);
        item.resolve(false);
      }
    }

    this.isMotorTxBusy = false;
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
      }, 300);

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

      await new Promise(r => setTimeout(r, 25));

      // 2. Torque Enable (Addr 64)
      const tBlock = await this.readMotorMemory(64, 1);
      if (tBlock && tBlock.params && tBlock.params.length > 0) {
        const armed = tBlock.params[0] === 1;
        this.updateTorqueUI(armed);
      }

      await new Promise(r => setTimeout(r, 25));

      // 3. Hardware Error Status (Addr 70)
      const errBlock = await this.readMotorMemory(70, 1);
      if (errBlock && errBlock.params && errBlock.params.length > 0) {
        const errByte = errBlock.params[0];
        if (errByte !== 0) {
          this.log("MOTOR", `Actuator ID ${this.motorActiveId} Hardware Alert: 0x${errByte.toString(16).toUpperCase()}`, "err");
        }
      }

      if (notify) {
        this.log("MOTOR", `Actuator ID ${this.motorActiveId} Synced: Mode=${this.motorOperatingMode}, Torque=${this.motorTorqueEnabled ? "ON" : "OFF"}`, "motor");
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
    // Poll at 10 Hz (100 ms) matching WebDynamixelInterface
    this.motorPollTimer = setInterval(() => {
      if (
        this.isMotorConnected &&
        !this.motorIsPollingBusy &&
        !this.isMotorTxBusy &&
        this.motorTxQueue.length === 0 &&
        !this.motorActiveResolver
      ) {
        this.queryMotorTelemetry();
      }
    }, 100);
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
      // 128..131: Present Velocity (4 bytes signed int32)
      // 132..135: Present Position (4 bytes signed int32)
      const packet = DXL.buildPacketP2(this.motorActiveId, DXL.INST.READ, [128, 0, 8, 0]);
      await this.sendMotorPacket(packet);
    } finally {
      this.motorIsPollingBusy = false;
    }
  }

  /**
   * Explicitly reads fresh Present Position (Addr 132, 4 bytes) directly from motor hardware.
   * Updates motorPresentTicks and motion display before any motion command executes,
   * preventing runaway from out-of-sync or stale coordinates.
   */
  async readPresentPosition() {
    if (!this.isMotorConnected) return this.motorPresentTicks || 0;
    try {
      const res = await this.readMotorMemory(132, 4);
      if (res && res.params && res.params.length === 4) {
        this.motorPresentTicks = DXL.bytesToInt(res.params, true);
        this.updateMotionDisplay();
      }
    } catch (e) {
      console.warn("readPresentPosition query error:", e);
    }
    return this.motorPresentTicks || 0;
  }

  setSoftwareZero() {
    this.softwareZeroOffsetTicks = this.motorPresentTicks;
    const rawMm = this.ticksToLinearMm(this.motorPresentTicks);
    this.log("MOTOR", `🎯 Software 0 set at ${rawMm >= 0 ? '+' : ''}${rawMm.toFixed(2)} mm raw (${this.motorPresentTicks} ticks). Display & logging zeroed.`, "motor");
    this.updateMotionDisplay();
  }

  clearSoftwareZero() {
    this.softwareZeroOffsetTicks = 0;
    this.log("MOTOR", "Software zero offset cleared. Display reverted to hardware absolute position.", "motor");
    this.updateMotionDisplay();
  }

  getLiveDisplacementMm() {
    return this.ticksToLinearMm(this.motorPresentTicks - this.softwareZeroOffsetTicks);
  }

  updateMotionDisplay() {
    const relTicks = this.motorPresentTicks - this.softwareZeroOffsetTicks;
    const mm = this.ticksToLinearMm(relTicks);
    const revs = relTicks / this.TICKS_PER_REV;

    this.el.dispLinearMm.textContent = `${mm >= 0 ? '+' : ''}${mm.toFixed(2)} mm`;
    this.el.dispRevs.textContent = `${revs >= 0 ? '+' : ''}${revs.toFixed(2)} Revs`;
    this.el.dispTicks.textContent = `${relTicks.toLocaleString()} ticks`;

    if (this.el.dispSoftwareOffsetNote) {
      if (this.softwareZeroOffsetTicks !== 0) {
        const offsetMm = this.ticksToLinearMm(this.softwareZeroOffsetTicks);
        this.el.dispSoftwareOffsetNote.style.display = "inline";
        this.el.dispSoftwareOffsetNote.textContent = `(SW Offset: ${offsetMm >= 0 ? '+' : ''}${offsetMm.toFixed(2)} mm)`;
      } else {
        this.el.dispSoftwareOffsetNote.style.display = "none";
      }
    }
    if (this.el.btnClearSoftwareZero) {
      this.el.btnClearSoftwareZero.style.display = this.softwareZeroOffsetTicks !== 0 ? "inline-flex" : "none";
    }
  }

  /**
   * Ensures the actuator is configured in Mode 4 (Extended Position / Multi-turn)
   * with Profile Velocity set appropriately.
   */
  async ensureExtendedPositionMode() {
    if (this.motorOperatingMode !== 4) {
      this.log("MOTOR", `Switching Actuator ID ${this.motorActiveId} to Extended Position Mode (Mode 4)...`, "motor");

      // 1. Robotis Rule: Disarm torque before changing Operating Mode register
      await this.setMotorTorque(false);
      await new Promise(r => setTimeout(r, 40));

      // 2. Write Mode 4 (Extended Position)
      const modePkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [11, 0, 4]);
      await this.sendMotorPacket(modePkt, "SET OPERATING MODE = 4");
      await new Promise(r => setTimeout(r, 40));

      // 3. Set Profile Velocity (112) and Profile Acceleration (108) for smooth trapezoidal motion
      const velUnits = this.speedMmSToVelocityUnits(this.CURRENT_SPEED_MM_S);
      const velBytes = DXL.intToBytes(velUnits, 4);
      const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
      await this.sendMotorPacket(velPkt, `SET PROFILE VELOCITY = ${velUnits}`);
      await new Promise(r => setTimeout(r, 30));

      const accelUnits = Math.min(32767, Math.max(80, Math.round(velUnits * 0.35)));
      const accelBytes = DXL.intToBytes(accelUnits, 4);
      const accelPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [108, 0, ...accelBytes]);
      await this.sendMotorPacket(accelPkt, `SET PROFILE ACCEL = ${accelUnits}`);
      await new Promise(r => setTimeout(r, 30));

      // 4. Re-arm torque
      await this.setMotorTorque(true);
      await new Promise(r => setTimeout(r, 40));

      this.motorOperatingMode = 4;
      this.log("MOTOR", `Actuator ID ${this.motorActiveId} is ready in Extended Position Mode (Mode 4).`, "motor");
    } else if (!this.motorTorqueEnabled) {
      await this.setMotorTorque(true);
      await new Promise(r => setTimeout(r, 30));
    }
  }

  /**
   * Fixed Incremental Distance Jog (Relative step in mm, e.g. +1.0 mm, -0.5 mm)
   * Tracks discrete commanded position to avoid erratic drift during rapid clicks.
   * Motion executes at the hardware V_max configured by the user UI.
   */
  async jogRelativeMm(deltaMm) {
    if (!this.isMotorConnected) {
      alert("Please connect U2D2 motor port first.");
      return;
    }

    if (!this.motorTorqueEnabled) {
      await this.setMotorTorque(true);
      await new Promise(r => setTimeout(r, 30));
    }

    // Fixed incremental step in ticks (4096 ticks = 1 mm)
    const deltaTicks = Math.round(deltaMm * this.TICKS_PER_MM);

    // 1. Read fresh current position directly from motor hardware before jogging to prevent runaway
    const liveTicks = await this.readPresentPosition();

    // 2. Base the jog target strictly on the live actual physical position
    this.commandedPositionTicks = liveTicks + deltaTicks;
    const targetMm = this.commandedPositionTicks / this.TICKS_PER_MM;

    // Send Goal Position (Address 116, 4 bytes signed int32)
    const posBytes = DXL.intToBytes(this.commandedPositionTicks, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, `JOG ${deltaMm >= 0 ? '+' : ''}${deltaMm} mm`);

    this.log("MOTOR", `Jog ${deltaMm >= 0 ? '+' : ''}${deltaMm} mm from ${this.ticksToLinearMm(liveTicks).toFixed(2)} mm → Target: ${targetMm >= 0 ? '+' : ''}${targetMm.toFixed(2)} mm (${this.commandedPositionTicks} ticks @ Vmax ${this.CURRENT_SPEED_MM_S} mm/s)`, "motor");
  }

  /**
   * Continuous Jog Streaming in Multi-Turn Position Mode (Mode 4)
   * Continuously moves the goal forward/reverse as long as the user keeps pressing button.
   * Eliminates the need for a stop button and prevents counter-rotation jerk.
   */
  async startContinuousJogStream(direction) {
    if (!this.isMotorConnected) {
      alert("Please connect U2D2 motor port first.");
      return;
    }
    this.stopContinuousJogStream();

    if (!this.motorTorqueEnabled) {
      await this.setMotorTorque(true);
    }

    // Read fresh current position from motor hardware before starting continuous jog stream
    const liveTicks = await this.readPresentPosition();

    const dir = direction >= 0 ? 1 : -1;
    this.continuousJogDirection = dir;
    this.motorIsContinuousMoving = true;
    this.updateContinuousUI(dir > 0 ? "JOGGING FWD (+)" : "JOGGING REV (-)");

    // Ensure commanded coordinate tracks live present ticks
    this.commandedPositionTicks = liveTicks;

    const INTERVAL_MS = 40; // 25 Hz streaming
    this.continuousJogTimer = setInterval(() => {
      if (!this.isMotorConnected || this.continuousJogDirection === 0) {
        this.stopContinuousJogStream();
        return;
      }
      const speedMmS = Math.min(3.0, Math.max(0.05, this.CURRENT_SPEED_MM_S || 1.0));
      // Ticks to advance in this interval
      const stepTicks = Math.round(speedMmS * (INTERVAL_MS / 1000.0) * this.TICKS_PER_MM);
      this.commandedPositionTicks += dir * stepTicks;

      const posBytes = DXL.intToBytes(this.commandedPositionTicks, 4);
      const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
      this.sendMotorPacket(posPkt, `CONT JOG ${dir > 0 ? '+' : '-'}`);
    }, INTERVAL_MS);

    this.log("MOTOR", `Continuous Jog started: ${dir > 0 ? 'FORWARD (+)' : 'REVERSE (-)'} @ Vmax ${this.CURRENT_SPEED_MM_S} mm/s (streaming goal)`, "motor");
  }

  stopContinuousJogStream() {
    if (this.continuousJogTimer) {
      clearInterval(this.continuousJogTimer);
      this.continuousJogTimer = null;
    }
    if (this.continuousJogDirection !== 0 || this.motorIsContinuousMoving) {
      this.continuousJogDirection = 0;
      this.motorIsContinuousMoving = false;
      this.updateContinuousUI("STOPPED");
      // Since the streaming goal is only a tiny fraction of a millimeter ahead (<= 0.04 - 0.12 mm),
      // we do nothing to the motor on mouse release: no extra packet is sent. The actuator naturally
      // settles at the last streamed goal position smoothly without jerk or packet congestion.
      this.log("MOTOR", `Continuous Jog released at ~${this.ticksToLinearMm(this.commandedPositionTicks).toFixed(2)} mm (stream halted, no packet sent on release)`, "motor");
    }
  }

  async startContinuousJog(direction) {
    return this.startContinuousJogStream(direction);
  }

  async stopAutomatedTestMotion() {
    if (!this.isMotorConnected) return;

    // Clear any pending queued packets so STOP executes immediately with highest priority
    this.motorTxQueue = [];

    // Command Goal Position (Address 116) to current motorPresentTicks to clamp motion
    const stopTicks = this.motorPresentTicks;
    this.commandedPositionTicks = stopTicks;
    const posBytes = DXL.intToBytes(stopTicks, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, "SAFETY STOP: CLAMP GOAL TO PRESENT POSITION");

    this.log("MOTOR", `Safety stop executed: Actuator motion halted at ${this.ticksToLinearMm(stopTicks).toFixed(2)} mm (${stopTicks} ticks)`, "motor");
  }

  async stopMotionSmoothly() {
    this.stopContinuousJogStream();
    if (this.isTestRunning) {
      await this.stopAutomatedTestMotion();
    }
  }

  async haltMotion(preferredDir = null) {
    return this.stopContinuousJogStream();
  }

  updateContinuousUI(stateText) {
    if (this.el.dispContinuousState) {
      this.el.dispContinuousState.textContent = stateText;
      if (stateText.includes("FWD")) {
        this.el.dispContinuousState.className = "val-pill badge-cyan";
      } else if (stateText.includes("REV")) {
        this.el.dispContinuousState.className = "val-pill badge-purple";
      } else {
        this.el.dispContinuousState.className = "val-pill";
      }
    }

    if (this.el.btnContinuousFwd) {
      this.el.btnContinuousFwd.classList.toggle('is-jogging', stateText.includes("FWD"));
    }
    if (this.el.btnContinuousRev) {
      this.el.btnContinuousRev.classList.toggle('is-jogging', stateText.includes("REV"));
    }
  }

  async gotoOriginMm() {
    if (!this.isMotorConnected) return;
    await this.ensureExtendedPositionMode();

    const velUnits = this.speedMmSToVelocityUnits(this.CURRENT_SPEED_MM_S);
    const velBytes = DXL.intToBytes(velUnits, 4);
    const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
    await this.sendMotorPacket(velPkt, `PROFILE VEL = ${velUnits}`);

    this.commandedPositionTicks = 0;
    const posBytes = DXL.intToBytes(0, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, "GOTO 0.0 MM");

    this.log("MOTOR", `Driving carriage to 0.00 mm origin @ ${this.CURRENT_SPEED_MM_S} mm/s`, "motor");
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

      // 1. Robotis Rule: Torque MUST be disabled to execute Clear instruction (0x10)
      if (prevTorque) {
        await this.setMotorTorque(false);
        await new Promise(r => setTimeout(r, 40));
      }

      // 2. Clear Instruction: 0x10 with params [0x01, 0x44, 0x58, 0x4C, 0x22] ("DXL\"")
      const clearParams = [0x01, 0x44, 0x58, 0x4C, 0x22];
      const clearPacket = DXL.buildPacketP2(this.motorActiveId, DXL.INST.CLEAR, clearParams);
      await this.sendMotorPacket(clearPacket, "CLEAR MULTI-TURN REVOLUTIONS");

      await new Promise(r => setTimeout(r, 60));

      // 3. Re-arm torque if it was armed
      if (prevTorque) {
        await this.setMotorTorque(true);
        await new Promise(r => setTimeout(r, 40));
      }

      // Reset internal commanded coordinate tracker and software zero
      this.commandedPositionTicks = 0;
      this.softwareZeroOffsetTicks = 0;

      // 4. Query fresh position from motor
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

  async pairNewArduinoPort() {
    if (this.isArduinoConnected) {
      await this.disconnectArduino();
    }
    await this.connectArduino(true);
  }

  async connectArduino(forceSelectNew = false) {
    if (!navigator.serial) {
      alert("Web Serial API is not supported in this browser.");
      return;
    }

    try {
      let selectedPort = null;
      if (!forceSelectNew) {
        selectedPort = await this.findMatchingArduinoPort(this.motorPort);
        if (selectedPort) {
          this.log("BENCH", `Auto-selected remembered Arduino port`, "bench");
        }
      }

      if (!selectedPort) {
        // Filter specifically for Arduino Nano Bench (CH340: VID 0x1A86, PID 0x7523)
        const filters = [
          { usbVendorId: 0x1a86, usbProductId: 0x7523 },
          { usbVendorId: 0x1a86 }
        ];
        try {
          selectedPort = await navigator.serial.requestPort({ filters });
        } catch (filterErr) {
          if (filterErr.name === 'NotFoundError') throw filterErr;
          selectedPort = await navigator.serial.requestPort();
        }
      }

      this.arduinoPort = selectedPort;
      const info = this.arduinoPort.getInfo ? this.arduinoPort.getInfo() : {};
      if (info && info.usbVendorId !== undefined && info.usbProductId !== undefined) {
        this.savedArduinoVid = info.usbVendorId.toString(16).padStart(4, '0').toUpperCase();
        this.savedArduinoPid = info.usbProductId.toString(16).padStart(4, '0').toUpperCase();
        localStorage.setItem('unified_arduino_vid', this.savedArduinoVid);
        localStorage.setItem('unified_arduino_pid', this.savedArduinoPid);
        this.updateUsbPillDisplays();
        this.log("BENCH", `Paired Arduino USB: ${this.formatVidPid(this.savedArduinoVid, this.savedArduinoPid)}`, "bench");
      }

      const baudRate = this.ARDUINO_BAUD_RATE;
      await this.arduinoPort.open({ baudRate, bufferSize: 4096 });

      this.isArduinoConnected = true;
      this.arduinoKeepReading = true;
      this.el.cardArduinoPort.classList.add('connected');
      if (this.el.dotNavArduino) this.el.dotNavArduino.classList.add('connected');
      this.el.btnConnectArduino.textContent = "DISCONNECT";
      this.el.btnConnectArduino.classList.replace('btn-primary-sm', 'btn-danger-sm');

      // Start stream reading loop
      this.startArduinoReaderLoop();
      this.log("BENCH", `Connected to Arduino at ${baudRate} baud. Starting continuous telemetry...`, "bench");

      // Continuously report temp and loadcell right after connection until disconnect:
      // Note: Arduino resets on DTR pulse, which takes ~1.5 - 2s in bootloader.
      // Stagger 'REPORT ON' commands to catch both non-resetting interfaces and rebooted bootloaders.
      await new Promise(r => setTimeout(r, 300));
      await this.sendArduinoCommand("REPORT ON");

      setTimeout(() => {
        if (this.isArduinoConnected) this.sendArduinoCommand("REPORT ON");
      }, 1200);

      setTimeout(() => {
        if (this.isArduinoConnected) this.sendArduinoCommand("REPORT ON");
      }, 2200);

      // Start 1.5s keepalive watchdog to guarantee telemetry flows continuously until disconnect
      this.lastArduinoDatTimestamp = performance.now();
      this.startArduinoTelemetryWatchdog();
    } catch (err) {
      console.error("Arduino connect failed:", err);
      this.log("BENCH", `Connection failed: ${err.message}`, "err");
      this.isArduinoConnected = false;
      this.el.cardArduinoPort.classList.remove('connected');
      if (this.el.dotNavArduino) this.el.dotNavArduino.classList.remove('connected');
    }
  }

  startArduinoTelemetryWatchdog() {
    this.stopArduinoTelemetryWatchdog();
    this.arduinoReportWatchdog = setInterval(() => {
      if (this.isArduinoConnected) {
        const elapsedSinceLastDat = performance.now() - this.lastArduinoDatTimestamp;
        if (elapsedSinceLastDat > 2500) {
          // Telemetry stream silent for >2.5s -> re-assert REPORT ON
          this.sendArduinoCommand("REPORT ON");
        }
      }
    }, 1500);
  }

  stopArduinoTelemetryWatchdog() {
    if (this.arduinoReportWatchdog) {
      clearInterval(this.arduinoReportWatchdog);
      this.arduinoReportWatchdog = null;
    }
  }

  async disconnectArduino() {
    this.stopArduinoTelemetryWatchdog();
    this.arduinoKeepReading = false;

    // Gracefully send REPORT OFF before disconnecting
    if (this.isArduinoConnected) {
      try { await this.sendArduinoCommand("REPORT OFF"); } catch (e) {}
    }

    if (this.arduinoReader) {
      try { await this.arduinoReader.cancel(); } catch (e) {}
    }

    if (this.arduinoPort) {
      try { await this.arduinoPort.close(); } catch (e) {}
      this.arduinoPort = null;
    }

    this.isArduinoConnected = false;
    this.el.cardArduinoPort.classList.remove('connected');
    if (this.el.dotNavArduino) this.el.dotNavArduino.classList.remove('connected');
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
    this.lastArduinoDatTimestamp = performance.now();
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
          if (this.el.dispChamberTemp) {
            this.el.dispChamberTemp.textContent = `${this.formatSignedNumber(val, 2)} °C`;
          }
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
          if (this.el.dispChamberPwm) {
            this.el.dispChamberPwm.textContent = `(PWM ${this.livePwm} / 255)`;
          }
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

  /**
   * Format signed number with leading space ' ' placeholder for non-negative numbers
   * to eliminate horizontal display jiggle when oscillating around zero.
   */
  formatSignedNumber(val, decimals = 3) {
    if (val === null || val === undefined || isNaN(val)) {
      return ' 0.000';
    }
    const threshold = Math.pow(10, -decimals) / 2.0;
    const isNeg = val < -threshold;
    const absStr = Math.abs(val).toFixed(decimals);
    const sign = isNeg ? '-' : ' ';
    return `${sign}${absStr}`;
  }

  updateForceStats() {
    const weightStr = `${this.formatSignedNumber(this.liveWeightKg, 3)} kg`;
    const forceStr = `${this.formatSignedNumber(this.liveForceN, 3)} N`;

    const activeStatStr = this.isForceUnitNewton ? forceStr : weightStr;
    const peakStr = this.isForceUnitNewton
      ? `${this.formatSignedNumber(this.peakForceN, 3)} N`
      : `${this.formatSignedNumber(this.peakForceN / 9.80665, 3)} kg`;

    if (this.el.statLiveForce) {
      this.el.statLiveForce.textContent = `FORCE: ${activeStatStr}`;
    }
    if (this.el.statPeakForce) {
      this.el.statPeakForce.textContent = `PEAK: ${peakStr}`;
    }

    if (this.el.dispLoadcellWeight) {
      this.el.dispLoadcellWeight.textContent = weightStr;
    }
    if (this.el.dispLoadcellForce) {
      this.el.dispLoadcellForce.textContent = `(${this.formatSignedNumber(this.liveForceN, 2)} N)`;
    }
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
    if (!this.isArduinoConnected) {
      alert("Please connect Arduino Bench port first.");
      return;
    }

    const confirmed = confirm(
      "⚠️ CONFIRM LOAD CELL TARE\n\n" +
      "Are you sure you want to TARE the load cell to 0.000 kg?\n" +
      "Make sure all test samples and weights have been completely removed from the fixture.\n\n" +
      "Click OK to Zero the sensor, or Cancel to abort."
    );

    if (!confirmed) {
      this.log("BENCH", "Tare operation cancelled by user.", "bench");
      return;
    }

    this.log("BENCH", "Taring HX711 Loadcell...", "bench");
    await this.sendArduinoCommand("TARE");
  }

  async calibrateLoadcell() {
    if (!this.isArduinoConnected) {
      alert("Please connect Arduino Bench port first.");
      return;
    }

    const wStr = prompt(
      "📐 LOAD CELL CALIBRATION (STEP 1/2):\n\n" +
      "Enter the exact known test weight in kilograms (kg) that will be placed on the scale:",
      "0.500"
    );

    if (wStr === null) return; // User pressed Cancel

    const weightVal = parseFloat(wStr);
    if (isNaN(weightVal) || weightVal <= 0) {
      alert("Invalid calibration weight. Must be a positive number (e.g. 0.500).");
      return;
    }

    const confirmed = confirm(
      "⚠️ CONFIRM LOAD CELL CALIBRATION (STEP 2/2)\n\n" +
      `Please place exactly ${weightVal.toFixed(3)} kg onto the load cell fixture now.\n\n` +
      "Ensure the scale is completely stationary, then click OK to calibrate.\n" +
      "Click Cancel to abort without changes."
    );

    if (!confirmed) {
      this.log("BENCH", "Calibration cancelled by user.", "bench");
      return;
    }

    this.log("BENCH", `Calibrating load cell with known weight: ${weightVal.toFixed(3)} kg...`, "bench");
    await this.sendArduinoCommand(`CAL ${weightVal.toFixed(3)}`);
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
  // Automated Push / Pull Mechanical Test Runner & Config Modals
  // --------------------------------------------------------------------------
  openTestConfigModal(direction = 'push') {
    this.modalActiveTestDirection = direction;
    const isPush = (direction === 'push');
    const cfg = this.testConfigs[direction] || this.testConfigs.push;

    if (this.el.titleTestConfigModal) {
      this.el.titleTestConfigModal.textContent = isPush ? "PUSH TEST CONFIGURATION" : "PULL TEST CONFIGURATION";
    }
    if (this.el.noticeTestConfigDirection) {
      this.el.noticeTestConfigDirection.innerHTML = isPush
        ? `Configuring parameters for <strong>PUSH</strong> (Compression / Move toward object &minus;) mechanical test.`
        : `Configuring parameters for <strong>PULL</strong> (Tension / Move away from object +) mechanical test.`;
    }

    if (this.el.inputModalTestDistance) this.el.inputModalTestDistance.value = cfg.travelMm;
    if (this.el.inputModalTestSpeed) this.el.inputModalTestSpeed.value = cfg.speedMmS;
    if (this.el.inputModalMaxForceLimit) this.el.inputModalMaxForceLimit.value = cfg.maxForceLimitN;
    if (this.el.checkModalAutoReturn) this.el.checkModalAutoReturn.checked = !!cfg.autoReturn;
    if (this.el.checkModalAutoTare) this.el.checkModalAutoTare.checked = !!cfg.autoTare;

    if (this.el.modalTestConfig) {
      this.el.modalTestConfig.classList.add('is-open');
      this.el.modalTestConfig.setAttribute('aria-hidden', 'false');
    }
  }

  closeTestConfigModal() {
    if (this.el.modalTestConfig) {
      this.el.modalTestConfig.classList.remove('is-open');
      this.el.modalTestConfig.setAttribute('aria-hidden', 'true');
    }
  }

  saveTestConfigFromModal() {
    const dir = this.modalActiveTestDirection || 'push';
    const cfg = this.testConfigs[dir];

    if (this.el.inputModalTestDistance) {
      cfg.travelMm = parseFloat(this.el.inputModalTestDistance.value) || 10.0;
    }
    if (this.el.inputModalTestSpeed) {
      cfg.speedMmS = Math.min(3.0, Math.max(0.05, parseFloat(this.el.inputModalTestSpeed.value) || 1.0));
    }
    if (this.el.inputModalMaxForceLimit) {
      cfg.maxForceLimitN = parseFloat(this.el.inputModalMaxForceLimit.value) || 50.0;
    }
    if (this.el.checkModalAutoReturn) {
      cfg.autoReturn = this.el.checkModalAutoReturn.checked;
    }
    if (this.el.checkModalAutoTare) {
      cfg.autoTare = this.el.checkModalAutoTare.checked;
    }

    localStorage.setItem(`${dir}_travel_mm`, cfg.travelMm);
    localStorage.setItem(`${dir}_speed_mms`, cfg.speedMmS);
    localStorage.setItem(`${dir}_force_limit`, cfg.maxForceLimitN);
    localStorage.setItem(`${dir}_auto_return`, cfg.autoReturn);
    localStorage.setItem(`${dir}_auto_tare`, cfg.autoTare);

    this.updateTestParamSummaries();
    this.closeTestConfigModal();
    this.log("TEST", `Saved ${dir.toUpperCase()} test settings: ${cfg.travelMm} mm @ ${cfg.speedMmS} mm/s (Limit: ${cfg.maxForceLimitN} N)`, "test");
  }

  updateTestParamSummaries() {
    if (this.el.summaryPushParams && this.testConfigs.push) {
      this.el.summaryPushParams.textContent = `${this.testConfigs.push.travelMm.toFixed(1)} mm @ ${this.testConfigs.push.speedMmS.toFixed(1)} mm/s`;
      this.el.summaryPushParams.title = `Limit: ${this.testConfigs.push.maxForceLimitN} N | Auto-return: ${this.testConfigs.push.autoReturn ? 'ON' : 'OFF'} | Auto-tare: ${this.testConfigs.push.autoTare ? 'ON' : 'OFF'}`;
    }
    if (this.el.summaryPullParams && this.testConfigs.pull) {
      this.el.summaryPullParams.textContent = `${this.testConfigs.pull.travelMm.toFixed(1)} mm @ ${this.testConfigs.pull.speedMmS.toFixed(1)} mm/s`;
      this.el.summaryPullParams.title = `Limit: ${this.testConfigs.pull.maxForceLimitN} N | Auto-return: ${this.testConfigs.pull.autoReturn ? 'ON' : 'OFF'} | Auto-tare: ${this.testConfigs.pull.autoTare ? 'ON' : 'OFF'}`;
    }
  }

  clearTestPlot(manual = false) {
    this.testDataPoints = [];
    this.testPeakForceN = 0.0;
    this.testDispAtPeakMm = 0.0;
    this.testStiffnessNPerMm = 0.0;
    this.testEnergyMJ = 0.0;
    this.clearTestResultsDisplay();
    if (manual) {
      this.el.badgeTestState.textContent = "TEST: READY";
      this.el.badgeTestState.className = "badge-purple";
      this.el.dispTestPhase.textContent = "IDLE";
      this.log("TEST", "Push/Pull mechanical plot cleared manually.", "test");
    }
  }

  clearTestResultsDisplay() {
    if (this.el.lblResPeakForce) {
      this.el.lblResPeakForce.textContent = "PEAK FORCE";
    }
    if (this.el.resPeakForce) this.el.resPeakForce.textContent = "-- N (-- kg)";
    if (this.el.resPeakDisp) this.el.resPeakDisp.textContent = "-- mm";
    if (this.el.resStiffness) this.el.resStiffness.textContent = "-- N/mm";
    if (this.el.resEnergy) this.el.resEnergy.textContent = "-- mJ";
    if (this.el.statTestPeak) this.el.statTestPeak.textContent = "PEAK: -- N @ -- mm";
    if (this.el.statTestStiffness) this.el.statTestStiffness.textContent = "STIFFNESS: -- N/mm";
    if (this.el.btnExportCsv) this.el.btnExportCsv.disabled = true;
  }

  async startTestSequence(direction = 'push') {
    if (!this.isMotorConnected) {
      alert("Please connect U2D2 motor port first.");
      return;
    }
    if (!this.isArduinoConnected) {
      alert("Please connect Arduino Bench port to record force data.");
      return;
    }

    this.testDirection = direction; // "push" or "pull"
    const isPull = (this.testDirection === "pull");
    const cfg = this.testConfigs[direction] || this.testConfigs.push;
    this.testTravelMm = Math.max(0.1, cfg.travelMm || 10.0);
    this.testSpeedMmS = Math.min(3.0, Math.max(0.05, cfg.speedMmS || 1.0));
    this.testMaxForceLimitN = Math.max(1.0, cfg.maxForceLimitN || 50.0);
    const autoTare = cfg.autoTare;
    this.testAutoReturn = cfg.autoReturn;

    // 1. Immediately clear previous plot data, peak tracking, and results cards
    this.clearTestPlot();

    if (this.el.lblResPeakForce) {
      this.el.lblResPeakForce.textContent = isPull ? "PEAK FORCE (MIN PULL)" : "PEAK FORCE (MAX PUSH)";
    }

    this.isTestRunning = false; // Do not sample until motion actually starts
    this.el.dispTestPhase.textContent = `PREPARING (${this.testDirection.toUpperCase()})`;
    this.el.badgeTestState.textContent = `TEST: PREPARING (${this.testDirection.toUpperCase()})`;
    this.el.badgeTestState.className = "badge-emerald";
    if (this.el.rowTestButtons) this.el.rowTestButtons.style.display = "none";
    if (this.el.btnAbortTest) this.el.btnAbortTest.style.display = "flex";
    this.el.btnExportCsv.disabled = true;

    this.log("TEST", `Starting automated ${this.testDirection.toUpperCase()} test (${this.testTravelMm} mm @ ${this.testSpeedMmS} mm/s, Limit: ${this.testMaxForceLimitN} N)... Previous plot cleared.`, "test");

    // 2. Optional Auto-Tare (non-blocking direct command)
    if (autoTare) {
      this.log("TEST", "Auto-taring loadcell before motion begins...", "test");
      await this.sendArduinoCommand("TARE");
      await new Promise(r => setTimeout(r, 600));
    }

    // 3. Record initial linear position - read fresh live position directly from hardware
    const liveStartTicks = await this.readPresentPosition();
    this.testStartMm = this.ticksToLinearMm(liveStartTicks);

    // 4. Ensure Mode 4 and set test velocity with smooth acceleration profile
    await this.ensureExtendedPositionMode();
    const velUnits = this.speedMmSToVelocityUnits(this.testSpeedMmS);
    const velBytes = DXL.intToBytes(velUnits, 4);
    const velPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [112, 0, ...velBytes]);
    await this.sendMotorPacket(velPkt, `SET TEST VELOCITY = ${velUnits}`);

    // Set Profile Acceleration (108) to provide smooth trapezoidal ramp and eliminate stopping kick
    const accelUnits = Math.min(32767, Math.max(80, Math.round(velUnits * 0.35)));
    const accelBytes = DXL.intToBytes(accelUnits, 4);
    const accelPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [108, 0, ...accelBytes]);
    await this.sendMotorPacket(accelPkt, `SET TEST ACCEL = ${accelUnits}`);

    // 5. Command target displacement
    // Direction convention: Positive (+) is move away from object for PULL test;
    // Negative (-) is move toward object for PUSH test.
    const sign = isPull ? 1 : -1;
    const targetMm = this.testStartMm + (sign * this.testTravelMm);
    const targetTicks = this.linearMmToTicks(targetMm);

    const posBytes = DXL.intToBytes(targetTicks, 4);
    const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
    await this.sendMotorPacket(posPkt, `TEST MOVE TO ${targetMm.toFixed(2)} mm`);

    // 6. Motion officially starts: activate live sampling loop
    this.isTestRunning = true;
    this.el.dispTestPhase.textContent = `RUNNING (${this.testDirection.toUpperCase()})`;
    this.el.badgeTestState.textContent = `TEST: RUNNING (${this.testDirection.toUpperCase()})`;

    // Monitor motion arrival & sample real-time test data
    const checkInterval = setInterval(async () => {
      if (!this.isTestRunning) {
        clearInterval(checkInterval);
        return;
      }

      // Sample real-time data point at 20 Hz for smooth plot animation
      const nowSec = (performance.now() - this.startTime) / 1000.0;
      this.sampleMechanicalTestDataPoint(nowSec);

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
    if (!this.isTestRunning) return;

    const currMm = this.ticksToLinearMm(this.motorPresentTicks);
    const dispFromStart = Math.abs(currMm - this.testStartMm);
    const forceAbsN = Math.abs(this.liveForceN);

    // Prevent duplicate points if timestamp or displacement has not moved within 25ms
    if (this.testDataPoints.length > 0) {
      const last = this.testDataPoints[this.testDataPoints.length - 1];
      if (Math.abs(nowSec - last.t) < 0.025 && Math.abs(dispFromStart - last.dispMm) < 0.002) {
        return;
      }
    }

    this.testDataPoints.push({
      t: nowSec,
      dispMm: dispFromStart,
      forceN: this.liveForceN,
      forceAbsN: forceAbsN,
      tempC: this.liveTempC || 0
    });

    // Track min force when pull (most negative algebraic force), track max force when push (highest positive algebraic force)
    if (this.testDirection === "pull") {
      if (this.testDataPoints.length === 1 || this.liveForceN < this.testPeakForceN) {
        this.testPeakForceN = this.liveForceN;
        this.testDispAtPeakMm = dispFromStart;
      }
    } else {
      if (this.testDataPoints.length === 1 || this.liveForceN > this.testPeakForceN) {
        this.testPeakForceN = this.liveForceN;
        this.testDispAtPeakMm = dispFromStart;
      }
    }

    // Update real-time stats in header badges on every sample
    if (this.el.statTestPeak) {
      const signP = this.testPeakForceN > 0.001 ? "+" : "";
      const pkLabel = (this.testDirection === "pull") ? "MIN (PULL)" : "MAX (PUSH)";
      this.el.statTestPeak.textContent = `${pkLabel}: ${signP}${this.testPeakForceN.toFixed(2)} N @ ${this.testDispAtPeakMm.toFixed(2)} mm`;
    }

    // Update real-time stiffness slope
    if (this.testDataPoints.length >= 4) {
      const pFirst = this.testDataPoints[0];
      const pLatest = this.testDataPoints[this.testDataPoints.length - 1];
      const dx = pLatest.dispMm - pFirst.dispMm;
      if (dx > 0.05) {
        const k = Math.abs(pLatest.forceN - pFirst.forceN) / dx;
        if (k > 0 && this.el.statTestStiffness) {
          this.el.statTestStiffness.textContent = `STIFFNESS: ${k.toFixed(2)} N/mm`;
        }
      }
    }

    // Safety force limit check (triggers if absolute force magnitude exceeds limit)
    if (forceAbsN >= this.testMaxForceLimitN && this.isTestRunning) {
      this.isTestRunning = false;
      const limitSign = (this.testDirection === "pull") ? "-" : "+";
      this.log("TEST", `⚠️ SAFETY FORCE LIMIT REACHED (${this.liveForceN.toFixed(2)} N reached ${limitSign}${this.testMaxForceLimitN} N limit)! Stopping motor immediately...`, "err");
      this.stopAutomatedTestMotion().then(() => {
        this.finishTestSequence(true);
      });
      return;
    }
  }

  async finishTestSequence(safetyStopped = false) {
    this.isTestRunning = false;
    this.el.dispTestPhase.textContent = safetyStopped ? "LIMIT HALT" : "COMPLETED";
    this.el.badgeTestState.textContent = safetyStopped ? "TEST: LIMIT HALT" : "TEST: COMPLETED";
    this.el.badgeTestState.className = safetyStopped ? "badge-purple" : "badge-emerald";
    if (this.el.rowTestButtons) this.el.rowTestButtons.style.display = "grid";
    if (this.el.btnAbortTest) this.el.btnAbortTest.style.display = "none";

    // Guarantee motor is stopped immediately at current position if safety limit triggered
    if (safetyStopped) {
      await this.stopAutomatedTestMotion();
    }

    // Calculate metrics
    this.computeTestCharacterization();

    // Auto-return if requested (with 1.0s settle delay so carriage comes to complete rest before reversing)
    if (this.testAutoReturn) {
      this.log("TEST", "Auto-return requested. Pausing 1.0s for carriage to settle...", "test");
      await new Promise(r => setTimeout(r, 1000));
      if (!this.isTestRunning) {
        const startTicks = this.linearMmToTicks(this.testStartMm);
        const posBytes = DXL.intToBytes(startTicks, 4);
        const posPkt = DXL.buildPacketP2(this.motorActiveId, DXL.INST.WRITE, [116, 0, ...posBytes]);
        await this.sendMotorPacket(posPkt, "RETURN TO START");
        this.log("TEST", `Carriage returning smoothly to starting displacement (${this.testStartMm.toFixed(2)} mm)...`, "test");
      }
    }

    this.el.btnExportCsv.disabled = false;
    this.log("TEST", `✓ Test sequence finished. Recorded ${this.testDataPoints.length} mechanical data points.`, "test");
  }

  async abortTestSequence() {
    this.isTestRunning = false;
    await this.stopAutomatedTestMotion();
    this.el.dispTestPhase.textContent = "ABORTED";
    this.el.badgeTestState.textContent = "TEST: ABORTED";
    this.el.badgeTestState.className = "badge-purple";
    if (this.el.rowTestButtons) this.el.rowTestButtons.style.display = "grid";
    if (this.el.btnAbortTest) this.el.btnAbortTest.style.display = "none";
    this.log("TEST", "Push/Pull test ABORTED by user: Motor stopped immediately at present position.", "err");
    this.computeTestCharacterization();
    this.el.btnExportCsv.disabled = (this.testDataPoints.length === 0);
  }

  computeTestCharacterization() {
    if (this.testDataPoints.length < 2) return;

    let peakF = 0;
    let dispAtPeak = 0;

    // Track min force when pull (tension), track max force when push (compression)
    if (this.testDirection === "pull") {
      let minF = Infinity;
      this.testDataPoints.forEach(p => {
        if (p.forceN < minF) {
          minF = p.forceN;
          dispAtPeak = p.dispMm;
        }
      });
      peakF = (minF === Infinity) ? 0 : minF;
    } else {
      let maxF = -Infinity;
      this.testDataPoints.forEach(p => {
        if (p.forceN > maxF) {
          maxF = p.forceN;
          dispAtPeak = p.dispMm;
        }
      });
      peakF = (maxF === -Infinity) ? 0 : maxF;
    }

    this.testPeakForceN = peakF;
    this.testDispAtPeakMm = dispAtPeak;

    // Stiffness (Linear slope |ΔF| / ΔX from 20% to 80% of peak magnitude)
    const peakAbs = Math.abs(peakF);
    const fLow = peakAbs * 0.2;
    const fHigh = peakAbs * 0.8;
    const pLow = this.testDataPoints.find(p => Math.abs(p.forceN) >= fLow);
    const pHigh = this.testDataPoints.find(p => Math.abs(p.forceN) >= fHigh);

    let stiffness = 0;
    if (pLow && pHigh && Math.abs(pHigh.dispMm - pLow.dispMm) > 0.05) {
      stiffness = Math.abs(pHigh.forceN - pLow.forceN) / Math.abs(pHigh.dispMm - pLow.dispMm);
    }

    this.testStiffnessNPerMm = stiffness;

    // Energy = trapezoidal integration ∫ |F| dx (mJ = N * mm)
    let energy = 0;
    for (let i = 1; i < this.testDataPoints.length; i++) {
      const dx = Math.abs(this.testDataPoints[i].dispMm - this.testDataPoints[i - 1].dispMm);
      const avgF = (Math.abs(this.testDataPoints[i].forceN) + Math.abs(this.testDataPoints[i - 1].forceN)) / 2.0;
      energy += avgF * dx;
    }
    this.testEnergyMJ = energy;

    // Update UI with signed values and clear direction indicators
    const signPeak = this.testPeakForceN > 0.001 ? "+" : "";
    const pkTitle = (this.testDirection === "pull") ? "MIN (PULL)" : "MAX (PUSH)";
    this.el.resPeakForce.textContent = `${signPeak}${this.testPeakForceN.toFixed(2)} N (${signPeak}${(this.testPeakForceN / 9.80665).toFixed(3)} kg)`;
    this.el.resPeakDisp.textContent = `${this.testDispAtPeakMm.toFixed(2)} mm`;
    this.el.resStiffness.textContent = stiffness > 0 ? `${stiffness.toFixed(2)} N/mm` : "-- N/mm";
    this.el.resEnergy.textContent = `${energy.toFixed(1)} mJ`;

    this.el.statTestPeak.textContent = `${pkTitle}: ${signPeak}${this.testPeakForceN.toFixed(2)} N @ ${this.testDispAtPeakMm.toFixed(2)} mm`;
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
    if (window.ResizeObserver) {
      const ro = new ResizeObserver(() => this.resizeCanvases());
      const col = document.querySelector('.plot-column');
      if (col) ro.observe(col);
      document.querySelectorAll('.canvas-wrapper').forEach(w => ro.observe(w));
    }
  }

  resizeCanvases() {
    const dpr = window.devicePixelRatio || 1;
    [this.el.canvasTempPlot, this.el.canvasForcePlot, this.el.canvasPushPullPlot].forEach(c => {
      if (!c) return;
      const parent = c.parentElement;
      if (!parent) return;
      const rect = parent.getBoundingClientRect();
      if (rect.width > 0 && rect.height > 0) {
        const targetW = Math.round(rect.width * dpr);
        const targetH = Math.round(rect.height * dpr);
        if (c.width !== targetW || c.height !== targetH) {
          c.width = targetW;
          c.height = targetH;
        }
      }
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
    const parent = c.parentElement;
    if (!parent || parent.clientHeight <= 0 || parent.clientWidth <= 0) return;
    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(parent.clientWidth * dpr);
    const targetH = Math.round(parent.clientHeight * dpr);
    if (c.width !== targetW || c.height !== targetH) {
      c.width = targetW;
      c.height = targetH;
    }

    const ctx = c.getContext('2d');
    const w = c.width;
    const h = c.height;

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

  // Subplot 2: Force / Weight vs Time (Symmetric Signed Range)
  renderSubplot2Force() {
    const c = this.el.canvasForcePlot;
    if (!c) return;
    const parent = c.parentElement;
    if (!parent || parent.clientHeight <= 0 || parent.clientWidth <= 0) return;
    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(parent.clientWidth * dpr);
    const targetH = Math.round(parent.clientHeight * dpr);
    if (c.width !== targetW || c.height !== targetH) {
      c.width = targetW;
      c.height = targetH;
    }

    const ctx = c.getContext('2d');
    const w = c.width;
    const h = c.height;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#040711";
    ctx.fillRect(0, 0, w, h);

    const pad = { top: 15 * dpr, right: 15 * dpr, bottom: 20 * dpr, left: 55 * dpr };
    const pw = w - pad.left - pad.right;
    const ph = h - pad.top - pad.bottom;
    if (pw <= 0 || ph <= 0) return;

    // Y Range (Symmetric around zero for signed negative & positive forces)
    let maxAbs = this.isForceUnitNewton ? 5.0 : 0.5; // 5 N or 0.5 kg default
    this.forceHistory.forEach(d => {
      const val = Math.abs(this.isForceUnitNewton ? d.forceN : d.weightKg);
      if (val > maxAbs) maxAbs = val * 1.15;
    });
    const minY = -maxAbs;
    const maxY = maxAbs;
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
      const signStr = yVal > 0.005 ? "+" : "";
      ctx.fillText(`${signStr}${yVal.toFixed(2)} ${unitLbl}`, pad.left - 6 * dpr, y);
    }

    // Zero Reference Baseline (Cyan dashed line at 0.0)
    const zeroY = pad.top + ph - ((0.0 - minY) / yRange) * ph;
    ctx.save();
    ctx.strokeStyle = "rgba(56, 189, 248, 0.35)";
    ctx.lineWidth = 1 * dpr;
    ctx.setLineDash([4 * dpr, 4 * dpr]);
    ctx.beginPath();
    ctx.moveTo(pad.left, zeroY);
    ctx.lineTo(w - pad.right, zeroY);
    ctx.stroke();
    ctx.restore();

    if (this.forceHistory.length < 2) return;

    const nowSec = (performance.now() - this.startTime) / 1000.0;
    const minT = nowSec - this.historyDurationSec;

    // Live Force Curve (Emerald Green) - signed value
    ctx.beginPath();
    ctx.strokeStyle = "#10b981";
    ctx.lineWidth = 2.2 * dpr;
    this.forceHistory.forEach((d, i) => {
      const val = this.isForceUnitNewton ? d.forceN : d.weightKg;
      const x = pad.left + ((d.t - minT) / this.historyDurationSec) * pw;
      const y = pad.top + ph - ((val - minY) / yRange) * ph;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  // Subplot 3: Push / Pull Test (Force vs Displacement) - Real-time auto-scaling with signed forces
  renderSubplot3PushPull() {
    const c = this.el.canvasPushPullPlot;
    if (!c) return;
    const parent = c.parentElement;
    if (!parent || parent.clientHeight <= 0 || parent.clientWidth <= 0) return;
    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(parent.clientWidth * dpr);
    const targetH = Math.round(parent.clientHeight * dpr);
    if (c.width !== targetW || c.height !== targetH) {
      c.width = targetW;
      c.height = targetH;
    }

    const ctx = c.getContext('2d');
    const w = c.width;
    const h = c.height;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#040711";
    ctx.fillRect(0, 0, w, h);

    const pad = { top: 22 * dpr, right: 25 * dpr, bottom: 32 * dpr, left: 60 * dpr };
    const pw = w - pad.left - pad.right;
    const ph = h - pad.top - pad.bottom;
    if (pw <= 0 || ph <= 0) return;

    // Determine current observed min/max force and displacement for dynamic scaling
    let minObservedF = 0.0;
    let maxObservedF = 0.0;
    let maxObservedDisp = Math.max(1.0, this.testTravelMm || 5.0);

    this.testDataPoints.forEach(p => {
      const fn = p.forceN !== undefined ? p.forceN : 0;
      if (fn < minObservedF) minObservedF = fn;
      if (fn > maxObservedF) maxObservedF = fn;
      if (p.dispMm > maxObservedDisp) maxObservedDisp = p.dispMm * 1.05;
    });

    // Auto-scale Y Range:
    // If predominantly negative (pull): provide headroom downwards + small positive margin for 0 line visibility
    // If predominantly positive (push): provide headroom upwards + small negative margin for 0 line visibility
    // If both: symmetric bounds around zero
    let minY, maxY;
    if (minObservedF < -0.1 && maxObservedF > 0.1) {
      const bound = Math.max(Math.abs(minObservedF), maxObservedF, 1.0) * 1.25;
      minY = -bound;
      maxY = bound;
    } else if (minObservedF < -0.1 || this.testDirection === "pull") {
      const absLow = Math.max(Math.abs(minObservedF), 1.0) * 1.25;
      minY = -absLow;
      maxY = Math.max(0.5, maxObservedF * 1.25, absLow * 0.15);
    } else {
      const absHigh = Math.max(maxObservedF, 1.0) * 1.25;
      maxY = absHigh;
      minY = Math.min(-0.5, minObservedF * 1.25, -absHigh * 0.15);
    }

    const yRange = maxY - minY || 1;
    const maxX = maxObservedDisp;

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
      const fVal = minY + (i / ySteps) * yRange;
      const y = pad.top + ph - (i / ySteps) * ph;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(w - pad.right, y);
      ctx.stroke();
      const signStr = fVal > 0.005 ? "+" : "";
      ctx.fillText(`${signStr}${fVal.toFixed(2)} N`, pad.left - 6 * dpr, y);
    }

    // Zero Reference Baseline (Cyan dashed line)
    const zeroY = pad.top + ph - ((0.0 - minY) / yRange) * ph;
    if (zeroY >= pad.top && zeroY <= pad.top + ph) {
      ctx.save();
      ctx.strokeStyle = "rgba(56, 189, 248, 0.35)";
      ctx.lineWidth = 1.2 * dpr;
      ctx.setLineDash([4 * dpr, 4 * dpr]);
      ctx.beginPath();
      ctx.moveTo(pad.left, zeroY);
      ctx.lineTo(w - pad.right, zeroY);
      ctx.stroke();
      ctx.restore();
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
    ctx.font = `bold ${9.5 * dpr}px "Outfit", sans-serif`;
    ctx.fillStyle = "rgba(192, 132, 252, 0.85)";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText("FORCE (N)", pad.left, pad.top - 4 * dpr);

    ctx.textAlign = "right";
    ctx.textBaseline = "top";
    ctx.fillText("DISPLACEMENT (MM)", w - pad.right, pad.top + ph + 16 * dpr);

    // Live Running Indicator in Top-Right Corner
    if (this.isTestRunning) {
      ctx.save();
      const pulseAlpha = 0.6 + 0.4 * Math.sin(performance.now() / 150);
      ctx.fillStyle = `rgba(16, 185, 129, ${pulseAlpha})`;
      ctx.font = `bold ${9 * dpr}px "JetBrains Mono", monospace`;
      ctx.textAlign = "right";
      ctx.textBaseline = "middle";
      ctx.fillText(`● LIVE CAPTURE (${this.testDataPoints.length} PTS)`, w - pad.right, pad.top - 8 * dpr);
      ctx.restore();
    }

    // Render Mechanical Curve
    if (this.testDataPoints.length >= 2) {
      ctx.save();

      // 1. Smooth gradient area fill from curve to 0 baseline
      const grad = ctx.createLinearGradient(0, pad.top, 0, pad.top + ph);
      grad.addColorStop(0, "rgba(192, 132, 252, 0.22)");
      grad.addColorStop(1, "rgba(192, 132, 252, 0.02)");

      ctx.beginPath();
      const firstX = pad.left + (this.testDataPoints[0].dispMm / maxX) * pw;
      const lastX = pad.left + (this.testDataPoints[this.testDataPoints.length - 1].dispMm / maxX) * pw;
      ctx.moveTo(firstX, zeroY);
      this.testDataPoints.forEach(p => {
        const x = pad.left + (p.dispMm / maxX) * pw;
        const y = pad.top + ph - ((p.forceN - minY) / yRange) * ph;
        ctx.lineTo(x, y);
      });
      ctx.lineTo(lastX, zeroY);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();

      // 2. High-contrast neon curve
      ctx.beginPath();
      ctx.strokeStyle = "#c084fc";
      ctx.lineWidth = 2.5 * dpr;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";

      this.testDataPoints.forEach((p, i) => {
        const x = pad.left + (p.dispMm / maxX) * pw;
        const y = pad.top + ph - ((p.forceN - minY) / yRange) * ph;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      // 3. Peak Point Reticle (Min force for pull, Max force for push)
      if (Math.abs(this.testPeakForceN) > 0.001) {
        const peakX = pad.left + (this.testDispAtPeakMm / maxX) * pw;
        const peakY = pad.top + ph - ((this.testPeakForceN - minY) / yRange) * ph;
        ctx.fillStyle = "#f43f5e";
        ctx.beginPath();
        ctx.arc(peakX, peakY, 4.5 * dpr, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 1.5 * dpr;
        ctx.stroke();

        ctx.fillStyle = "#f43f5e";
        ctx.font = `600 ${8.5 * dpr}px "JetBrains Mono", monospace`;
        ctx.textAlign = (peakX > w - 85 * dpr) ? "right" : "left";
        const signPk = this.testPeakForceN > 0.001 ? "+" : "";
        const labelX = (peakX > w - 85 * dpr) ? peakX - 8 * dpr : peakX + 8 * dpr;
        const pkTitle = (this.testDirection === "pull") ? "MIN" : "MAX";
        ctx.fillText(`${pkTitle} ${signPk}${this.testPeakForceN.toFixed(2)}N`, labelX, peakY - 4 * dpr);
      }

      // 4. Live Cursor Head at Current Tip
      const pLast = this.testDataPoints[this.testDataPoints.length - 1];
      const curX = pad.left + (pLast.dispMm / maxX) * pw;
      const curY = pad.top + ph - ((pLast.forceN - minY) / yRange) * ph;

      ctx.fillStyle = "#a855f7";
      ctx.beginPath();
      ctx.arc(curX, curY, 5 * dpr, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2 * dpr;
      ctx.stroke();

      // Floating live tooltip pill
      const signTip = pLast.forceN > 0.001 ? "+" : "";
      const hudText = `${pLast.dispMm.toFixed(2)} mm • ${signTip}${pLast.forceN.toFixed(2)} N`;
      ctx.font = `600 ${9 * dpr}px "JetBrains Mono", monospace`;
      const textW = ctx.measureText(hudText).width;
      const pillW = textW + 12 * dpr;
      const pillH = 18 * dpr;
      let pillX = curX - pillW / 2;
      let pillY = curY - 24 * dpr;
      if (pillX < pad.left) pillX = pad.left;
      if (pillX + pillW > w - pad.right) pillX = w - pad.right - pillW;
      if (pillY < pad.top) pillY = curY + 10 * dpr;

      ctx.fillStyle = "rgba(15, 23, 42, 0.9)";
      ctx.strokeStyle = "rgba(192, 132, 252, 0.6)";
      ctx.lineWidth = 1 * dpr;
      ctx.beginPath();
      ctx.roundRect(pillX, pillY, pillW, pillH, 4 * dpr);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = "#f8fafc";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(hudText, pillX + pillW / 2, pillY + pillH / 2);

      ctx.restore();
    } else if (this.isTestRunning && this.testDataPoints.length === 1) {
      // First data point captured during active test
      const p = this.testDataPoints[0];
      const curX = pad.left + (p.dispMm / maxX) * pw;
      const curY = pad.top + ph - ((p.forceN - minY) / yRange) * ph;
      ctx.fillStyle = "#a855f7";
      ctx.beginPath();
      ctx.arc(curX, curY, 5 * dpr, 0, Math.PI * 2);
      ctx.fill();
    } else if (this.isTestRunning) {
      // Initializing state right after test start
      ctx.fillStyle = "rgba(16, 185, 129, 0.8)";
      ctx.font = `600 ${10 * dpr}px "Outfit", sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      const dirUpper = this.testDirection ? this.testDirection.toUpperCase() : "TEST";
      ctx.fillText(`STARTING ${dirUpper} TEST • ACQUIRING BASELINE...`, pad.left + pw / 2, pad.top + ph / 2);
    } else {
      // Clean static awaiting state when not running a test
      ctx.fillStyle = "rgba(255, 255, 255, 0.22)";
      ctx.font = `600 ${10 * dpr}px "Outfit", sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText('AWAITING TEST SEQUENCE • PRESS "PUSH TEST" OR "PULL TEST" TO BEGIN', pad.left + pw / 2, pad.top + ph / 2);
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
