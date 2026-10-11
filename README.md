# Web Force Bench - Web Serial Monitor

A client-side static web application enabling direct hardware communication with serial devices using the browser's native **Web Serial API**. Designed to be deployed seamlessly on **GitHub Pages**.

## Features

- 🔌 **Web Serial Connection**: Connect to USB serial devices with configurable baud rates (standard 9600 to 921600+ or custom), data bits, stop bits, and parity.
- 🔍 **Hardware VID & PID Detection**: Displays Vendor ID (VID) and Product ID (PID) in hexadecimal notation upon device selection, with automatic identification for known hardware vendors (Arduino, ESP32, STM32, Raspberry Pi RP2040, FTDI, CP210x, CH340, etc.).
- 💻 **Real-Time Serial Monitor**: Full bidirectional serial monitor terminal with timestamps, autoscroll toggle, TX/RX visual differentiation, and HEX inspection mode.
- ⌨️ **Command Transmission & History**: Input box with configurable line endings (`\r\n`, `\n`, `\r`, or None) and terminal history navigation (Arrow Up / Down).
- 📊 **Telemetry Stats & Export**: Real-time tracking of received and transmitted bytes / packets, plus one-click log export.
- 🚀 **100% Client-Side**: No backend required. Works directly out-of-the-box on GitHub Pages over HTTPS.

## Requirements

- **Supported Browsers**: Google Chrome (v89+), Microsoft Edge (v89+), Opera, or Chromium-based browsers with Web Serial API enabled.
- **Protocol**: HTTPS (required by Web Serial API specification in production) or `localhost` / `file://` during local testing.

## Local Usage

Simply serve the directory with any static web server:

```bash
# Using Python
python -m http.server 8000

# Using Node.js
npx serve .
```

Open `http://localhost:8000` in Google Chrome or Edge.

## GitHub Pages Deployment

1. Push this repository to GitHub.
2. Go to **Settings** > **Pages**.
3. Under **Build and deployment**, select **Deploy from a branch**.
4. Choose the `main` (or `master`) branch and `/ (root)` folder, then click **Save**.
5. Your serial monitor will be live at `https://<username>.github.io/<repo-name>/`.
