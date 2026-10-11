# BBAI-64 Autonomous Teleoperation Cockpit & Cybernetic Robot Face Engine

> **High-Performance C++17 Robot Web Server, Direct POSIX Serial Bridge, Hardware Framebuffer Face Engine & Kinetic HUD**  
> Engineered for BeagleBone AI-64 (Texas Instruments TDA4VM dual Cortex-A72 @ 2.0 GHz) and 1024x600 LCD Panel (`/dev/fb0`).

---

## 1. System Overview & Architecture

This repository contains the native C++ web server and browser-based teleoperation cockpit that provides real-time robot control, camera streaming, hardware telemetry diagnostics, and an onboard tear-free cybernetic face engine for the BeagleBone AI-64.

```
                      +-------------------------------------------------------------+
                      |                  Remote Operator Devices                    |
                      |            (Browser on PC, Tablet, or Smartphone)           |
                      |                                                             |
                      |    - Kinetic Virtual Joystick (2WD Curvature Mixing)        |
                      |    - 30 FPS Live MJPEG Camera Feed (/video_feed)            |
                      |    - Slide-out Telemetry Drawer (Encoders, Speed, TCS)      |
                      |    - Remote Safe Shutdown Modal (/api/shutdown)             |
                      +------------------------------+------------------------------+
                                                     |
                                                     | HTTP / REST (Port 5000)
                                                     v
+-----------------------------------------------------------------------------------+
|                        BeagleBone AI-64 Linux Host                                |
|                                                                                   |
|  +-----------------------------------------------------------------------------+  |
|  |                 Native C++17 Web Server (`web_teleop_server`)               |  |
|  |                                                                             |  |
|  |  [HTTP REST Engine]     [POSIX Serial Bridge]      [V4L2 Camera Engine]     |  |
|  |  cpp-httplib Server     Direct termios @ 57600     Auto-detects USB cam     |  |
|  |  Routes: /api/teleop    /dev/ttyUSB0 (Arduino)     30 FPS 640x480 MJPEG     |  |
|  |  Routes: /api/status    Non-blocking background    Fallback test pattern    |  |
|  |  Routes: /api/shutdown  Thread-safe mutex buffer                            |  |
|  |                                                                             |  |
|  |  +-----------------------------------------------------------------------+  |  |
|  |  |             Native OpenCV Vector Framebuffer Engine (/dev/fb0)        |  |  |
|  |  |    - Direct kernel mmap with hardware VSync (FBIO_WAITFORVSYNC)       |  |  |
|  |  |    - 180° inverted rotation for physical panel mounting               |  |  |
|  |  |    - Animated Cybernetic Visor, Ears, Blinking Eyes, and Cheeks       |  |  |
|  |  |    - Real-time Picture-in-Picture (PiP) Live Camera Stream            |  |  |
|  |  |    - Tactical Diagnostic Telemetry (Speed, Motor PWM, TCS, Link)      |  |  |
|  |  +-----------------------------------------------------------------------+  |  |
|  +-----------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------+
             |                                              |
             v                                              v
+-----------------------------+               +-----------------------------+
|  Physical LCD Panel         |               |  Arduino Nano & Motors      |
|  1024x600 32bpp on /dev/fb0 |               |  Cytron MDD10A on D5/D6/D10/D9
+-----------------------------+               +-----------------------------+
```

---

## 2. Key Features

- **Sub-Millisecond Latency**: 100% native C++17 server bypassing Python interpreter and GIL overhead.
- **Hardware Framebuffer Vector Face Engine**: Direct memory-mapped (`mmap`) rendering to `/dev/fb0` synchronized to the display vertical blanking interval (`ioctl(FBIO_WAITFORVSYNC)`). Guarantees **zero tearing, zero flicker, and zero glitching**.
- **180° Inversion Handling**: Automatically corrects for inverted physical LCD panel mounting directly in hardware RAM (`cv::flip(canvas, -1)`).
- **Kinetic Web Cockpit (`/`)**:
  - Virtual 2WD analog joystick with exponential throttle curve ($y^{1.25}$) and differential steering curvature.
  - Three speed selector tiers: `SLOW` (80 PWM), `NORM` (130 PWM), `FAST` (210 PWM).
  - Emergency Stop button (<kbd>Spacebar</kbd> / touch).
  - Slide-out telemetry drawer showing live encoder counts, wheel delta ticks, PWM outputs, and TCS slip flags.
- **V4L2 Auto-Isolating Camera Engine**: Scans `/dev/v4l/by-id` and filters real USB capture devices, bypassing TI hardware codec nodes (`/dev/video0` and `/dev/video1`).
- **Remote Safe Shutdown (`/api/shutdown`)**: Double-confirmation power-down button that safely zeroes drive motors, dispatches dual filesystem `sync()` flushes, and invokes `/bin/systemctl poweroff` without requiring an SSH terminal.
- **Intelligent Multi-Zone Thermal Daemon**: Userspace cooling controller tracking 5 SoC thermal zones (`wkup`, `mpu`, `c7x`, `gpu`, `r5f`) to prevent thermal throttling.

---

## 3. Directory Structure

```
bbai64_web_interface/
|-- Makefile                         # High-performance g++ -O3 build script
|-- CMakeLists.txt                   # CMake alternative build definition
|-- launch_robot.sh                  # Boot startup script for systemd
|-- start_server_and_face.sh         # Local kiosk and test runner
|-- README.md                        # Documentation
|-- GUIDE.md                         # Service installation & deployment guide
|-- .gitignore                       # Git ignore filters
|-- src/
|   `-- web_teleop_server.cpp        # Unified C++17 server & framebuffer engine
|-- include/
|   `-- httplib.h                    # Embedded multithreaded HTTP/REST library
|-- templates/
|   |-- index.html                   # Operator Cockpit Web Dashboard
|   `-- robot_face.html              # Browser-mirrored Cyber Face Viewer
|-- static/
|   |-- css/teleop.css               # Neon-tactical dark theme styling
|   |-- js/teleop.js                 # Virtual joystick & telemetry polling
|   `-- img/                         # UI icons and graphic assets
|-- service/
|   |-- robot-teleop.service         # Systemd service unit for autostart
|   |-- bbai64_fan_control.py        # Multi-zone thermal cooling daemon
|   |-- bbai64-fan-control.service   # Systemd service for fan daemon
|   `-- 99-robot-poweroff            # Sudoers privilege rule for shutdown
`-- scripts/
    |-- deploy_to_bbai64.py          # Secure deployment script
    `-- grab_screen.py               # Framebuffer screenshot capture tool
```

---

## 4. REST API Endpoint Reference

| Endpoint | Method | Parameters | Description |
| :--- | :---: | :--- | :--- |
| `/` | `GET` | none | Serves the main Operator Cockpit (`index.html`) |
| `/face` | `GET` | none | Serves the browser Face Mirror (`robot_face.html`) |
| `/video_feed` | `GET` | none | Streams 30 FPS MJPEG multipart video feed |
| `/api/teleop` | `POST` | `{"left_pwm": int, "right_pwm": int}` | Commands proportional wheel drive (-220 to 220) |
| `/api/status` | `GET` | none | Returns JSON status (encoders, motors, speed, link) |
| `/api/thermal` | `GET` | none | Returns JSON temperatures across all SoC zones |
| `/api/shutdown` | `POST` | none | Cuts motor torque, syncs disks, and powers off board |

---

## 5. Quick Build & Run

### Compilation on BeagleBone AI-64:
```bash
make
```

### Run Server on Port 5000:
```bash
./web_teleop_server 5000
```
Open **`http://<your-robot-ip>:5000`** in any browser.

---

## 6. License

Released under the **BSD-3-Clause License**.
