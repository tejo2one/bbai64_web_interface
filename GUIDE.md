# BeagleBone AI-64 Web Teleoperation & Face Engine Deployment Guide

## 1. Prerequisites & Dependencies

On the BeagleBone AI-64 host (Debian Bullseye aarch64):
```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev v4l-utils
```

---

## 2. Preventing Display Collisions & Screen Glitching

To grant the native C++ vector engine exclusive, glitch-free control of `/dev/fb0`:

### Step 1: Permanently Disable Desktop Display Manager (LightDM)
By default, LightDM runs an Xorg X11 server that overwrites `/dev/fb0`. Disable it:
```bash
sudo systemctl stop lightdm
sudo systemctl disable lightdm
```

### Step 2: Prevent Screen DPMS Energy-Saving Sleep
To keep the panel awake during operation:
```bash
echo 0 | sudo tee /sys/class/graphics/fb0/blank
```

---

## 3. High-Speed RAM tmpfs Deployment Pattern

To eliminate Flash wear and ensure crash-proof operation:
- The persistent master copy resides in `/opt/robot_teleop/`.
- On boot, files are synchronized into `/dev/shm/web_teleop/` (a 2.0 GB in-memory `tmpfs`), where compilation and runtime execution occur.

---

## 4. Automatic Boot Service Configuration

### Step 1: Install Robot Teleoperation Service
```bash
sudo mkdir -p /opt/robot_teleop
sudo cp -r * /opt/robot_teleop/
sudo cp service/robot-teleop.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot-teleop.service
sudo systemctl start robot-teleop.service
```

### Step 2: Configure Passwordless Safe Poweroff
To allow the web interface to trigger `/bin/systemctl poweroff` without an interactive password prompt:
```bash
sudo cp service/99-robot-poweroff /etc/sudoers.d/99-robot-poweroff
sudo chmod 0440 /etc/sudoers.d/99-robot-poweroff
```

### Step 3: Install the Active Cooling Daemon
```bash
sudo mkdir -p /opt/bbai64_fan_control
sudo cp service/bbai64_fan_control.py /opt/bbai64_fan_control/
sudo cp service/bbai64-fan-control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable bbai64-fan-control.service
sudo systemctl start bbai64-fan-control.service
```

---

## 5. Verifying Installation

### Check Service Status:
```bash
sudo systemctl status robot-teleop.service --no-pager
sudo systemctl status bbai64-fan-control.service --no-pager
```

### View Live Service Journal:
```bash
journalctl -u robot-teleop.service -f
```

### Accessing the Web Cockpit:
Point your browser on any phone, tablet, or PC to:
- **Dashboard**: `http://<your-robot-ip>:5000/`
- **Robot Face Mirror**: `http://<your-robot-ip>:5000/face`
- **MJPEG Camera Stream**: `http://<your-robot-ip>:5000/video_feed`

---

## 6. Remote Deployment from Development Machine

Use the deployment script in `scripts/deploy_to_bbai64.py`:

```bash
# Set environment variables for target board
export BBAI64_IP="bbai.local"        # or your board IP
export BBAI64_USER="debian"
export BBAI64_PASSWORD="<your-password>"

# Deploy and rebuild
python3 scripts/deploy_to_bbai64.py
```
This transfers updated sources, triggers remote compilation with `-O3`, synchronizes to `/opt/robot_teleop/`, and restarts `robot-teleop.service` with zero downtime.
