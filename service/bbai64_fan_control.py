#!/usr/bin/env python3
"""
BeagleBone AI-64 Intelligent Active Cooling Daemon
Monitors all on-chip SoC thermal zones (Cortex-A72 MPU, C7x DSP, PowerVR GPU, R5F MCU, WKUP)
and drives the PWM cooling fan with smooth multi-tier hysteresis.
"""

import os
import sys
import time
import signal
import json
import glob

POLICY_FILE = "/sys/class/thermal/thermal_zone0/policy"
COOLING_DEV = "/sys/class/thermal/cooling_device0/cur_state"
HWMON_PWM   = "/sys/class/hwmon/hwmon1/pwm1"
HWMON_RPM   = "/sys/class/hwmon/hwmon1/fan1_input"
STATUS_FILE = "/dev/shm/thermal_status.json"

# Fan states (matching DT cooling levels: 0=100, 1=140, 2=180, 3=220, 4=254)
# Thresholds (deg C):
# < 42°C: State 1 (PWM 140, ~55% - quiet baseline)
# >= 45°C: State 2 (PWM 180, ~70% - moderate)
# >= 52°C: State 3 (PWM 220, ~86% - high cooling)
# >= 60°C: State 4 (PWM 254, 100% - maximum airflow)
FAN_PROFILES = [
    {"state": 1, "pwm": 140, "trigger_temp": 0.0,  "lower_temp": 0.0},
    {"state": 2, "pwm": 180, "trigger_temp": 45.0, "lower_temp": 42.0},
    {"state": 3, "pwm": 220, "trigger_temp": 52.0, "lower_temp": 49.0},
    {"state": 4, "pwm": 254, "trigger_temp": 60.0, "lower_temp": 56.0},
]

running = True

def handle_exit(signum, frame):
    global running
    print(f"\n[FAN-DAEMON] Caught signal {signum}, restoring kernel step_wise governor...")
    running = False

def set_kernel_policy(policy_name):
    try:
        if os.path.exists(POLICY_FILE):
            with open(POLICY_FILE, 'w') as f:
                f.write(policy_name)
    except Exception as e:
        print(f"[WARN] Failed to set policy to {policy_name}: {e}")

def set_fan_state(state):
    try:
        with open(COOLING_DEV, 'w') as f:
            f.write(str(state))
    except Exception as e:
        print(f"[ERROR] Failed to set fan state {state}: {e}")

def read_fan_rpm():
    try:
        if os.path.exists(HWMON_RPM):
            return int(open(HWMON_RPM).read().strip())
    except Exception:
        pass
    return 0

def read_thermal_zones():
    temps = {}
    for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        try:
            ztype_file = os.path.join(z, 'type')
            temp_file = os.path.join(z, 'temp')
            if os.path.exists(temp_file):
                t_raw = int(open(temp_file).read().strip())
                # Discard invalid/uninitialized readings (e.g. -32768)
                if t_raw > 0:
                    ztype = open(ztype_file).read().strip() if os.path.exists(ztype_file) else os.path.basename(z)
                    temps[ztype] = t_raw / 1000.0
        except Exception:
            continue
    return temps

def determine_fan_state(max_temp, current_state):
    # Determine target state using hysteresis
    target_state = current_state
    
    # Check upward transitions
    for prof in reversed(FAN_PROFILES):
        if max_temp >= prof["trigger_temp"]:
            if prof["state"] > target_state:
                target_state = prof["state"]
            break
            
    # Check downward transitions with hysteresis
    for prof in reversed(FAN_PROFILES):
        if target_state == prof["state"]:
            if max_temp < prof["lower_temp"] and prof["state"] > 1:
                target_state = prof["state"] - 1
            break

    return max(1, min(4, target_state))

def main():
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    print("==========================================================")
    print("  🌀 BeagleBone AI-64 Intelligent Fan Control Daemon")
    print("==========================================================")

    # Take ownership of cooling governor
    set_kernel_policy("user_space")

    # Initial 1-second pulse at max speed to clear dust and verify tachometer
    print("[FAN-DAEMON] Initializing fan tachometer pulse (State 4)...")
    set_fan_state(4)
    time.sleep(1.0)

    current_state = 4
    last_log_state = current_state

    print(f"[FAN-DAEMON] Active cooling online. Monitoring thermal zones...")

    while running:
        temps = read_thermal_zones()
        if not temps:
            # Failsafe: max speed if reading fails
            set_fan_state(4)
            current_state = 4
            time.sleep(2.0)
            continue

        # Exclude wifi from max_temp if it exists, focus on SoC cores
        soc_temps = {k: v for k, v in temps.items() if 'iwl' not in k.lower()}
        max_temp = max(soc_temps.values()) if soc_temps else max(temps.values())
        highest_zone = max(soc_temps, key=soc_temps.get) if soc_temps else "unknown"

        new_state = determine_fan_state(max_temp, current_state)
        if new_state != current_state:
            set_fan_state(new_state)
            rpm = read_fan_rpm()
            print(f"[FAN] State change: {current_state} -> {new_state} (Max: {max_temp:.1f}°C on {highest_zone}, RPM: {rpm})")
            current_state = new_state
            last_log_state = current_state

        rpm = read_fan_rpm()

        # Publish live telemetry to /dev/shm
        status = {
            "max_temp_c": round(max_temp, 1),
            "highest_zone": highest_zone,
            "fan_state": current_state,
            "fan_rpm": rpm,
            "zones": {k: round(v, 1) for k, v in temps.items()},
            "timestamp": time.time()
        }
        try:
            with open(STATUS_FILE + ".tmp", 'w') as f:
                json.dump(status, f)
            os.replace(STATUS_FILE + ".tmp", STATUS_FILE)
        except Exception:
            pass

        time.sleep(2.0)

    # Graceful exit
    print("[FAN-DAEMON] Shutting down, reverting to kernel step_wise governor...")
    set_fan_state(2)
    set_kernel_policy("step_wise")
    print("[FAN-DAEMON] Exited cleanly.")

if __name__ == '__main__':
    main()
