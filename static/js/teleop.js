/**
 * ============================================================================
 * Autonomous Kinetic HUD - Teleoperation & Telemetry Engine
 * ============================================================================
 * - 2WD Differential Analog Virtual Joystick with Proportional Mixing
 * - Dynamic Reticle Horizon Deflection & Vector HUD Telemetry
 * - Realtime USB Webcam & Arduino Serial Hardware Diagnostics
 * - High-Priority Instant E-STOP & WASD/Arrow Keyboard Fallback
 * - 100% Self-Contained, Zero Cloud Dependencies
 * ============================================================================
 */

document.addEventListener('DOMContentLoaded', () => {
  // -------------------------------------------------------------------------
  // CONFIGURATION & DRIVE STATE
  // -------------------------------------------------------------------------
  const speeds = {
    'slow': { pwm: 80,  lin: 0.20, ang: 0.8, label: '0.2 m/s (80 PWM)' },
    'norm': { pwm: 130, lin: 0.50, ang: 1.6, label: '0.5 m/s (130 PWM)' },
    'fast': { pwm: 210, lin: 1.00, ang: 2.8, label: '1.0 m/s (210 PWM)' }
  };

  let currentMode = 'norm';
  let maxPwm = speeds[currentMode].pwm;
  let maxLinSpeed = speeds[currentMode].lin;
  let maxAngSpeed = speeds[currentMode].ang;

  let isEStopped = false;
  let isDragging = false;
  let activePointerId = null;

  let currentX = 0; // -1 to +1 (steer: -1 left, +1 right)
  let currentY = 0; // -1 to +1 (throttle: -1 reverse, +1 forward)

  let lastSentLeft = 0;
  let lastSentRight = 0;
  let teleopInterval = null;

  // -------------------------------------------------------------------------
  // DOM REFERENCES
  // -------------------------------------------------------------------------
  const boundary = document.getElementById('joystick-boundary');
  const knob = document.getElementById('joystick-knob');
  const knobDot = document.getElementById('knob-dot');

  const telemetryVx = document.getElementById('telemetry-vx');
  const telemetryWz = document.getElementById('telemetry-wz');
  const vectorArrow = document.getElementById('vector-arrow');
  const velReadout = document.getElementById('vel-readout');

  const estopBtn = document.getElementById('estop-btn');
  const speedButtons = document.querySelectorAll('.speed-btn');

  // Status indicators
  const pingText = document.getElementById('ping-text');
  const connStatusTitle = document.getElementById('conn-status-title');
  const connIndicatorDot = document.getElementById('conn-indicator-dot');
  const connPingRing = document.getElementById('conn-ping-ring');

  const camStatusText = document.getElementById('cam-status-text');
  const camStatusDot = document.getElementById('cam-status-dot');
  const serialStatusText = document.getElementById('serial-status-text');
  const serialStatusDot = document.getElementById('serial-status-dot');
  const tcsStatusChip = document.getElementById('tcs-status-chip');
  const tcsStatusDot = document.getElementById('tcs-status-dot');
  const tcsStatusText = document.getElementById('tcs-status-text');
  const streamFormatLabel = document.getElementById('stream-format-label');

  // Diagnostics drawer
  const telemetryDrawer = document.getElementById('telemetry-drawer');
  const btnToggleTelemetry = document.getElementById('btn-toggle-telemetry');
  const btnCloseTelemetry = document.getElementById('btn-close-telemetry');
  const diagLeftEnc = document.getElementById('diag-left-enc');
  const diagRightEnc = document.getElementById('diag-right-enc');
  const diagLeftPwm = document.getElementById('diag-left-pwm');
  const diagRightPwm = document.getElementById('diag-right-pwm');
  const diagTcsStatus = document.getElementById('diag-tcs-status');
  const diagTcsPill = document.getElementById('diag-tcs-pill');
  const diagWebcamNode = document.getElementById('diag-webcam-node');
  const diagSerialPort = document.getElementById('diag-serial-port');

  // Aux buttons
  const btnToggleHeadlight = document.getElementById('btn-toggle-headlight');
  const btnCaptureSnapshot = document.getElementById('btn-capture-snapshot');
  const onboardStream = document.getElementById('onboard-stream');

  // Video resolution & display mode controls
  const resButtons = document.querySelectorAll('.res-btn');
  const resSelect = document.getElementById('resolution-select');
  const liveResBadge = document.getElementById('live-res-badge');
  const btnToggleDisplayMode = document.getElementById('btn-toggle-display-mode');
  let currentDisplayMode = 'fit'; // 'fit' or 'native1x'
  let currentResolution = '640x480';
  let isSwitchingResolution = false;

  // -------------------------------------------------------------------------
  // REST API MOTOR DISPATCH (RATE-LIMITED & DE-BOUNCED)
  // -------------------------------------------------------------------------
  let isDispatching = false;
  let pendingMotorCommand = null;
  let lastDispatchTime = 0;
  const MIN_DISPATCH_INTERVAL_MS = 50; // Max 20 Hz teleop dispatch to protect WiFi & serial

  async function flushPendingMotorCommand() {
    if (isDispatching || !pendingMotorCommand || isEStopped) return;

    const target = pendingMotorCommand;
    pendingMotorCommand = null;

    // Skip if identical to last sent command
    if (target.left === lastSentLeft && target.right === lastSentRight) {
      return;
    }

    isDispatching = true;
    lastDispatchTime = Date.now();
    lastSentLeft = target.left;
    lastSentRight = target.right;

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 1200);

    try {
      await fetch('/api/teleop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        signal: controller.signal,
        body: JSON.stringify({
          action: 'analog',
          left_pwm: target.left,
          right_pwm: target.right
        })
      });
    } catch (e) {
      // Handled gracefully
    } finally {
      clearTimeout(timeoutId);
      isDispatching = false;
      if (pendingMotorCommand) {
        setTimeout(flushPendingMotorCommand, 10);
      }
    }
  }

  async function sendMotorCommand(leftPwm, rightPwm) {
    if (isEStopped) return;

    pendingMotorCommand = { left: leftPwm, right: rightPwm };
    if (isDispatching) return;

    const now = Date.now();
    const elapsed = now - lastDispatchTime;
    if (elapsed < MIN_DISPATCH_INTERVAL_MS) {
      setTimeout(flushPendingMotorCommand, MIN_DISPATCH_INTERVAL_MS - elapsed);
      return;
    }

    await flushPendingMotorCommand();
  }

  async function sendStopCommand() {
    pendingMotorCommand = null;
    lastSentLeft = 0;
    lastSentRight = 0;
    try {
      await fetch('/api/stop', { method: 'POST' });
    } catch (e) {
      // Ignore
    }
  }

  // -------------------------------------------------------------------------
  // 2WD DIFFERENTIAL DRIVE MIXING
  // -------------------------------------------------------------------------
  // 2WD CURVATURE DIFFERENTIAL DRIVE MIXING
  // -------------------------------------------------------------------------
  function computeAndDispatchDrive() {
    if (isEStopped) return;

    if (Math.abs(currentX) < 0.05 && Math.abs(currentY) < 0.05) {
      if (lastSentLeft !== 0 || lastSentRight !== 0) {
        sendStopCommand();
      }
      return;
    }

    // Steering deadband: lock pure forward and reverse straight vectors
    let turnX = currentX;
    if (Math.abs(turnX) < 0.08) {
      turnX = 0;
    }

    let left, right;
    const throttleSign = currentY >= 0 ? 1 : -1;
    // Exponential throttle curve for fine low-speed docking and powerful high-speed drive
    const throttle = throttleSign * Math.pow(Math.abs(currentY), 1.25) * maxPwm;

    if (Math.abs(currentY) < 0.10) {
      // Stationary Spin / Pivot Turn on the spot
      const spinSpeed = Math.round(turnX * maxPwm * 0.85);
      left = spinSpeed;
      right = -spinSpeed;
    } else {
      // Curvature Steering: inner wheel speed is reduced proportionally to turn
      // Allowing smooth arc turns without wheel stalls or saturation
      if (turnX > 0) {
        // Turning RIGHT: Left wheel is outer (full throttle), Right wheel is inner (slowed)
        left = Math.round(throttle);
        right = Math.round(throttle * (1.0 - 1.4 * turnX));
      } else if (turnX < 0) {
        // Turning LEFT: Right wheel is outer (full throttle), Left wheel is inner (slowed)
        left = Math.round(throttle * (1.0 + 1.4 * turnX));
        right = Math.round(throttle);
      } else {
        // Straight
        left = Math.round(throttle);
        right = Math.round(throttle);
      }
    }

    // Clamp to [-maxPwm, maxPwm]
    left = Math.max(-maxPwm, Math.min(maxPwm, left));
    right = Math.max(-maxPwm, Math.min(maxPwm, right));

    sendMotorCommand(left, right);
  }

  // -------------------------------------------------------------------------
  // HUD VECTOR & RETICLE DYNAMICS
  // -------------------------------------------------------------------------
  function updateVectorDisplay(normX, normY) {
    if (isEStopped) {
      telemetryVx.textContent = 'HALTED';
      telemetryWz.textContent = 'HALTED';
      if (vectorArrow) vectorArrow.style.transform = 'scale(0)';
      return;
    }

    const linVel = normY * maxLinSpeed;
    const angVel = -normX * maxAngSpeed;

    const signLin = linVel >= 0 ? '+' : '';
    const signAng = angVel >= 0 ? '+' : '';

    telemetryVx.textContent = `${signLin}${linVel.toFixed(2)}m/s`;
    telemetryWz.textContent = `${signAng}${angVel.toFixed(2)}r/s`;

    // Rotate and scale directional vector arrow
    const mag = Math.hypot(normX, normY);
    if (mag > 0.08) {
      const angleDeg = Math.atan2(-normX, normY) * (180 / Math.PI);
      vectorArrow.style.transform = `rotate(${angleDeg}deg) scale(${Math.min(1.3, 0.6 + mag * 0.7)})`;
      vectorArrow.style.opacity = '1';
    } else {
      vectorArrow.style.transform = 'scale(0)';
      vectorArrow.style.opacity = '0';
    }

  }

  function setKnobPosition(px, py, animate = false) {
    if (animate) {
      knob.style.transition = 'transform 0.22s cubic-bezier(0.18, 0.89, 0.32, 1.28)';
    } else {
      knob.style.transition = 'none';
    }
    knob.style.transform = `translate(${px}px, ${py}px)`;
  }

  function handleJoystickMove(clientX, clientY) {
    const rect = boundary.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    let dx = clientX - centerX;
    let dy = clientY - centerY;

    const maxRadius = (rect.width / 2) - 24;
    const dist = Math.hypot(dx, dy);

    if (dist > maxRadius) {
      const angle = Math.atan2(dy, dx);
      dx = Math.cos(angle) * maxRadius;
      dy = Math.sin(angle) * maxRadius;
    }

    currentX = dx / maxRadius;
    currentY = -dy / maxRadius; // Screen Y is downward, cartesian Y is upward

    setKnobPosition(dx, dy, false);
    updateVectorDisplay(currentX, currentY);
    computeAndDispatchDrive();
  }

  function resetJoystick() {
    isDragging = false;
    activePointerId = null;
    currentX = 0;
    currentY = 0;
    knob.classList.remove('dragging');
    if (knobDot) knobDot.classList.remove('scale-125');
    setKnobPosition(0, 0, true);
    updateVectorDisplay(0, 0);
    sendStopCommand();
  }

  // -------------------------------------------------------------------------
  // POINTER & TOUCH EVENT LISTENERS
  // -------------------------------------------------------------------------
  boundary.addEventListener('pointerdown', (e) => {
    if (isEStopped) return;
    isDragging = true;
    activePointerId = e.pointerId;
    knob.classList.add('dragging');
    if (knobDot) knobDot.classList.add('scale-125');
    boundary.setPointerCapture(e.pointerId);
    handleJoystickMove(e.clientX, e.clientY);

    // Continuous keepalive while held
    if (!teleopInterval) {
      teleopInterval = setInterval(computeAndDispatchDrive, 80);
    }
  });

  boundary.addEventListener('pointermove', (e) => {
    if (!isDragging || e.pointerId !== activePointerId) return;
    handleJoystickMove(e.clientX, e.clientY);
  });

  const stopDragHandler = (e) => {
    if (e.pointerId === activePointerId) {
      if (teleopInterval) {
        clearInterval(teleopInterval);
        teleopInterval = null;
      }
      resetJoystick();
    }
  };

  boundary.addEventListener('pointerup', stopDragHandler);
  boundary.addEventListener('pointercancel', stopDragHandler);

  // -------------------------------------------------------------------------
  // KEYBOARD FALLBACK (WASD & ARROWS)
  // -------------------------------------------------------------------------
  const keysPressed = {};
  let keyLoopInterval = null;

  function handleKeyVector() {
    if (isDragging || isEStopped) return;
    let kx = 0;
    let ky = 0;

    if (keysPressed['KeyW'] || keysPressed['ArrowUp']) ky += 1;
    if (keysPressed['KeyS'] || keysPressed['ArrowDown']) ky -= 1;
    if (keysPressed['KeyA'] || keysPressed['ArrowLeft']) kx -= 1;
    if (keysPressed['KeyD'] || keysPressed['ArrowRight']) kx += 1;

    if (kx !== 0 || ky !== 0) {
      const mag = Math.hypot(kx, ky);
      kx = kx / mag;
      ky = ky / mag;

      const rect = boundary.getBoundingClientRect();
      const maxRadius = (rect.width / 2) - 24;
      const px = kx * maxRadius * 0.85;
      const py = -ky * maxRadius * 0.85;

      currentX = kx * 0.85;
      currentY = ky * 0.85;
      setKnobPosition(px, py, false);
      updateVectorDisplay(currentX, currentY);
      computeAndDispatchDrive();
    } else if (!isDragging && (currentX !== 0 || currentY !== 0)) {
      resetJoystick();
    }
  }

  window.addEventListener('keydown', (e) => {
    if (e.code === 'Space') {
      e.preventDefault();
      if (estopBtn) estopBtn.click();
      return;
    }

    if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowLeft', 'ArrowDown', 'ArrowRight'].includes(e.code)) {
      keysPressed[e.code] = true;
      if (!keyLoopInterval) {
        keyLoopInterval = setInterval(handleKeyVector, 30);
      }
    }
  });

  window.addEventListener('keyup', (e) => {
    if (keysPressed[e.code]) {
      delete keysPressed[e.code];
      const anyActive = Object.keys(keysPressed).length > 0;
      if (!anyActive) {
        if (keyLoopInterval) {
          clearInterval(keyLoopInterval);
          keyLoopInterval = null;
        }
        if (!isDragging) {
          resetJoystick();
        }
      }
    }
  });

  // -------------------------------------------------------------------------
  // SPEED TIER SELECTOR
  // -------------------------------------------------------------------------
  speedButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      speedButtons.forEach(b => {
        b.className = 'speed-btn px-4 py-1.5 rounded-full text-xs font-semibold tracking-wider text-slate-400 hover:text-white transition-colors font-sans';
      });
      btn.className = 'speed-btn px-4 py-1.5 rounded-full text-xs font-bold tracking-wider bg-[#00ff9d] text-black shadow-[0_0_12px_rgba(0,255,157,0.4)] transition-all font-sans';

      currentMode = btn.getAttribute('data-mode');
      if (speeds[currentMode]) {
        maxPwm = speeds[currentMode].pwm;
        maxLinSpeed = speeds[currentMode].lin;
        maxAngSpeed = speeds[currentMode].ang;
        if (velReadout && !isEStopped) {
          velReadout.textContent = speeds[currentMode].label;
        }
        updateVectorDisplay(currentX, currentY);
      }
    });
  });

  // -------------------------------------------------------------------------
  // EMERGENCY STOP (E-STOP) ACTUATOR
  // -------------------------------------------------------------------------
  if (estopBtn) {
    estopBtn.addEventListener('click', () => {
      isEStopped = true;
      resetJoystick();
      sendStopCommand();

      estopBtn.classList.add('halted');
      if (velReadout) {
        velReadout.textContent = 'HALTED (E-STOP LOCKED)';
        velReadout.classList.remove('text-[#00ff9d]');
        velReadout.classList.add('text-red-500');
      }
      telemetryVx.textContent = 'HALTED';
      telemetryWz.textContent = 'HALTED';
      telemetryVx.classList.add('text-red-400');
      telemetryWz.classList.add('text-red-400');

      // Auto-unlock throttle after 2.5 seconds
      setTimeout(() => {
        isEStopped = false;
        estopBtn.classList.remove('halted');
        if (velReadout) {
          velReadout.textContent = speeds[currentMode].label;
          velReadout.classList.remove('text-red-500');
          velReadout.classList.add('text-[#00ff9d]');
        }
        telemetryVx.classList.remove('text-red-400');
        telemetryWz.classList.remove('text-red-400');
        updateVectorDisplay(0, 0);
      }, 2500);
    });
  }

  // -------------------------------------------------------------------------
  // AUX BUTTONS: HEADLIGHT & SNAPSHOT CAPTURE
  // -------------------------------------------------------------------------
  if (btnToggleHeadlight) {
    let headlightOn = false;
    btnToggleHeadlight.addEventListener('click', () => {
      headlightOn = !headlightOn;
      btnToggleHeadlight.classList.toggle('headlight-active', headlightOn);
    });
  }

  if (btnCaptureSnapshot && onboardStream) {
    btnCaptureSnapshot.addEventListener('click', () => {
      try {
        const canvas = document.createElement('canvas');
        canvas.width = onboardStream.naturalWidth || 640;
        canvas.height = onboardStream.naturalHeight || 480;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(onboardStream, 0, 0, canvas.width, canvas.height);

        const a = document.createElement('a');
        a.href = canvas.toDataURL('image/jpeg', 0.9);
        a.download = `bbai64_snapshot_${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
        a.click();
      } catch (err) {
        console.warn('Snapshot capture failed:', err);
      }
    });
  }

  // -------------------------------------------------------------------------
  // CAMERA RESOLUTION SELECTION & DYNAMIC DISPLAY ENGINE
  // -------------------------------------------------------------------------
  async function setCameraResolution(width, height) {
    if (isSwitchingResolution) return;
    isSwitchingResolution = true;

    const resKey = `${width}x${height}`;
    currentResolution = resKey;

    // Update dropdown selector value
    if (resSelect) {
      resSelect.value = resKey;
    }

    // Update button active classes immediately for responsive feedback (if any)
    resButtons.forEach(btn => {
      if (btn.getAttribute('data-res') === resKey) {
        btn.className = 'res-btn active px-2.5 sm:px-3 py-1 rounded-full text-[10px] sm:text-xs font-bold tracking-wider bg-[#00ff9d] text-black shadow-[0_0_12px_rgba(0,255,157,0.4)] transition-all font-mono';
      } else {
        btn.className = 'res-btn px-2.5 sm:px-3 py-1 rounded-full text-[10px] sm:text-xs font-medium tracking-wider text-slate-400 hover:text-white transition-colors font-mono';
      }
    });

    if (liveResBadge) {
      liveResBadge.textContent = `${resKey} [CONFIGURING...]`;
      liveResBadge.className = 'hidden md:inline-block text-[10px] font-mono font-bold text-amber-300 bg-amber-950/60 border border-amber-500/30 px-2 py-0.5 rounded-full animate-pulse';
    }

    try {
      const resp = await fetch('/api/camera/resolution', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ width: width, height: height })
      });
      const data = await resp.json();
      const actualW = data.width || width;
      const actualH = data.height || height;
      const actualFps = data.fps || 30;

      // Reconnect MJPEG video stream so browser updates dimensions seamlessly
      if (onboardStream) {
        onboardStream.style.opacity = '0.6';
        setTimeout(() => {
          onboardStream.src = `/video_feed?t=${Date.now()}`;
          onboardStream.style.opacity = '1';
        }, 150);
      }

      if (resSelect) {
        resSelect.value = `${actualW}x${actualH}`;
      }

      if (liveResBadge) {
        liveResBadge.textContent = `${actualW}x${actualH}`;
        liveResBadge.className = 'hidden md:inline-block text-[10px] font-mono font-bold text-[#00ff9d] bg-[#00ff9d]/10 border border-[#00ff9d]/30 px-2 py-0.5 rounded-full';
      }

      if (streamFormatLabel) {
        streamFormatLabel.textContent = `${actualW}x${actualH} // ${actualFps} FPS // V4L2 USB UVC`;
      }

      console.log(`[CAMERA RES] Dynamic resolution applied: ${actualW}x${actualH} @ ${actualFps} FPS`);
    } catch (err) {
      console.warn('[CAMERA RES] Failed to change camera resolution:', err);
      if (liveResBadge) {
        liveResBadge.textContent = resKey;
        liveResBadge.className = 'text-[10px] font-mono font-bold text-red-400 bg-red-950/60 border border-red-500/30 px-2 py-0.5 rounded-full';
      }
    } finally {
      setTimeout(() => { isSwitchingResolution = false; }, 400);
    }
  }

  // Bind change listener to resolution dropdown menu
  if (resSelect) {
    resSelect.addEventListener('change', () => {
      const val = resSelect.value;
      const parts = val.split('x');
      if (parts.length === 2) {
        const w = parseInt(parts[0], 10);
        const h = parseInt(parts[1], 10);
        if (w && h) {
          setCameraResolution(w, h);
        }
      }
    });
  }

  // Bind click listener to each resolution option (if buttons exist)
  resButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      const w = parseInt(btn.getAttribute('data-w'), 10);
      const h = parseInt(btn.getAttribute('data-h'), 10);
      if (w && h) {
        setCameraResolution(w, h);
      }
    });
  });

  // Display Mode Switcher (Scaled Contain vs 1:1 Actual Hardware Pixels)
  if (btnToggleDisplayMode && onboardStream) {
    btnToggleDisplayMode.addEventListener('click', () => {
      if (currentDisplayMode === 'fit') {
        currentDisplayMode = 'native1x';
        onboardStream.classList.remove('camera-feed-fit');
        onboardStream.classList.add('camera-feed-native-1x');
        btnToggleDisplayMode.textContent = 'MODE: 1:1 NATIVE';
        btnToggleDisplayMode.className = 'text-[10px] font-mono text-[#00ff9d] hover:text-[#00ff9d]/80 underline underline-offset-2 transition-colors cursor-pointer';
      } else {
        currentDisplayMode = 'fit';
        onboardStream.classList.remove('camera-feed-native-1x');
        onboardStream.classList.add('camera-feed-fit');
        btnToggleDisplayMode.textContent = 'MODE: FIT';
        btnToggleDisplayMode.className = 'text-[10px] font-mono text-cyan-400 hover:text-cyan-300 underline underline-offset-2 transition-colors cursor-pointer';
      }
    });
  }

  // -------------------------------------------------------------------------
  // TELEMETRY DRAWER TOGGLE
  // -------------------------------------------------------------------------
  if (btnToggleTelemetry && telemetryDrawer) {
    btnToggleTelemetry.addEventListener('click', () => {
      telemetryDrawer.classList.toggle('hidden');
    });
  }
  if (btnCloseTelemetry && telemetryDrawer) {
    btnCloseTelemetry.addEventListener('click', () => {
      telemetryDrawer.classList.add('hidden');
    });
  }

  // -------------------------------------------------------------------------
  // REAL-TIME HARDWARE & WEBCAM TELEMETRY POLLER (200ms)
  // -------------------------------------------------------------------------
  let consecutiveStatusFailures = 0;

  async function pollHardwareTelemetry() {
    try {
      const t0 = performance.now();
      const resp = await fetch('/api/status');
      if (!resp.ok) throw new Error('Status failed');
      const data = await resp.json();
      const latency = Math.round(performance.now() - t0);
      consecutiveStatusFailures = 0;

      // Latency & Connection
      pingText.textContent = `${latency} ms`;
      connStatusTitle.textContent = 'ONLINE';
      connIndicatorDot.className = 'w-2 h-2 rounded-full bg-[#00ff9d]';
      connPingRing.style.display = 'block';

      // USB Webcam Status Update
      if (data.usb_webcam) {
        const cam = data.usb_webcam;
        if (cam.connected) {
          camStatusDot.className = 'w-2 h-2 rounded-full bg-[#00ff9d]';
          camStatusText.textContent = `CAM: USB UVC (${cam.device_node || 'ACTIVE'})`;
          if (!isSwitchingResolution && cam.width && cam.height) {
            currentResolution = `${cam.width}x${cam.height}`;
            if (liveResBadge && !liveResBadge.classList.contains('animate-pulse')) {
              liveResBadge.textContent = currentResolution;
            }
            if (streamFormatLabel) {
              streamFormatLabel.textContent = `${cam.width}x${cam.height} // ${cam.fps || 30} FPS // V4L2 USB UVC`;
            }
          }
          diagWebcamNode.textContent = `${cam.device_node} - ${cam.card_name}`;
          diagWebcamNode.className = 'text-green-400 font-medium text-[10px] break-all';
        } else {
          camStatusDot.className = 'w-2 h-2 rounded-full bg-yellow-400';
          camStatusText.textContent = 'CAM: VIRTUAL HUD';
          streamFormatLabel.textContent = '640x480 // 30 FPS // SYNTHETIC HUD';
          diagWebcamNode.textContent = 'None detected (/dev/video0/1 are TI codecs)';
          diagWebcamNode.className = 'text-yellow-400 font-medium text-[10px] break-all';
        }
      }

      // Serial Link Status Update
      const activeSerialPort = data.serial_port || (data.telemetry && data.telemetry.port) || '/dev/ttyUSB0';
      if (data.serial_connected) {
        serialStatusDot.className = 'w-2 h-2 rounded-full bg-[#00ff9d]';
        serialStatusText.textContent = 'SERIAL: CONNECTED';
        diagSerialPort.textContent = `${activeSerialPort} @ 57600 baud (ACTIVE)`;
        diagSerialPort.className = 'text-green-400 font-medium text-[10px]';
      } else {
        serialStatusDot.className = 'w-2 h-2 rounded-full bg-yellow-400';
        serialStatusText.textContent = 'SERIAL: CONNECTING';
        diagSerialPort.textContent = `${activeSerialPort} (RECONNECTING...)`;
        diagSerialPort.className = 'text-yellow-400 font-medium text-[10px]';
      }

      // Wi-Fi Link Status Update
      const wifiStatusText = document.getElementById('wifi-status-text');
      const wifiStatusIcon = document.getElementById('wifi-status-icon');
      const diagWifiSsid = document.getElementById('diag-wifi-ssid');
      const diagWifiSignal = document.getElementById('diag-wifi-signal');
      if (data.wifi && wifiStatusText) {
        if (data.wifi.connected) {
          wifiStatusText.textContent = `WIFI: ${data.wifi.ssid}`;
          if (wifiStatusIcon) wifiStatusIcon.className = 'w-3.5 h-3.5 text-[#00ff9d]';
          if (diagWifiSsid) {
            diagWifiSsid.textContent = `${data.wifi.ssid} (${data.wifi.ip || 'DHCP'})`;
            diagWifiSsid.className = 'text-green-400 font-medium text-[10px]';
          }
          if (diagWifiSignal) {
            diagWifiSignal.textContent = `${data.wifi.signal_dbm} dBm (${data.wifi.quality}%)`;
            diagWifiSignal.className = 'text-[#00ff9d] font-bold text-[10px]';
          }
        } else {
          wifiStatusText.textContent = 'WIFI: DISCONNECTED';
          if (wifiStatusIcon) wifiStatusIcon.className = 'w-3.5 h-3.5 text-yellow-400';
          if (diagWifiSsid) {
            diagWifiSsid.textContent = 'DISCONNECTED';
            diagWifiSsid.className = 'text-yellow-400 font-medium text-[10px]';
          }
          if (diagWifiSignal) {
            diagWifiSignal.textContent = '--';
            diagWifiSignal.className = 'text-slate-500 font-medium text-[10px]';
          }
        }
      }

      // Encoders and PWM
      if (data.telemetry) {
        const tel = data.telemetry;
        if (diagLeftEnc) diagLeftEnc.textContent = tel.left_enc !== undefined ? tel.left_enc : '--';
        if (diagRightEnc) diagRightEnc.textContent = tel.right_enc !== undefined ? tel.right_enc : '--';
        if (diagLeftPwm) diagLeftPwm.textContent = tel.left_pwm !== undefined ? `${tel.left_pwm}` : '--';
        if (diagRightPwm) diagRightPwm.textContent = tel.right_pwm !== undefined ? `${tel.right_pwm}` : '--';

        // Update dedicated desktop side wing telemetry
        const dLenc = document.getElementById('desktop-l-enc');
        const dRenc = document.getElementById('desktop-r-enc');
        const dLpwm = document.getElementById('desktop-l-pwm');
        const dRpwm = document.getElementById('desktop-r-pwm');
        if (dLenc) dLenc.textContent = tel.left_enc !== undefined ? tel.left_enc : '--';
        if (dRenc) dRenc.textContent = tel.right_enc !== undefined ? tel.right_enc : '--';
        if (dLpwm) dLpwm.textContent = tel.left_pwm !== undefined ? `${tel.left_pwm}` : '--';
        if (dRpwm) dRpwm.textContent = tel.right_pwm !== undefined ? `${tel.right_pwm}` : '--';

        // Update mobile ticker strip
        const mobSerialText = document.getElementById('mob-serial-text');
        const mobSerialDot = document.getElementById('mob-serial-dot');
        const mobTcsText = document.getElementById('mob-tcs-text');
        const mobTcsDot = document.getElementById('mob-tcs-dot');
        const mobWifiText = document.getElementById('mob-wifi-text');
        if (mobSerialText) mobSerialText.textContent = data.serial_connected ? 'SER: OK' : 'SER: --';
        if (mobSerialDot) mobSerialDot.className = data.serial_connected ? 'w-1.5 h-1.5 rounded-full bg-[#00ff9d]' : 'w-1.5 h-1.5 rounded-full bg-yellow-400';
        if (mobWifiText && data.wifi) mobWifiText.textContent = data.wifi.connected ? `WIFI: ${data.wifi.ssid}` : 'WIFI: OFF';

        // Traction Control (TCS) Status Evaluation
        if (tel.tcs) {
          const tcs = tel.tcs;
          if (tcsStatusChip && tcsStatusDot && tcsStatusText) {
            tcsStatusChip.classList.remove('tcs-slip-active', 'tcs-guard-active');
            if (tcs.guard) {
              tcsStatusChip.classList.add('tcs-guard-active');
              tcsStatusDot.className = 'w-2 h-2 rounded-full bg-red-500 animate-ping';
              tcsStatusText.textContent = 'TCS: GUARD [CLAMPED]';
              if (diagTcsStatus) diagTcsStatus.textContent = 'GUARD: RUNAWAY PREVENTED';
              if (diagTcsPill) {
                diagTcsPill.className = 'text-[9px] px-2 py-0.5 rounded bg-red-500/20 text-red-400 border border-red-500/30 font-mono';
                diagTcsPill.textContent = 'INTERVENING';
              }
            } else if (tcs.active) {
              tcsStatusChip.classList.add('tcs-slip-active');
              tcsStatusDot.className = 'w-2 h-2 rounded-full bg-amber-400';
              const side = tcs.slip_left && tcs.slip_right ? 'DUAL' : (tcs.slip_left ? 'LEFT' : 'RIGHT');
              tcsStatusText.textContent = `TCS: SLIP [${side}]`;
              if (diagTcsStatus) diagTcsStatus.textContent = `e-LSD SLIP CUT (${side})`;
              if (diagTcsPill) {
                diagTcsPill.className = 'text-[9px] px-2 py-0.5 rounded bg-amber-400/20 text-amber-300 border border-amber-400/30 font-mono';
                diagTcsPill.textContent = 'INTERVENING';
              }
            } else {
              tcsStatusDot.className = 'w-2 h-2 rounded-full bg-[#00ff9d]';
              tcsStatusText.textContent = 'TCS: STABLE';
              if (diagTcsStatus) diagTcsStatus.textContent = 'STABLE (e-LSD ENGAGED)';
              if (diagTcsPill) {
                diagTcsPill.className = 'text-[9px] px-2 py-0.5 rounded bg-[#00ff9d]/20 text-[#00ff9d] border border-[#00ff9d]/30 font-mono';
                diagTcsPill.textContent = 'ACTIVE';
              }
            }
          }
        }
      }

    } catch (e) {
      consecutiveStatusFailures++;
      pingText.textContent = '-- ms';
      if (consecutiveStatusFailures < 4) {
        connStatusTitle.textContent = 'LINK LAG';
        connIndicatorDot.className = 'w-2 h-2 rounded-full bg-yellow-400 animate-pulse';
      } else {
        connStatusTitle.textContent = 'OFFLINE';
        connIndicatorDot.className = 'w-2 h-2 rounded-full bg-red-500';
        connIndicatorDot.classList.remove('animate-pulse');
        connPingRing.style.display = 'none';
        serialStatusDot.className = 'w-2 h-2 rounded-full bg-yellow-400';
        serialStatusText.textContent = 'SERIAL: RECONNECTING';
        if (mobSerialText) mobSerialText.textContent = 'SER: --';
        if (mobSerialDot) mobSerialDot.className = 'w-1.5 h-1.5 rounded-full bg-yellow-400';
      }
    }
  }

  // -------------------------------------------------------------------------
  // GRACEFUL ROBOT SHUTDOWN MODAL & CONTROLLER
  // -------------------------------------------------------------------------
  const btnOpenShutdown = document.getElementById('btn-open-shutdown');
  const btnCancelShutdown = document.getElementById('btn-cancel-shutdown');
  const btnConfirmShutdown = document.getElementById('btn-confirm-shutdown');
  const shutdownModal = document.getElementById('shutdown-modal');
  const shutdownProgress = document.getElementById('shutdown-progress');
  const shutdownStatusMsg = document.getElementById('shutdown-status-msg');
  const shutdownModalActions = document.getElementById('shutdown-modal-actions');

  if (btnOpenShutdown && shutdownModal) {
    btnOpenShutdown.addEventListener('click', () => {
      shutdownModal.classList.remove('hidden');
      if (shutdownProgress) shutdownProgress.classList.add('hidden');
      if (shutdownModalActions) shutdownModalActions.classList.remove('hidden');
    });
  }

  if (btnCancelShutdown && shutdownModal) {
    btnCancelShutdown.addEventListener('click', () => {
      shutdownModal.classList.add('hidden');
    });
  }

  if (btnConfirmShutdown) {
    btnConfirmShutdown.addEventListener('click', async () => {
      if (shutdownModalActions) shutdownModalActions.classList.add('hidden');
      if (shutdownProgress) shutdownProgress.classList.remove('hidden');
      if (shutdownStatusMsg) shutdownStatusMsg.textContent = 'Commanding motors halt & syncing filesystems...';

      try {
        const resp = await fetch('/api/shutdown', { method: 'POST' });
        const resData = await resp.json();
        if (shutdownStatusMsg) {
          shutdownStatusMsg.textContent = 'System shutdown dispatched! Safe to flip power switch in 5 seconds.';
        }
      } catch (err) {
        if (shutdownStatusMsg) {
          shutdownStatusMsg.textContent = 'Shutdown command sent. System is powering down now.';
        }
      }
    });
  }

  // Initial poll & 200ms periodic timer
  pollHardwareTelemetry();
  setInterval(pollHardwareTelemetry, 220);
});
