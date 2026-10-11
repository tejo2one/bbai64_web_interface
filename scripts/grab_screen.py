#!/usr/bin/env python3
"""
Captures the raw Linux framebuffer (/dev/fb0) from BeagleBone AI-64 and saves as PNG.
"""

import os
import sys
import getpass
import paramiko
from PIL import Image

def grab_screen():
    target_ip = os.environ.get('BBAI64_IP', 'bbai.local')
    username = os.environ.get('BBAI64_USER', 'debian')
    password = os.environ.get('BBAI64_PASSWORD')

    if not password:
        password = getpass.getpass(f"Enter SSH password for {username}@{target_ip}: ")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(target_ip, username=username, password=password, timeout=10)

    # Dump /dev/fb0 to /dev/shm/fb0.raw
    client.exec_command('cat /dev/fb0 > /dev/shm/fb0.raw')
    
    sftp = client.open_sftp()
    local_raw = 'fb0_dump.raw'
    sftp.get('/dev/shm/fb0.raw', local_raw)
    sftp.close()
    client.close()

    # Convert 1024x600 32bpp BGRA raw to image
    with open(local_raw, 'rb') as f:
        raw_data = f.read()

    width, height = 1024, 600
    img = Image.frombytes('RGBA', (width, height), raw_data, 'raw', 'BGRA')
    img.save('screen_capture.png')
    # Save 180 rotated version (physical display mounting orientation)
    img_rot = img.rotate(180)
    img_rot.save('screen_capture_rot180.png')
    
    if os.path.exists(local_raw):
        os.remove(local_raw)

    print("[OK] Captured display: screen_capture.png & screen_capture_rot180.png")

if __name__ == '__main__':
    grab_screen()
