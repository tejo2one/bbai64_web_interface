#!/usr/bin/env python3
"""
Deployment script to build and sync the Web Teleoperation & Face Engine to BeagleBone AI-64.
Reads connection parameters from environment variables (BBAI64_IP, BBAI64_USER, BBAI64_PASSWORD).
"""

import os
import sys
import getpass
import paramiko
import time

def deploy():
    target_ip = os.environ.get('BBAI64_IP', 'bbai.local')
    username = os.environ.get('BBAI64_USER', 'debian')
    password = os.environ.get('BBAI64_PASSWORD')

    if not password:
        password = getpass.getpass(f"Enter SSH password for {username}@{target_ip}: ")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print(f"[1/5] Connecting to BBAI-64 ({username}@{target_ip})...")
    client.connect(target_ip, username=username, password=password, timeout=15)

    # 1. Prepare directory structure in /dev/shm (RAM tmpfs for zero SD card wear)
    print("[2/5] Preparing /dev/shm/web_teleop directories...")
    cmd = 'python3 -c "import os\nfor p in [\'/dev/shm/web_teleop/src\', \'/dev/shm/web_teleop/include\', \'/dev/shm/web_teleop/templates\', \'/dev/shm/web_teleop/static\']:\n os.makedirs(p, exist_ok=True)\n"'
    stdin, stdout, stderr = client.exec_command(cmd)
    stdout.channel.recv_exit_status()

    # 2. Upload source files
    print("[3/5] Uploading sources, templates, and static assets...")
    sftp = client.open_sftp()
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    sftp.put(os.path.join(base_dir, 'Makefile'), '/dev/shm/web_teleop/Makefile')
    sftp.put(os.path.join(base_dir, 'src', 'web_teleop_server.cpp'), '/dev/shm/web_teleop/src/web_teleop_server.cpp')
    sftp.put(os.path.join(base_dir, 'include', 'httplib.h'), '/dev/shm/web_teleop/include/httplib.h')

    for tmpl in ['index.html', 'robot_face.html']:
        local_p = os.path.join(base_dir, 'templates', tmpl)
        if os.path.exists(local_p):
            sftp.put(local_p, f'/dev/shm/web_teleop/templates/{tmpl}')

    static_local = os.path.join(base_dir, 'static')
    if os.path.exists(static_local):
        for root, dirs, files in os.walk(static_local):
            rel_dir = os.path.relpath(root, static_local).replace('\\', '/')
            remote_dir = '/dev/shm/web_teleop/static' if rel_dir == '.' else f'/dev/shm/web_teleop/static/{rel_dir}'
            try:
                sftp.mkdir(remote_dir)
            except Exception:
                pass
            for f in files:
                lp = os.path.join(root, f)
                rp = f"{remote_dir}/{f}"
                sftp.put(lp, rp)

    sftp.close()
    client.close()
    print("      Files transferred successfully.")

    # 3. Compile in RAM tmpfs
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(target_ip, username=username, password=password, timeout=60)

    print("[4/5] Compiling web_teleop_server on BBAI-64 (g++ -O3 -pthread)...")
    compile_cmd = 'cd /dev/shm/web_teleop && g++ -O3 -std=c++17 -Wall -Wextra -pthread -I./include -I/usr/include/opencv4 -o web_teleop_server src/web_teleop_server.cpp -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui -pthread'
    stdin, stdout, stderr = client.exec_command(compile_cmd)
    code = stdout.channel.recv_exit_status()

    if code != 0:
        print("[-] Build failed! Output:")
        print(stderr.read().decode())
        client.close()
        return False

    print("      Compilation successful.")

    # 4. Sync to /opt/robot_teleop/ and restart service
    print("[5/5] Updating /opt/robot_teleop and restarting robot-teleop.service...")
    sync_cmd = f'echo "{password}" | sudo -S systemctl stop robot-teleop.service && echo "{password}" | sudo -S cp -rf /dev/shm/web_teleop/* /opt/robot_teleop/ && echo "{password}" | sudo -S systemctl start robot-teleop.service'
    stdin, stdout, stderr = client.exec_command(sync_cmd)
    stdout.channel.recv_exit_status()
    time.sleep(2)

    # Check status
    stdin, stdout, stderr = client.exec_command('systemctl is-active robot-teleop.service')
    status = stdout.read().decode().strip()
    print(f"      Service status: {status}")

    client.close()
    print("[SUCCESS] Deployment completed successfully!")
    return True

if __name__ == '__main__':
    deploy()
