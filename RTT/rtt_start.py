# rtt_start.py
import subprocess
import time
import sys
import os
import signal

OPENOCD_CMD = [
    r'C:\OpenOCD\xpack-openocd-0.12.0-7\bin\openocd.exe',
    '-f', 'interface/cmsis-dap.cfg',
    '-f', 'target/stm32h7x.cfg',
    '-c', 'adapter speed 8000',
    '-c', 'rtt setup 0x20000000 0x20000 "SEGGER RTT"',
    '-c', 'init',
    '-c', 'reset run',
    '-c', 'rtt server start 60001 0',
    '-c', 'rtt start',
]


openocd_proc = None

def cleanup():
    if openocd_proc:
        print("\nShutting down OpenOCD...")
        openocd_proc.kill()
        openocd_proc.wait()
        print("Done.")

try:
    # Kill any leftover OpenOCD
    os.system('taskkill /F /IM openocd.exe > nul 2>&1')
    time.sleep(1)

    # Start OpenOCD
    print("=" * 50)
    print("  RTT Capture - Auto Launch")
    print("=" * 50)
    print("\n[1/3] Starting OpenOCD + RTT server...")

    openocd_proc = subprocess.Popen(
        OPENOCD_CMD,
        stdout=open('openocd_log.txt', 'w'),
        stderr=subprocess.STDOUT,
    )

    # Wait for OpenOCD to be ready
    print("[2/3] Waiting for RTT server...")
    time.sleep(2)

    # Check OpenOCD is still running
    if openocd_proc.poll() is not None:
        print("ERROR: OpenOCD exited unexpectedly. Check openocd_log.txt")
        sys.exit(1)

    # Start capture script
    print("[3/3] Starting capture...\n")
    capture_proc = subprocess.Popen([sys.executable, 'rtt_capture.py'])
    capture_proc.wait()

except KeyboardInterrupt:
    print("\n\nStopping...")
finally:
    cleanup()