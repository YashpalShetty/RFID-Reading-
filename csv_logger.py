import serial
import csv
import time
import sys
from datetime import datetime
from pathlib import Path

DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 115200

DOWNLOADS_DIR = Path.home() / "Downloads"
DOWNLOADS_DIR.mkdir(parents=True, exist_ok=True)

# e.g. moisture_rfid_log_2026-09-30_14-32-05.csv
OUTPUT_PATH = DOWNLOADS_DIR / f"moisture_rfid_log_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"


def wait_for_ready(ser, timeout=15):
    """Wait until the ESP32 prints the CSV header (setup finished)."""
    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"[ESP32 STATUS] {line}")
        if line.startswith("timestamp_ms"):
            return True
    return False

def start_inventory(ser, retries=3):
    """Send 'g' and wait for the ESP32 to confirm."""
    for attempt in range(retries):
        ser.write(b'g\n')
        ser.flush()
        end = time.time() + 2
        while time.time() < end:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[ESP32 STATUS] {line}")
            if "Starting inventory" in line:
                return True
        print(f"No confirmation, retrying ({attempt + 1}/{retries})...")
    return False

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    print(f"Connecting to {port} at {DEFAULT_BAUD} baud...")

    try:
        ser = serial.Serial(port, DEFAULT_BAUD, timeout=1)  # normal open: lets the ESP32 reset like Serial Monitor
    except Exception as e:
        print(f"Failed to open serial port {port}: {e}")
        return

    if not wait_for_ready(ser):
        print("Never saw the CSV header - ESP32 setup didn't finish. Check wiring/reset.")
        ser.close()
        return

    if not start_inventory(ser):
        print("ESP32 never confirmed inventory start.")
        ser.close()
        return

    print(f"Logging to '{OUTPUT_PATH}'...\n")
    with open(OUTPUT_PATH, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp_ms", "moisture_raw", "moisture_voltage", "tag_epc", "rssi_dbm"])
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith("#"):
                    print(f"[ESP32 STATUS] {line}")
                    continue
                if line.startswith("timestamp_ms"):
                    continue  # header already written
                parts = line.split(',')
                print(f"[CSV DATA] {line}")
                writer.writerow(parts)
                file.flush()
        except KeyboardInterrupt:
            print("\nStopping scan...")
            ser.write(b's\n')
            ser.flush()
            time.sleep(0.5)
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()
