import socket
import struct
import csv
import time

RTT_PORT = 60001
OUTPUT_FILE = 'variables.csv'
MAGIC = 0xAA

print("RTT Data Capture Script")
print("=" * 50)
print(f"Connecting to RTT server on port {RTT_PORT}...")
print(f"Saving to: {OUTPUT_FILE}")
print("Press Ctrl+C to stop\n")

line_count = 0

try:
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('localhost', RTT_PORT))
    print("Connected! Flushing stale data...")

    client.setblocking(False)
    flush_end = time.time() + 2.0
    while time.time() < flush_end:
        try:
            client.recv(4096)
        except BlockingIOError:
            time.sleep(0.01)
    client.setblocking(True)

    print("Capturing data...\n")

    with open(OUTPUT_FILE, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        header_written = False
        buffer = b""

        while True:
            data = client.recv(1024)
            if not data:
                break

            buffer += data

            while len(buffer) >= 2:
                # Find magic byte
                if buffer[0] != MAGIC:
                    # Skip until we find magic
                    idx = buffer.find(bytes([MAGIC]), 1)
                    if idx == -1:
                        buffer = b""
                        break
                    buffer = buffer[idx:]
                    continue

                count = buffer[1]

                if count == 0 or count > 15:
                    buffer = buffer[1:]
                    continue

                packet_size = 2 + 4 + count * 4

                if len(buffer) < packet_size:
                    break

                packet = buffer[2:packet_size]
                buffer = buffer[packet_size:]

                timestamp = struct.unpack('<I', packet[0:4])[0]
                floats = struct.unpack(f'<{count}f', packet[4:])

                # Sanity check: timestamp should be reasonable
                if timestamp > 100000000:
                    continue

                if not header_written:
                    header = ['timestamp_ms'] + [f'val_{i}' for i in range(count)]
                    writer.writerow(header)
                    header_written = True

                row = [timestamp] + list(floats)
                writer.writerow(row)
                csvfile.flush()

                float_str = ", ".join(f"{v:.6f}" for v in floats)
                print(f"{timestamp}, {float_str}")

                line_count += 1
                if line_count % 100 == 0:
                    print(f"\n[Captured {line_count} lines]\n")

except KeyboardInterrupt:
    print(f"\n\nCapture stopped by user.")
except Exception as e:
    print(f"\nError: {e}")
finally:
    client.close()
    print(f"\nTotal lines captured: {line_count}")
    print(f"Data saved to: {OUTPUT_FILE}")