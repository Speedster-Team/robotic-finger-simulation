import serial
import time

# --- CONFIG ---
PORT = "/dev/ttyACM0"   # change to your Teensy port (e.g. /dev/ttyACM0 or COM3)
BAUD = 115200
DELAY = 0.0             # seconds between commands

# --- COMMANDS ---
# Each entry is a list of motor positions (one per motor)
# Format is whatever your Teensy expects — adjust as needed
commands = [
    [0, 0, 0],
    [10, 20, 30],
    [45, 90, 0],
    [0, 0, 0],
]

def format_command(positions):
    """Convert a list of positions to a string the Teensy can parse."""
    return ",".join(str(p) for p in positions) + "\n"

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)  # wait for Teensy to reset after serial connect
        print(f"Connected to {PORT}")

        for positions in commands:
            msg = format_command(positions)
            ser.write(msg.encode())
            print(f"Sent: {msg.strip()}")

            # optional: read back a response from Teensy
            response = ser.readline().decode().strip()
            if response:
                print(f"  <- {response}")

            time.sleep(DELAY)

    print("Done.")

if __name__ == "__main__":
    main()