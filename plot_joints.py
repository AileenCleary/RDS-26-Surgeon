import serial
import time
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301' # Change this to your Teensy's port!
BAUD_RATE = 115200
# ---------------------

times = []
expected = []
actual = []

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("Connected successfully!")
except Exception as e:
    print(f"Error connecting to serial port: {e}")
    print("TIP: Make sure the Arduino Serial Monitor and ODrive WebGUI are fully closed!")
    exit()

# Wait a brief moment for the serial connection to stabilize
time.sleep(1)

# Tell Python to trigger the test on the Teensy
print("Sending test command to Teensy...")
ser.write(b"TEST LINEARITY PIP\n")

print("Waiting for START_DATA signal...")

# Wait for the test to begin
while True:
    line = ser.readline().decode('utf-8').strip()
    if line == "START_DATA":
        print("Data stream started. Recording...")
        break
    elif line:
        # Print any normal serial output from the Teensy while we wait
        print(f"Teensy: {line}")

# Record the data
while True:
    line = ser.readline().decode('utf-8').strip()
    print(line)
    
    if line == "END_DATA":
        print("Data stream finished. Plotting graphs...")
        break
    
    # Parse the CSV line (Time, Expected, Actual)
    if line:
        try:
            parts = line.split(',')
            if len(parts) == 3:
                times.append(float(parts[0]))
                expected.append(float(parts[1]))
                actual.append(float(parts[2]))
        except ValueError:
            pass # Ignore corrupted serial lines

ser.close()

# Plot the data on two separate, static graphs
fig, (ax1) = plt.subplots(1, 1, figsize=(10, 8), sharex=True)

# Graph 1: Expected Mechanical Angle
ax1.plot(times, expected, 'b-', label='Expected Angle (deg)')
ax1.plot(times, actual, 'r-', label='Actual Angle (deg)')
ax1.set_ylabel('Angle (Degrees)')
ax1.set_title('Expected Mechanical Motion (Linear Sweep)')
ax1.grid(True)
ax1.legend()

plt.tight_layout()
plt.show()