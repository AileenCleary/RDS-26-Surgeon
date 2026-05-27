import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301'
BAUD_RATE = 115200
# ---------------------

times = []
expected_x = []
expected_y = []
actual_x = [] 
actual_y = []

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("Connected successfully!")
except Exception as e:
    print(f"Error connecting to serial port: {e}")
    exit()

time.sleep(1)
print("Sending test command to Teensy...")
ser.write(b"TEST TRAJ\n") 
print("Waiting for START_DATA signal...")

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line == "START_DATA":
        print("Data stream started. Recording...")
        break
    elif line:
        print(f"Teensy: {line}")

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line == "END_DATA":
        print("Data stream finished. Calculating parameters...")
        break
    
    if line:
        try:
            parts = line.split(',')
            if len(parts) == 3:
                times.append(float(parts[0]))
                expected_x.append(float(parts[1]))
                expected_y.append(float(parts[2]))
                actual_x.append(int(parts[3])) 
                actual_y.append(int(parts[4]))
        except ValueError:
            pass 

ser.close()

if not expected_x:
    print("Error: No data recorded.")
    exit()

# --- MATH & CALIBRATION ---
exp_x_arr = np.array(expected_x)
actual_x_arr = np.array(actual_x)
exp_y_arr = np.array(expected_y)
actual_y_arr = np.array(actual_y)

# 1. Calculate Error
error_x = exp_x_arr - actual_x_arr
error_y = exp_y_arr - actual_y_arr
dist_sq = error_x**2 + error_y**2

# 2. RMS Tracking Error
rms_error = np.sqrt(dist_sq.mean())
print(f"RMS Tracking Error: {rms_error:.3f} mm")

# 3. Visualization
plt.figure(figsize=(8,8))
plt.plot(expected_x, expected_y, label='Reference', linestyle='--')
plt.plot(actual_x, actual_y, label='Actual Path')
plt.title(f"Trajectory Tracking (RMS Error: {rms_error:.2f}mm)")
plt.legend()
plt.grid(True)
plt.show()