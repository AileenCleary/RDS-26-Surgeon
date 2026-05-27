import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301'
BAUD_RATE = 115200
# ---------------------

times = []
expected = []
actual_raw = [] 

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("Connected successfully!")
except Exception as e:
    print(f"Error connecting to serial port: {e}")
    exit()

time.sleep(1)
print("Sending test command to Teensy...")
ser.write(b"CALIBRATE ENCODER MCP\n") 
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
                expected.append(float(parts[1]))
                actual_raw.append(int(parts[2])) 
        except ValueError:
            pass 

ser.close()

if not expected:
    print("Error: No data recorded.")
    exit()

# --- MATH & CALIBRATION ---
exp_arr = np.array(expected)
raw_arr = np.array(actual_raw)

# 1. Unwrap the raw counts to remove the 65535 <-> 0 jump
# This creates a perfectly continuous line for the linear fit
unwrapped_raw = np.copy(raw_arr).astype(float)
for i in range(1, len(unwrapped_raw)):
    diff = unwrapped_raw[i] - unwrapped_raw[i-1]
    if diff > 32768:
        unwrapped_raw[i:] -= 65536
    elif diff < -32768:
        unwrapped_raw[i:] += 65536

# 2. Linear Fit (Find w_slope and w_zero on the continuous data)
m, b = np.polyfit(exp_arr, unwrapped_raw, 1)
w_slope_calc = m

# Wrap w_zero back to the physical 0-65535 range so it works in your C++ code
w_zero_calc = b % 65536

# 3. Simulate the exact diff16 wrap-around math your Teensy uses
def diff16(a, b_val):
    d = a - b_val
    d = np.where(d > 32768, d - 65536, d)
    d = np.where(d < -32768, d + 65536, d)
    return d

raw_diffs = diff16(raw_arr, w_zero_calc)
raw_angles = raw_diffs / w_slope_calc

# 4. Calculate the Error
error = exp_arr - raw_angles
old_rmse = np.sqrt(np.mean(error**2))

# 5. Fit a Sine Wave to the Error
def sine_comp(x, A, phase_deg, C):
    return A * np.sin((2.0 * x + phase_deg) * np.pi / 180.0) + C

# Smart initial guesses for a partial wave
guess_amp = (np.max(error) - np.min(error)) / 2.0
guess_offset = np.mean(error)

guess_amp = np.clip(guess_amp, 0, 179)
guess_offset = np.clip(guess_offset, -179, 179)
initial_guess = [guess_amp, 0.0, guess_offset]

param_bounds = ([0, -360, -180], [180, 360, 180])

popt, _ = curve_fit(sine_comp, raw_angles, error, p0=initial_guess, bounds=param_bounds)

comp_a_calc = popt[0]
comp_phase_calc = popt[1]
comp_offset_calc = popt[2]

# 6. Calculate the Final Compensated Angle
compensated_angles = raw_angles + sine_comp(raw_angles, comp_a_calc, comp_phase_calc, comp_offset_calc)

# Calculate the new error after compensation
new_error = exp_arr - compensated_angles
new_rmse = np.sqrt(np.mean(new_error**2))

# --- TERMINAL OUTPUT ---
print("\n=======================================================")
print("  CALIBRATION SUCCESSFUL - COPY THESE INTO MA782.ino  ")
print("=======================================================")
print(f"w_slope     = {w_slope_calc:.2f}")
print(f"w_zero      = {w_zero_calc:.1f}")
print(f"COMP_A      = {comp_a_calc:.3f}")
print(f"COMP_PHASE  = {comp_phase_calc:.2f}")
print(f"COMP_OFFSET = {comp_offset_calc:.3f}")
print("-------------------------------------------------------")
print(f"Original RMSE   : {old_rmse:.3f} degrees")
print(f"Compensated RMSE: {new_rmse:.3f} degrees")
print("=======================================================\n")

# --- PLOTTING ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Graph 1: Expected vs Uncompensated vs Compensated
ax1.plot(exp_arr, exp_arr, 'b-', label='Perfect Motion (Expected)', linewidth=2)
ax1.plot(exp_arr, raw_angles, 'r--', label=f'Uncompensated (RMSE: {old_rmse:.2f}°)', alpha=0.7)
ax1.plot(exp_arr, compensated_angles, 'g-', label=f'Compensated (RMSE: {new_rmse:.2f}°)', linewidth=2)
ax1.set_ylabel('Angle (Degrees)')
ax1.set_title('Sensor Accuracy Improvement')
ax1.grid(True)
ax1.legend()

# Generate a perfectly smooth x-axis for drawing the sine fit line (fixes the "scribble")
x_smooth = np.linspace(np.min(raw_angles), np.max(raw_angles), 500)
fitted_sine_smooth = sine_comp(x_smooth, comp_a_calc, comp_phase_calc, comp_offset_calc)

# Graph 2: The Error & The Sine Fit
ax2.plot(raw_angles, error, 'r.', label='Actual Sensor Error', alpha=0.6)
ax2.plot(x_smooth, fitted_sine_smooth, 'k-', label=f'Sine Fit (A={comp_a_calc:.2f}, Ph={comp_phase_calc:.1f}°)', linewidth=2)
ax2.axhline(0, color='gray', linestyle='--')
ax2.set_xlabel('Sensor Angle (Degrees)')
ax2.set_ylabel('Error (Degrees)')
ax2.set_title('Error Curve Extraction')
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()