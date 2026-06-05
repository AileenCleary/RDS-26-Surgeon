import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301'  # Update with your port
BAUD_RATE = 115200

# Sweep Parameters (Must match Arduino)
f0 = 0.1
f1 = 100.0
T = 60.0
k = np.log(f1 / f0) / T
# ---------------------

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Connected successfully!")
except Exception as e:
    print(f"Error connecting: {e}")
    exit()

# Send command to trigger the chirp
print("Triggering 60-second Sine Chirp Sweep...")
ser.write(b"TEST SINE POS\n")

times, expected_z, actual_z = [], [], []
recording = False

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line == "START_DATA":
        print("Data stream started. Recording 60 seconds of high-speed sweep...")
        recording = True
        continue
    elif line == "END_DATA":
        print("Data stream finished. Processing Bode Plot...")
        break
        
    if recording and line:
        try:
            parts = line.split(',')
            if len(parts) == 3:
                times.append(float(parts[0]))
                expected_z.append(float(parts[1]))
                actual_z.append(float(parts[2]))
        except ValueError:
            pass 

ser.close()

if not times:
    print("Error: No data recorded.")
    exit()

t_arr = np.array(times)
exp_z = np.array(expected_z)
act_z = np.array(actual_z)

# --- CHIRP SIGNAL ANALYSIS ---
# We sample the continuous chirp at 40 distinct frequency targets to plot
target_freqs = np.logspace(np.log10(f0), np.log10(f1), 40)
amplitudes_db = []
phases_deg = []
valid_freqs = []

# The exact mathematical phase equation of the log chirp
def chirp_model(t, A, phi, offset):
    phase = 2 * np.pi * f0 * (np.exp(k * t) - 1.0) / k
    return offset + A * np.sin(phase + phi)

for f_target in target_freqs:
    # 1. Find the time at which the sweep passes this frequency
    t_center = np.log(f_target / f0) / k
    
    # 2. Grab a window of data (At least 1 period, max 10 seconds to avoid edge crossing)
    window_t = min(max(1.5 / f_target, 0.5), 10.0)
    t_start = max(t_center - window_t / 2.0, 0)
    t_end = min(t_center + window_t / 2.0, T)
    
    mask = (t_arr >= t_start) & (t_arr <= t_end)
    t_slice = t_arr[mask]
    exp_slice = exp_z[mask]
    act_slice = act_z[mask]
    
    if len(t_slice) < 20: continue
    
    try:
        # Fit Expected (Reference) to get exact input amplitude/phase
        popt_exp, _ = curve_fit(chirp_model, t_slice, exp_slice, p0=[10.0, 0.0, np.mean(exp_slice)])
        A_in, phi_in, _ = popt_exp
        
        # Fit Actual to get output amplitude/phase
        popt_act, _ = curve_fit(chirp_model, t_slice, act_slice, p0=[popt_exp[0], popt_exp[1], np.mean(act_slice)])
        A_out, phi_out, _ = popt_act
        
        # Enforce positive amplitudes for magnitude ratio
        if A_in < 0: A_in, phi_in = -A_in, phi_in + np.pi
        if A_out < 0: A_out, phi_out = -A_out, phi_out + np.pi
            
        mag_db = 20 * np.log10(A_out / A_in)
        phase_shift = np.degrees(phi_out - phi_in)
        phase_shift = (phase_shift + 180) % 360 - 180
        
        amplitudes_db.append(mag_db)
        phases_deg.append(phase_shift)
        valid_freqs.append(f_target)
        
    except RuntimeError:
        pass # Skip if curve_fit fails (usually happens at extreme high freq attenuation)

# --- CALCULATE BANDWIDTH ---
bandwidth = None
for i, mag in enumerate(amplitudes_db):
    if mag <= -3.0:
        bandwidth = valid_freqs[i]
        break

# --- PLOT BODE PLOT ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Magnitude
ax1.semilogx(valid_freqs, amplitudes_db, marker='o', linestyle='-', color='b')
ax1.axhline(y=-3, color='r', linestyle='--', label='-3 dB (Bandwidth Threshold)')
if bandwidth:
    ax1.axvline(x=bandwidth, color='k', linestyle=':', label=f'Bandwidth: {bandwidth:.1f} Hz')
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title(f'Bode Plot: Position Tracking Frequency Response')
ax1.grid(True, which="both", ls="-", alpha=0.5)
ax1.legend()

# Phase
ax2.semilogx(valid_freqs, phases_deg, marker='o', linestyle='-', color='g')
ax2.set_ylabel('Phase (Degrees)')
ax2.set_xlabel('Frequency (Hz)')
ax2.grid(True, which="both", ls="-", alpha=0.5)

plt.tight_layout()
plt.show()

if bandwidth:
    print(f"\n✅ System Bandwidth (-3dB dropoff) is approximately {bandwidth:.2f} Hz.")
else:
    print("\n⚠️ System did not drop below -3dB in the tested range. Bandwidth > 100 Hz.")