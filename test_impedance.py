import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301'
BAUD_RATE = 115200
# ---------------------

time_data = []
force_data = []
displacement_data = []

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("Connected successfully!")
except Exception as e:
    print(f"Error connecting to serial port: {e}")
    exit()

time.sleep(1)
print("Sending test command to Teensy...")
ser.write(b"TEST IMPEDANCE\n")  # Make sure Arduino listens for this
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
                time_data.append(float(parts[0]))
                force_data.append(float(parts[1]))
                displacement_data.append(float(parts[2]))
        except ValueError:
            pass 

ser.close()

if not time_data:
    print("Error: No data recorded.")
    exit()

time_data = np.array(time_data)
force_data = np.array(force_data)
displacement_data = np.array(displacement_data)

# Find frequency transitions (where time resets or force amplitude changes)
frequencies = [0.5, 1.0, 2.0, 5.0]
results = {}

for freq_idx, f in enumerate(frequencies):
    # Approximate time range for this frequency (20 seconds per frequency)
    start_idx = freq_idx * 1000  # Approximate, adjust based on actual data
    end_idx = (freq_idx + 1) * 1000
    
    # Find actual data boundaries by looking for where force changes pattern
    # For simplicity, we'll use approximate indices - you may need to refine this
    
    if end_idx > len(time_data):
        end_idx = len(time_data)
    
    t = time_data[start_idx:end_idx]
    F = force_data[start_idx:end_idx]
    x = displacement_data[start_idx:end_idx]
    
    if len(t) < 10:
        continue
    
    # Normalize time to start at 0 for this frequency segment
    t_norm = t - t[0]
    
    # Define the MSD model: steady-state response to sinusoidal forcing
    def msd_model(t, m, b, k, phase):
        amp = 1.0  # Force amplitude
        omega = 2 * np.pi * f
        Z = np.sqrt((k - m * omega**2)**2 + (b * omega)**2)
        return (amp / Z) * np.sin(omega * t - phase)
    
    try:
        # Fit the model to data (skip first 2 seconds for transient)
        transient_skip = int(2.0 / 0.02)  # Approximate based on 20ms updates
        t_fit = t_norm[transient_skip:]
        x_fit = x[transient_skip:]
        
        # Initial guesses: m, b, k, phase
        popt, _ = curve_fit(msd_model, t_fit, x_fit, 
                           p0=[0.01, 0.1, 10.0, 0.0],
                           maxfev=5000)
        
        m, b, k, phase = popt
        
        # Calculate impedance magnitude and phase
        omega = 2 * np.pi * f
        Z = np.sqrt((k - m * omega**2)**2 + (b * omega)**2)
        phase_lag = np.arctan2(b * omega, k - m * omega**2) * 180 / np.pi
        
        results[f] = {
            'mass': m,
            'damping': b,
            'stiffness': k,
            'impedance': Z,
            'phase_lag': phase_lag
        }
        
        print(f"\nFrequency {f} Hz:")
        print(f"  Mass: {m:.4f} kg")
        print(f"  Damping: {b:.4f} N·s/m")
        print(f"  Stiffness: {k:.4f} N/m")
        print(f"  Impedance: {Z:.4f} N·s/m")
        print(f"  Phase Lag: {phase_lag:.2f}°")
        
    except Exception as e:
        print(f"Failed to fit frequency {f} Hz: {e}")

# Plot results
fig, axes = plt.subplots(2, 2, figsize=(12, 10))
axes = axes.flatten()

for idx, f in enumerate(frequencies):
    if f in results:
        ax = axes[idx]
        start_idx = idx * 1000
        end_idx = (idx + 1) * 1000
        if end_idx > len(time_data):
            end_idx = len(time_data)
        
        ax.plot(time_data[start_idx:end_idx], displacement_data[start_idx:end_idx], 'b-', label='Measured')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Displacement (m)')
        ax.set_title(f'Frequency: {f} Hz\nZ = {results[f]["impedance"]:.3f} N·s/m')
        ax.grid(True)
        ax.legend()

plt.tight_layout()
plt.savefig('impedance_results.png')
plt.show()

print("\nImpedance characterization complete!")