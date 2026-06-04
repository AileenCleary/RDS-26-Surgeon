import serial
import time
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301'
BAUD_RATE = 115200

# The force we want the finger to exert (in Newtons)
STEP_TARGET_FORCE = 5.0 

# List of Force PID tuples to test: (P, I, D)
PID_TEST_SETS = [
    (10.0, 0.0, 0.1),
]
# ---------------------

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("Connected successfully!")
except Exception as e:
    print(f"Error connecting: {e}")
    exit()

time.sleep(2) # Wait for Arduino to reset on connection

results = {}

print("WARNING: Ensure the finger is resting against a hard surface!")
print("Force control in free air will cause the finger to snap to its mechanical limits.")
time.sleep(3)

ser.write(f"PID OFF\n".encode())
time.sleep(0.5)

# --- AUTOMATED TESTING LOOP ---
for (p, i, d) in PID_TEST_SETS:
    pid_label = f"P:{p} I:{i} D:{d}"
    print(f"\n--- Testing {pid_label} ---")
    
    # 1. Initialize Force to 0 while resting on the surface
    ser.write(f"MOVE FORCE 0\n".encode())
    time.sleep(1.0) 
    
    # 2. Send the new Force PID parameters
    ser.write(f"SET F_KP {p}\n".encode())
    time.sleep(0.1)
    ser.write(f"SET F_KI {i}\n".encode())
    time.sleep(0.1)
    ser.write(f"SET F_KD {d}\n".encode())
    time.sleep(0.5)
    
    # 3. Start Force Telemetry (Time, TargetForce, ActualForce)
    ser.write(b"STREAM FORCE ON\n")
    time.sleep(0.5)
    
    # 4. Trigger the Step Response
    ser.write(f"MOVE FORCE {STEP_TARGET_FORCE}\n".encode())
    
    # 5. Record data
    print(f"Recording Step Response to {STEP_TARGET_FORCE} N...")
    times, expected, actual = [], [], []
    
    start_time = time.time()
    return_triggered = False
    
    while True:
        elapsed = time.time() - start_time
        
        # Stop recording after 6 seconds
        if elapsed > 6.0:
            break
            
        # Trigger the RETURN Step (Release force) at exactly 3 seconds
        if elapsed > 3.0 and not return_triggered:
            ser.write(f"MOVE FORCE 0\n".encode())
            return_triggered = True
            print("Releasing force to 0 N...")

        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if line and "ACK" not in line and "START" not in line and "END" not in line:
            try:
                parts = line.split(',')
                # Expecting precisely: Time, Target, Actual
                if len(parts) == 3:
                    times.append(float(parts[0]))
                    expected.append(float(parts[1]))
                    actual.append(float(parts[2]))
            except ValueError:
                pass

    # 6. Stop Telemetry
    ser.write(b"STREAM FORCE OFF\n")
    results[pid_label] = (times, expected, actual)

# Final safety release
ser.write(f"MOVE FORCE 0\n".encode())
time.sleep(1.0)
ser.write(f"MOVE MOTOR 0 0 0 0 0\n".encode())
time.sleep(0.5)
ser.close()

print("\nTesting Complete. Generating Plot...")

# --- PLOTTING ---
plt.figure(figsize=(12, 8))

# Plot the Target Force path (Step up, step down)
first_key = list(results.keys())[0]
plt.plot(results[first_key][0], results[first_key][1], 'k--', linewidth=2, label='Target Force (N)')

# Plot all actual force responses
for label, data in results.items():
    t_arr, _, act_arr = data
    plt.plot(t_arr, act_arr, linewidth=2, label=label)

plt.title('Force PID Step Response Comparison')
plt.xlabel('Time (Seconds)')
plt.ylabel('Force (Newtons)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()