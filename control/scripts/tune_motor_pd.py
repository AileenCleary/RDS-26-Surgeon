import serial
import time
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301'
BAUD_RATE = 115200

TEST_AXIS = 1 
STEP_TARGET = -30.0

# List of PD values to test: 
# (Kp_strong, Kd_strong, Kp_soft, Kd_soft)
PD_TEST_SETS = [
    (0.02, 0.0005, 0.005, 0.0002),
]
# ---------------------

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("Connected successfully!")
except Exception as e:
    print(f"Error connecting: {e}")
    exit()

time.sleep(2) # Wait for Arduino to reset

results = {}
print("Initializing Hand...")
time.sleep(0.5)

# --- AUTOMATED TESTING LOOP ---
for (kp_s, kd_s, kp_w, kd_w) in PD_TEST_SETS:
    # Create a clean label for the graph legend
    pid_label = f"Kp_s:{kp_s} Kd_s:{kd_s} Kp_w:{kp_w} Kd_w:{kd_w}"
    print(f"\n--- Testing {pid_label} ---")
    
    # 1. Move to starting position (Zero)
    ser.write(f"MOVE JOINT 0 0 0 0\n".encode())
    time.sleep(2.0) 
    
    # 2. Send the new tuning parameters
    ser.write(f"SET M_KP_S {kp_s}\n".encode())
    time.sleep(0.1)
    ser.write(f"SET M_KD_S {kd_s}\n".encode())
    time.sleep(0.1)
    ser.write(f"SET M_KP_W {kp_w}\n".encode())
    time.sleep(0.1)
    ser.write(f"SET M_KD_W {kd_w}\n".encode())
    time.sleep(0.5)
    
    # 3. Start Telemetry
    ser.write(b"STREAM ON\n")
    time.sleep(0.5)
    
    # 4. Trigger the OUT Step (Testing Motor, not Joint)
    ser.write(f"MOVE JOINT {TEST_AXIS} {STEP_TARGET}\n".encode())
    
    # 5. Record data and manage the RETURN Step
    print("Recording Step Response (Out and Back)...")
    times, expected, actual = [], [], []
    
    start_time = time.time()
    return_triggered = False
    
    while True:
        elapsed = time.time() - start_time
        
        if elapsed > 6.0:
            break
            
        if elapsed > 3.0 and not return_triggered:
            ser.write(f"MOVE JOINT {TEST_AXIS} 0\n".encode())
            return_triggered = True
            print("Returning to 0...")

        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if line and "ACK" not in line and "START" not in line and "END" not in line and "Teensy" not in line:
            try:
                parts = line.split(',')
                if len(parts) == 9:
                    times.append(float(parts[0]))
                    expected.append(float(parts[1 + TEST_AXIS]))
                    actual.append(float(parts[5 + TEST_AXIS]))
            except ValueError:
                pass

    # 6. Stop Telemetry
    ser.write(b"STREAM OFF\n")
    results[pid_label] = (times, expected, actual)

ser.write(b"STOP\n")
ser.close()
print("\nTesting Complete. Generating Plot...")

# --- PLOTTING ---
plt.figure(figsize=(12, 8))

first_key = list(results.keys())[0]
plt.plot(results[first_key][0], results[first_key][1], 'k--', linewidth=2, label='Target Angle')

for label, data in results.items():
    t_arr, _, act_arr = data
    plt.plot(t_arr, act_arr, linewidth=2, label=label)

plt.title('Motor PD Step Response Comparison')
plt.xlabel('Time (Seconds)')
plt.ylabel('Position (Degrees)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()