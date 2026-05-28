import serial
import time
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
COM_PORT = '/dev/cu.usbmodem176147301'
BAUD_RATE = 115200

# The joint we are testing (2 = PIP)
TEST_AXIS = 2 
STEP_TARGET = -45.0 # The angle we want the joint to snap to

# List of PID tuples to test: (P, I, D)
PID_TEST_SETS = [
    (1.0, 0.0, 0.0),   # Test 1: Low P, No I, No D
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

print("Initializing Hand...")
ser.write(b"PID ON\n")
time.sleep(0.5)

# --- AUTOMATED TESTING LOOP ---
for (p, i, d) in PID_TEST_SETS:
    pid_label = f"P:{p} I:{i} D:{d}"
    print(f"\n--- Testing {pid_label} ---")
    
    # 1. Move to starting position (Zero) and let it settle
    ser.write(f"MOVE JOINT 0 0 0 0\n".encode())
    time.sleep(2.0) 
    
    # 2. Send the new tuning parameters
    ser.write(f"SET J_KP {p}\n".encode())
    time.sleep(0.1)
    ser.write(f"SET J_KI {i}\n".encode())
    time.sleep(0.1)
    ser.write(f"SET J_KD {d}\n".encode())
    time.sleep(0.5)
    
    # 3. Start Telemetry
    ser.write(b"STREAM ON\n")
    time.sleep(0.5)
    
    # 4. Trigger the OUT Step
    ser.write(f"MOVE JOINT {TEST_AXIS} {STEP_TARGET}\n".encode())
    
    # 5. Record data and manage the RETURN Step
    print("Recording Step Response (Out and Back)...")
    times, expected, actual = [], [], []
    
    start_time = time.time()
    return_triggered = False
    
    while True:
        elapsed = time.time() - start_time
        
        # Stop recording after 6 seconds
        if elapsed > 6.0:
            break
            
        # Trigger the RETURN Step at exactly 3 seconds
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

ser.write(b"PID OFF\n")
ser.close()
print("\nTesting Complete. Generating Plot...")

# --- PLOTTING ---
plt.figure(figsize=(12, 8))

first_key = list(results.keys())[0]
plt.plot(results[first_key][0], results[first_key][1], 'k--', linewidth=2, label='Target Angle')

for label, data in results.items():
    t_arr, _, act_arr = data
    plt.plot(t_arr, act_arr, linewidth=2, label=label)

plt.title('PID Step Response Comparison (PIP Joint)')
plt.xlabel('Time (Seconds)')
plt.ylabel('Angle (Degrees)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()