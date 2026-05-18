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
    # (3.0, 0.0, 0.0),   # Test 2: Medium P
    # (5.0, 0.0, 0.0),   # Test 3: High P (Might oscillate)
    # (3.0, 0.0, 0.1),   # Test 4: Medium P, with D damping
    # (3.0, 0.05, 0.1)   # Test 5: Full PID
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

# Dictionary to store the results for plotting later
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
    ser.write(f"TUNE PID {TEST_AXIS} {p} {i} {d}\n".encode())
    time.sleep(0.5)
    
    # 3. Start Telemetry
    ser.write(b"STREAM ON\n")
    
    # 4. Wait a fraction of a second to get baseline data, then trigger the Step
    time.sleep(0.5)
    ser.write(f"MOVE JOINT {TEST_AXIS} {STEP_TARGET}\n".encode())
    
    # 5. Record the data for 3 seconds
    print("Recording Step Response...")
    times, expected, actual = [], [], []
    
    start_time = time.time()
    while time.time() - start_time < 5.0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(line)
        
        # Skip the ACK lines and START_DATA tags
        if line and "ACK" not in line and "START" not in line and "END" not in line and "Teensy" not in line:
            try:
                parts = line.split(',')
                if len(parts) == 3:
                    times.append(float(parts[0]))
                    expected.append(float(parts[1]))
                    actual.append(float(parts[2]))
            except ValueError:
                pass

    # 6. Stop Telemetry
    ser.write(b"STREAM OFF\n")
    
    # Store the run in our dictionary
    results[pid_label] = (times, expected, actual)

ser.write(b"STOP\n")
ser.close()
print("\nTesting Complete. Generating Plot...")

# --- PLOTTING ---
plt.figure(figsize=(12, 8))

# Plot the Target (Expected) line once, using the times from the first run
first_key = list(results.keys())[0]
plt.plot(results[first_key][0], results[first_key][1], 'k--', linewidth=2, label='Target Angle (-45°)')

# Plot every PID response
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