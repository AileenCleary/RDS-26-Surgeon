import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/cu.usbmodem176147301'
BAUD_RATE = 115200

def trigger_and_record(test_command):
    log = {
        't': [], 'x_act': [], 'y_act': [], 'x_des': [], 'y_des': [],
        'f_act': [], 'f_des': [], 'q_act': [], 'q_des': []
    }
    
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        time.sleep(2) # Wait for connection
        ser.write(('PID OFF\n').encode())
        time.sleep(0.5)
        
        print(f"Sending command: {test_command}")
        ser.write((test_command + '\n').encode())
        
        recording = False
        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
                
            if "START_DRAW_DATA" in line:
                print("Recording telemetry...")
                recording = True
                continue
            
            if "END_DRAW_DATA" in line:
                print("Test Complete.")
                break
                
            if recording and line.startswith("DRAW_DATA"):
                parts = line.replace("DRAW_DATA", "").split(',')
                try:
                    vals = [float(x) for x in parts]
                    log['t'].append(vals[0])
                    log['x_des'].append(vals[1])
                    log['y_des'].append(vals[2])
                    log['f_des'].append(vals[3])
                    log['x_act'].append(vals[4])
                    log['y_act'].append(vals[5])
                    log['f_act'].append(vals[6])
                    log['q_des'].append(vals[7:11])
                    log['q_act'].append(vals[11:15])
                except ValueError:
                    pass # Corrupted line, skip

    # Convert lists to numpy arrays for plotting
    for k in log:
        log[k] = np.array(log[k])
    return log

def plot_results(log, title):
    t = log['t']
    x_act = log['x_act']
    y_act = log['y_act']
    x_des = log['x_des']
    y_des = log['y_des']
    f_act = log['f_act']
    f_des = log['f_des']
    q_act = log['q_act']
    q_des = log['q_des']

    fig = plt.figure(figsize=(14, 10))
    fig.canvas.manager.set_window_title(title)
    
    # 1. 2D Path Tracking
    ax1 = plt.subplot(2, 2, 1)
    # sc = ax1.scatter(x_act, y_act, c=f_act, cmap='Greys', s=20, edgecolor='none', vmin=0, vmax=np.max(f_act)+1.0)
    # plt.colorbar(sc, ax=ax1, label='Normal Force (N)')
    sc = ax1.scatter(-y_act, -x_act)
    ax1.plot(-y_des, -x_des, color='blue', linestyle='--', alpha=0.3, label='Desired Path')
    ax1.set_title("Physical Drawing Canvas (Planar X vs Splay Y)")
    ax1.set_xlabel("X (Forward/Back) mm")
    ax1.set_ylabel("Y (Splay Left/Right) mm")
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    # 2. Force Tracking
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(t, f_des, color='black', linestyle='--', label='Expected Force (Depth Map)', linewidth=2)
    ax2.plot(t, f_act, color='crimson', label='Actual Force (Kinematic Estimate)', linewidth=2)
    ax2.set_title("Pencil Tip Normal Force Tracking")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (N)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Joint Tracking
    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(t, q_des[:, 0], color='C0', linestyle='--', label='Desired Splay')
    ax3.plot(t, q_des[:, 1], color='C1', linestyle='--', label='Desired MCP')
    ax3.plot(t, q_des[:, 2], color='C2', linestyle='--', label='Desired PIP')
    ax3.plot(t, q_act[:, 0], color='C0', label='Actual Splay')
    ax3.plot(t, q_act[:, 1], color='C1', label='Actual MCP')
    ax3.plot(t, q_act[:, 2], color='C2', label='Actual PIP')
    ax3.set_title("Physical Joint Kinematics")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Angle (deg)")
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. Trajectory Tracking Error
    ax4 = plt.subplot(2, 2, 4)
    error = np.sqrt((x_act - x_des)**2 + (y_act - y_des)**2)
    ax4.plot(t, error, color='purple', label='Planar Path Error')
    ax4.set_title("2D Path Tracking Error")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Euclidean Error (mm)")
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    letter = "L"
    choice = input(f"Select Test [1: Write '{letter}', 2: Shade Square]: ").strip()
    
    if choice == '1':
        telemetry = trigger_and_record(f"TEST WRITE {letter}")
        plot_results(telemetry, "Physical Test: Writing Letter '{letter}'")
    elif choice == '2':
        telemetry = trigger_and_record("TEST SHADE")
        plot_results(telemetry, "Physical Test: Variable Force Shading")
    else:
        print("Invalid Selection.")