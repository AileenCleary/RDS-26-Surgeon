import numpy as np
import matplotlib.pyplot as plt

LETTERS = {
    'A': [(0, 0), (0.5, 1), (1, 0), (0.75, 0.5), (0.25, 0.5)],
    'B': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5), (0.75, 0.5), (1, 0.25), (0.75, 0), (0, 0)],
    'C': [(1, 1), (0.25, 1), (0, 0.75), (0, 0.25), (0.25, 0), (1, 0)],
    'D': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (1, 0.25), (0.75, 0), (0, 0)],
    'E': [(1, 1), (0, 1), (0, 0.5), (0.75, 0.5), (0, 0.5), (0, 0), (1, 0)],
    'F': [(0, 0), (0, 1), (1, 1), (0, 1), (0, 0.5), (0.75, 0.5)],
    'G': [(1, 1), (0.25, 1), (0, 0.75), (0, 0.25), (0.25, 0), (1, 0), (1, 0.5), (0.5, 0.5)],
    'H': [(0, 1), (0, 0), (0, 0.5), (1, 0.5), (1, 1), (1, 0)],
    'I': [(0.25, 1), (0.75, 1), (0.5, 1), (0.5, 0), (0.25, 0), (0.75, 0)],
    'J': [(0, 0.5), (0.25, 0), (0.75, 0), (1, 0.25), (1, 1)],
    'K': [(0, 1), (0, 0), (0, 0.5), (1, 1), (0, 0.5), (1, 0)],
    'L': [(0, 1), (0, 0), (1, 0)],
    'M': [(0, 0), (0, 1), (0.5, 0.5), (1, 1), (1, 0)],
    'N': [(0, 0), (0, 1), (1, 0), (1, 1)],
    'O': [(0.5, 1), (0.2, 0.8), (0, 0.5), (0.2, 0.2), (0.5, 0), (0.8, 0.2), (1, 0.5), (0.8, 0.8), (0.5, 1)],
    'P': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5)],
    'Q': [(0.8, 0.2), (1, 0.5), (0.8, 0.8), (0.5, 1), (0.2, 0.8), (0, 0.5), (0.2, 0.2), (0.5, 0), (0.8, 0.2), (0.5, 0.5), (1, 0)],
    'R': [(0, 0), (0, 1), (0.75, 1), (1, 0.75), (0.75, 0.5), (0, 0.5), (0.5, 0.5), (1, 0)],
    'S': [(1, 1), (0.25, 1), (0, 0.75), (0.25, 0.5), (0.75, 0.5), (1, 0.25), (0.75, 0), (0, 0)],
    'T': [(0, 1), (1, 1), (0.5, 1), (0.5, 0)],
    'U': [(0, 1), (0, 0.25), (0.25, 0), (0.75, 0), (1, 0.25), (1, 1)],
    'V': [(0, 1), (0.5, 0), (1, 1)],
    'W': [(0, 1), (0.25, 0), (0.5, 0.5), (0.75, 0), (1, 1)],
    'X': [(0, 1), (1, 0), (0.5, 0.5), (0, 0), (1, 1)],
    'Y': [(0, 1), (0.5, 0.5), (1, 1), (0.5, 0.5), (0.5, 0)],
    'Z': [(0, 1), (1, 1), (0, 0), (1, 0)]
}

def generate_trajectory(waypoints, points_per_segment=50):
    """
    Converts sparse waypoints into a dense, time-stamped trajectory.
    Returns times, x_coords, and y_coords.
    """
    t_total = []
    x_total = []
    y_total = []
    
    current_time = 0.0
    
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i+1]
        
        # Calculate distance to approximate time taken for this segment (assuming constant speed)
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        segment_time = distance  # 1 unit of distance = 1 unit of time
        
        # Interpolate points
        t = np.linspace(current_time, current_time + segment_time, points_per_segment)
        x = np.linspace(x1, x2, points_per_segment)
        y = np.linspace(y1, y2, points_per_segment)
        
        t_total.extend(t)
        x_total.extend(x)
        y_total.extend(y)
        
        current_time += segment_time
        
    return np.array(t_total), np.array(x_total), np.array(y_total)

def plot_trajectory(t, x, y, letter):
    """
    Plots the spatial trajectory (X vs Y) and the time-series targets (X vs Time, Y vs Time).
    """
    fig = plt.figure(figsize=(12, 5))
    fig.canvas.manager.set_window_title(f"Robot Trajectory for '{letter}'")

    # Plot 1: The resulting letter (X, Y space)
    ax1 = plt.subplot(1, 2, 1)
    ax1.plot(x, y) # The continuous path line
    ax1.set_title(f"Spatial Trajectory: '{letter}'")
    ax1.set_xlabel("X Coordinate")
    ax1.set_ylabel("Y Coordinate")
    ax1.set_aspect('equal')

    # Plot 2: X and Y vs Time
    ax2 = plt.subplot(1, 2, 2)
    ax2.plot(t, x, label='X Target', color='red')
    ax2.plot(t, y, label='Y Target', color='blue')
    ax2.set_title("Target Positions Over Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Coordinate Value")
    ax2.legend()

    plt.tight_layout()
    plt.show()

def main():
    print("=== Robot Letter Trajectory Generator ===")
    user_input = input("Enter any letter from A to Z: ").strip()
    
    if len(user_input) != 1 or not user_input.isalpha():
        print("Please enter a single valid letter.")
        return

    waypoints = LETTERS[user_input.upper()]
    
    if waypoints:
        print(f"Generating continuous trajectory for '{user_input.upper()}'...")
        t, x, y = generate_trajectory(waypoints, points_per_segment=50)
        
        print("\nFirst 5 generated trajectory points (t, x, y):")
        for i in range(5):
            print(f"Time: {t[i]:.2f}s | X: {x[i]:.2f} | Y: {y[i]:.2f}")
            
        plot_trajectory(t, x, y, user_input.upper())

if __name__ == "__main__":
    main()