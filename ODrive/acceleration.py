import odrive
from odrive.enums import *
import time
import math
import matplotlib.pyplot as plt

# Connet to ODrive
print("Connecting...")
odrv0 = odrive.find_any()
odrv = odrv0
odrv.config.dc_bus_overvoltage_trip_level = 28
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = 10
odrv.config.dc_max_negative_current = -10
odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
odrv.axis0.config.motor.pole_pairs = 1
odrv.axis0.config.motor.torque_constant = 0.01978468899521531
odrv.axis0.config.motor.current_soft_max = 10
odrv.axis0.config.motor.current_hard_max = 23
odrv.axis0.config.motor.calibration_current = 2
odrv.axis0.config.motor.resistance_calib_max_voltage = 5
odrv.axis0.config.calibration_lockin.current = 2
odrv.axis0.motor.motor_thermistor.config.enabled = False
odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv.axis0.controller.config.vel_limit = 10
odrv.axis0.controller.config.vel_limit_tolerance = 1.2
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.can.config.protocol = Protocol.NONE
odrv.axis0.config.enable_watchdog = False
odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
odrv.config.enable_uart_a = False

# Reset
print("Reseting...")
odrv0.clear_errors()
odrv0.axis0.requested_state = AxisState.IDLE
while odrv0.axis0.current_state != AxisState.IDLE:
    time.sleep(0.1)

print("Reading positions...")
dt = 0.01
start = time.time()
times = []
positions = []
velocities = []
accelerations = []
prev_pos = odrv0.axis0.pos_estimate*360
prev_vel = 0

try:
    while True:
        t = time.time() - start
        current_pos = odrv0.axis0.pos_estimate*360
        current_vel = (current_pos - prev_pos) / dt
        current_acc = (current_vel - prev_vel) / dt

        times.append(t)
        positions.append(current_pos)
        accelerations.append(current_acc)
        prev_pos = current_pos
        prev_vel = current_vel
        time.sleep(dt)

except KeyboardInterrupt:
    pass

plt.plot(times, accelerations)
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (deg/s^2)")
plt.title("Backdrivability Test (Accelerations)")
plt.legend()
plt.show()

plt.plot(times, positions)
plt.xlabel("Time (s)")
plt.ylabel("Position (deg)")
plt.title("Backdrivability Tes (Positions)")
plt.legend()
plt.show()