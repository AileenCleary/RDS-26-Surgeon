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

# Configure
print("Configuring...")
odrv0.axis0.set_abs_pos(odrv0.axis0.pos_estimate - int(odrv0.axis0.pos_estimate))
odrv0.axis0.config.motor.torque_constant = 1.0
odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)

if odrv0.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print(f"Could not enter Closed Loop. Error: {hex(odrv0.axis0.active_errors)}")
    exit()

print("Starting control...")
start = time.time()
duration = 5
rom = 90*3/2
amp = rom*0.75/2
center = -amp
freq = 2

prev_err = 0
err_i = 0
dt = 0.01
kp = 0.0015
kd = 0.00003
ki = 0

times = []
target_positions = []
actual_positions = []

while time.time() - start < duration:
    t = time.time() - start
    target_pos_deg = amp*math.sin(2*math.pi*freq*t) + center
    if target_pos_deg > 0:
        target_pos_deg = 0
    if target_pos_deg < -130:
        target_pos_deg = -130
    current_pos_rev = odrv0.axis0.pos_estimate
    current_pos_deg = odrv0.axis0.pos_estimate*360
    curr_err = target_pos_deg - current_pos_deg
    err_d = (curr_err - prev_err) / dt
    err_i = err_i + curr_err
    torque = kp*curr_err + kd*err_d + ki*err_i
    if torque > 2:
        torque = 2
    if torque < -2:
        torque = -2

    times.append(t)
    target_positions.append(target_pos_deg)
    actual_positions.append(current_pos_deg)
    print(f"Current position: {current_pos_deg} | Target position: {target_pos_deg} | Torque: {torque}")
    odrv0.axis0.controller.input_torque = torque
    prev_err = curr_err
    time.sleep(dt)

odrv0.axis0.requested_state = AxisState.IDLE

plt.plot(times, actual_positions, label="Actual", color="blue")
plt.plot(times, target_positions, label="Target", color="red")
plt.xlabel("Time (s)")
plt.ylabel("Position (deg)")
plt.title("Target vs. Actual Trajectory Motor Control")
plt.legend()
plt.show()