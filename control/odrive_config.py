import odrive
import odrive.enums as ODrive
import time

print("Searching for ODrive...")
odrv0 = odrive.find_any()
print(f"Found ODrive with serial number: {odrv0.serial_number}")
print(f"Firmware Version: {odrv0.fw_version_major}.{odrv0.fw_version_minor}.{odrv0.fw_version_revision}")

# Erase old configuration
print("Erasing previous configuration...")
try:
    odrv0.erase_configuration()
except Exception:
    pass

# Reboot ODrive
print("Waiting for ODrive to reboot...")
time.sleep(2)
odrv0 = odrive.find_any()
print("Reconnected! Applying new settings...")

# Clear any existing errors before configuring
odrv0.clear_errors()

# Map to DC bus and current trip levels
odrv0.config.dc_bus_overvoltage_trip_level = 48.0
odrv0.config.dc_bus_undervoltage_trip_level = 10.5
odrv0.config.dc_max_positive_current = 2.63
odrv0.config.dc_max_negative_current = -0.5

# Basic Motor Info
odrv0.axis0.config.motor.motor_type = ODrive.MotorType.PMSM_CURRENT_CONTROL
odrv0.axis0.config.motor.pole_pairs = 4
odrv0.axis0.config.motor.torque_constant = 0.057
odrv0.axis0.config.motor.current_soft_max = 2.63
odrv0.axis0.config.motor.calibration_current = 0.88
odrv0.axis0.config.motor.resistance_calib_max_voltage = 20.0
odrv0.axis0.config.calibration_lockin.current = 0.88

# Configure NME3 encoder (RS422 SSI interface)
odrv0.config.gpio0_mode = ODrive.GpioMode.SPI_A
odrv0.config.gpio18_mode = ODrive.GpioMode.SPI_A
odrv0.axis0.config.load_encoder = ODrive.EncoderId.SPI_ENCODER0
odrv0.axis0.config.commutation_encoder = ODrive.EncoderId.SPI_ENCODER0
odrv0.spi_encoder0.config.mode = ODrive.SpiEncoderMode.NETZER_VLP80
odrv0.spi_encoder0.config.singleturn_bits = 16

# Enable CAN Bus
odrv0.can.config.protocol = ODrive.Protocol.SIMPLE
odrv0.can.config.baud_rate = 250000
odrv0.axis0.config.can.node_id = 1
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv0.axis0.config.can.encoder_msg_rate_ms = 10
odrv0.axis0.config.can.iq_msg_rate_ms = 10
odrv0.axis0.config.can.temperature_msg_rate_ms = 1000
odrv0.axis0.config.can.bus_voltage_msg_rate_ms = 1000

# Save and reboot to apply the new pin mapping
print("Saving configuration...")
try:
    odrv0.save_configuration()
except Exception:
    print("USB disconnected during flash save (This is normal!)")
print("Rebooting...")
try:
    odrv0.reboot()
except Exception:
    print("ODrive rebooting...")
print("Configuration complete!")

print("Waiting for ODrive to come back online...")
time.sleep(2)
odrv0 = odrive.find_any()
odrv0.clear_errors()
print("Reconnected!")

# Run calibration sequence
print("Waiting for encoder to initialize...")
time.sleep(2)
print("Starting full calibration sequence...")
odrv0.axis0.requested_state = ODrive.AxisState.FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != ODrive.AxisState.IDLE:
    time.sleep(0.1)
if odrv0.axis0.active_errors != 0:
    print(f"Calibration failed! Active errors: {odrv0.axis0.active_errors}")
    exit()

# Enter closed-loop control
print("Calibration successful. Saving configuration...")
try:
    odrv0.save_configuration()
except Exception:
    print("USB disconnected during flash save (This is normal!)")
print("Rebooting...")
try:
    odrv0.reboot()
except Exception:
    print("ODrive rebooting...")
print("Configuration complete!")