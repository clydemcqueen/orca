# Header
Header header

# Mode
uint8 disarmed=0    # Thrusters are off; autopilot is off; all joystick buttons except "arm" are ignored
uint8 manual=1      # Thrusters are on; manual thruster control
uint8 hold_h=2      # Thrusters are on; autopilot is on and controlling heading
uint8 hold_d=3      # Thrusters are on; autopilot is on and controlling depth
uint8 hold_hd=4     # Thrusters are on; autopilot is on and controlling heading and depth
uint8 mode

# Camera tilt pwm, range [1100, 1900]
# 1100 = tilt 45 degrees up
# 1500 = straight ahead
# 1900 = tilt 45 degrees down
uint16 camera_tilt_pwm

# Light brightness pwm, range [1100, 1900]
# 1100 = lights off
# 1900 = lights on full
uint16 brightness_pwm

# Thruster efforts, range [1100, 1900]
# 1100 = full reverse
# 1500 = stop
# 1900 = full forward
uint16[] thruster_pwm