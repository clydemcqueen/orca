# Header
Header header

# Mode
uint8 disarmed=0    # Thrusters are off, all joystick buttons except "arm" are ignored
uint8 manual=1      # Manual thruster control
uint8 hold_h=2      # Autopilot is controlling heading
uint8 hold_d=3      # Autopilot is controlling depth
uint8 hold_hd=4     # Autopilot is controlling heading and depth
uint8 mission=5     # Autopilot is running a mission, and will switch to manual when complete
uint8 sos=99        # Thrusters are off, joystick is ignored, lights and radios are calling for help
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