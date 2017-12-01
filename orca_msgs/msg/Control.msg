# Mode
uint8 disarmed=0    # Thrusters are off; autopilot is off; all joystick buttons except "arm" are ignored
uint8 manual=1      # Thrusters are on; manual thruster control
uint8 stabilize=2   # Thrusters are on; autopilot is on and controlling yaw
uint8 depth_hold=3  # Thrusters are on; autopilot is on and controlling yaw and depth
uint8 mode

# Camera tilt in degrees, from -45 (down) to +45 (up)
int32 camera_tilt

# Light brightness, from 0 to 100
int32 brightness

# Thruster efforts, from -1.0 (full reverse) to 1.0 (full forward)
float64[] thruster_efforts