# TargetPose.msg
# Absolute Cartesian pose command (Euler angles)

float64 x                  # Target position (m)
float64 y
float64 z
float64 rx                 # Target orientation (deg) - Roll
float64 ry                 # Pitch
float64 rz                 # Yaw

# Optional parameters
float64 max_vel_scale      # Velocity scaling (0.0 ~ 1.0, 0 means default)
float64 max_acc_scale      # Acceleration scaling (0.0 ~ 1.0, 0 means default)
