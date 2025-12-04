# TargetJoint.msg
# Joint-space motion command (in radians)

float64[] joints  # Target joint angles (rad)

# Optional parameters
float64 max_vel_scale      # Velocity scaling (0.0 ~ 1.0, 0 means default)
float64 max_acc_scale      # Acceleration scaling (0.0 ~ 1.0, 0 means default)
