# TargetRelative.msg
# Relative motion command with respect to the current pose (Euler angles)

string reference_frame     # "base" or "tcp" (default: "tcp")

float64 dx                 # Relative translation (m)
float64 dy
float64 dz
float64 drx                # Relative rotation (deg)
float64 dry
float64 drz

# Optional parameters
float64 max_vel_scale      # Velocity scaling (0.0 ~ 1.0, 0 means default)
float64 max_acc_scale      # Acceleration scaling (0.0 ~ 1.0, 0 means default)
