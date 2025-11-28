# TargetNamed.msg
# Named target command (predefined pose name)

string target_name          # Example: "home", "ready"

# Optional parameters
float64 max_vel_scale       # Velocity scaling (0.0 ~ 1.0, 0 means default)
float64 max_acc_scale       # Acceleration scaling (0.0 ~ 1.0, 0 means default)
