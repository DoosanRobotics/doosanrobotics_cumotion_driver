string move_type      # "pose", "joint", "named", or "relative"

# --- Absolute / Relative Position ---
float32 x
float32 y
float32 z

# --- Orientation (either quaternion or euler, one of them can be 0) ---
float32 qx
float32 qy
float32 qz
float32 qw

float32 rx            # Euler roll (deg)
float32 ry            # Euler pitch (deg)
float32 rz            # Euler yaw (deg)

# --- Relative motion offsets (for move_type='relative') ---
float32 dx            # delta x (m)
float32 dy            # delta y (m)
float32 dz            # delta z (m)
float32 drx           # delta roll (deg)
float32 dry           # delta pitch (deg)
float32 drz           # delta yaw (deg)

# --- Joint motion ---
float32[6] joints

# --- Named target ---
string name

# --- Vel, Acc scaling factor ---
float64 max_vel_scale
float64 max_acc_scale
