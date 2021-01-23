uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample

float32[4] q			# Quaternion rotation from FRD body frame to refernce frame

float32[3] acceleration
float32[3] velocity
float32[3] position

float32 z_deriv

float32 delta_angle_error_norm # angular tracking error magnitude (rad)
float32 velocity_error_norm    # velocity tracking error magnitude (m/s)
float32 position_error_norm    # position tracking error magnitudes (m)
