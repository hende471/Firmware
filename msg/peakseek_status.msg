uint64 timestamp		# time since system start (microseconds)
float32 thrust_est		# Thrust estimated from dynamic model
float32 di_dbeta		# Partial derivative of current with respect to pitch angle
float32 dt_dbeta		# Partial of Thrust with respect to pitch angle
float32 deta_dbeta		# Partial of Efficiency with respect to pitch angle

float32 delbeta			# Stepsize in pitch angle for each iteration
