# time: time when command was issued
time time
# t_k: how long the trajectory should take
std_msgs/Duration t_k

# t_k_prime: transition time (only for linear polynomial blending)
std_msgs/Duration t_k_prime

# names of joint angles to be moved (all three lists should be in the same order)
string[] names

# final joint position for trajectory
float64[] q_final

# final joint velocity for trajectory (not used for multi-point paths)
float64[] qdot_final
