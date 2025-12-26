# Data structure for robot status description

# timestamp
float64 timestamp                    # time stamp   

# joint status
float64[] joint_pose                 # joint_pose
float64[] joint_velocity             # joint_velocity
float64[] joint_cmd_acceleration         # joint_acceleration
float64[] joint_torque               # joint_torque
float64[] joint_filter_torque        # joint_filter_torque

# arm angle state
float64 arm_angle                    # arm angle