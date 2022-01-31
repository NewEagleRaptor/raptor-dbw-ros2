# IMU command:
# Uses Z-up axis system:
# +x = forward,
# +y = left,
# +z = up
# 
# roll = rotation around X axis
# pitch = rotation around Y axis
# yaw = rotation around Z axis
# longitudinal = +/- X axis
# lateral = +/- Y axis
# vertical = +/- Z axis
#
# Note:
# Yaw/Pitch/Roll commands are received in 
# Z-down orientation & converted to Z-up
sensor_msgs/Imu imu_cmd

# Estimated angular rate measurement latency
uint8 imu_est_latency
