# Input message that is to be sent to the vehicle
#
# The input includes steering, throttle, and braking. 

std_msgs/Header header

# Steering value [-1,1]
# -1 indicates turn fully to the left, 1 indicates turn fully to the right
# Independent of the max turn of the vehicle hardware
float64 steering

# Throttle value [0, 1]
# 0 indicates no throttle, 1 indicates 100% throttle
# Independent of the acceleration profile for the car
float64 throttle

# Braking value [0, 1]
# 0 indicates no braking, 1 indicates 100% braking
float64 braking

