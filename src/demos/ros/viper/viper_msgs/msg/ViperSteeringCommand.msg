# Defines a steering command for the ViperDriver class
# If only steering is defined, SetSteering(angle) is called (i.e. wheel_id = ViperWheelID.V_UNDEFINED
# Otherwise, SetSteering(angle, wheel_id) is called

float64 angle
uint8 wheel_id # ViperWheelID