Header header
float64 time    #ping time
float64 dt1     #time from trigger to center of water track cell
float64 dt2     #time from start of output message to center of water track cell
geometry_msgs/Vector3 velocity # m/s
# If velocity is invalid its covariance should be set to -1
float64[9] velocity_covariance
float64 figure_of_merit
float64[4] beam_distance
float64 battery_voltage
float64 speed_sound
float64 pressure
float64 temp
