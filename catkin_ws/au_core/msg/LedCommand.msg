# mode enums
uint8 ONE_SHOT=0
uint8 INTERVAL=1

uint8 mode

# The meaning of the duration and interval fields is dependent on the mode
# ONE_SHOT:
#     duration:   Length of pulse in ms. If set to 0, then leds will be
#                 set to desired colour indefinitely
#     interval:   This field has no effect
#     duty_cycle: This field has no effect
# INTERVAL:
#     duration:   Number of pulses to send before stopping
#     interval:   Interval width in ms
#     duty_cycle: Proportion of interval to spend ON (0-1)
uint32 duration
uint32 interval
float32 duty_cycle

# The colour to set the leds
std_msgs/ColorRGBA colour
