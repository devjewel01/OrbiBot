# Overall robot system status
std_msgs/Header header

# Power status
float32 battery_voltage    # Battery voltage in Volts

# System state
bool motors_enabled       # True if motors are enabled
bool emergency_stop       # True if emergency stop is active
bool hardware_ok          # True if all hardware is responding