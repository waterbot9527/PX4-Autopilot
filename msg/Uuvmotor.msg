uint64 timestamp

# Device and motor identifiers
uint8 device_id        # MCU device index (dev_id)
uint8 motor_id         # Motor index (moto_index)

# Status bits (single-bit fields exposed as bool)
bool over_current
bool over_temp
bool over_voltage
bool under_voltage
bool charge_pump_under_voltage
bool motor_fault
bool high_z_mode
bool clear_fault

bool open_loop_lock
bool closed_loop_lock
bool bemf_error
bool speed_error
bool no_motor
bool current_fault
bool hall_fault
bool pump_no_load

# Multi-byte fields
uint16 motor_speed    # LSB+MSB combined (raw)
uint16 motor_period   # raw period
uint16 bemf_ke        # raw Ke

float32 supply_voltage        # supply voltage in volts (scaled: raw * 0.122)
float32 motor_current         # motor current in amps (scaled: raw * 5.86e-3)
