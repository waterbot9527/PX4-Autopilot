# Water depth derived from pressure sensor (e.g., MS5837)

uint64 timestamp            # time since system start (microseconds)
uint32 device_id            # unique device ID

float32 pressure_mbar       # pressure (mbar)
float32 temperature_celsius # temperature (degC)
float32 depth_m             # depth (m, positive down)
