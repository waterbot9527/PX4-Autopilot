# wbot motor

uint64 timestamp # time since system start (microseconds)

# 给每个 单片机发的 控制电机消息

# motor speed
uint8[4] speed #  0-255  转速
uint8[4] direction # 0 : 正转， 1 反转

