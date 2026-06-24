# Waterbot 电机控制调试指南

## 问题症状
启动 `wbot_actuator_bridge start` 后，电机无动作。

## 控制链路
```
manual_control (驱动输入)
       ↓
vehicle_control_mode (控制模式检查)
       ↓
uuv_att_control (姿态控制器) → vehicle_torque_setpoint + vehicle_thrust_setpoint
       ↓
control_allocator (控制分配) → actuator_motors
       ↓
wbot_actuator_bridge (转换) → wbot_ctrl_moto
       ↓
硬件驱动 (收集电机命令)
```

## 快速排查清单

### 1. 验证启动顺序和模式 (30秒)

在 PX4 控制台执行：
```bash
# 检查是否已进入 MANUAL 模式
status

# 检查是否已上锁
arm_disarm status

# 如果未上锁，执行：
arm_disarm arm
```

**预期结果**：
- 状态显示 "MANUAL" 或 "MANUAL_READY"
- 显示 "armed"

---

### 2. 验证传感器数据畅通 (30秒)

```bash
# 检查姿态估计是否工作
listener vehicle_attitude

# 检查角速度是否发布
listener vehicle_angular_velocity

# 检查 EKF2 状态
listener estimator_status
```

**预期结果**：各话题都有持续输出，无输出则该传感器有问题。

---

### 3. 验证控制指令链

#### 3.1 检查 manual_control_setpoint
```bash
listener manual_control_setpoint
```
**预期结果**：如果无输出，说明 manual_control 没有输入源（POSIX 环境下需要通过 MAVLink 或脚本提供）。

#### 3.2 检查 uuv_att_control 的输出
```bash
listener vehicle_torque_setpoint
listener vehicle_thrust_setpoint
```
**预期结果**：需要有非零值。如果全是 0，说明：
- uuv_att_control 未收到输入指令
- vehicle_control_mode 未允许控制
- attitude_setpoint 或 rates_setpoint 无数据

#### 3.3 检查 control_allocator 的输出
```bash
listener actuator_motors
```
**预期结果**：
- `control[0-7]` 应该有 -1.0 到 1.0 的值
- 如果全是 0，说明控制分配矩阵失败

检查分配器状态：
```bash
listener control_allocator_status
```
查看 `unallocated_torque` 和 `actuator_saturation` 是否异常高。

#### 3.4 检查 wbot_actuator_bridge 的输出
```bash
listener wbot_ctrl_moto
```
**预期结果**：
- `speed[0-7]` 应该有 0-255 的值
- `direction[0-7]` 应该有 0 或 1

---

### 4. 参数检查

检查关键参数是否配置正确：
```bash
# 检查控制分配配置
param show CA_AIRFRAME       # 应该是 7 (6DOF_UUV)
param show CA_ROTOR_COUNT    # 应该是 8
param show CA_R_REV          # 应该是 255 (所有可逆)

# 检查每个转子的配置（是否都已设置）
param show CA_ROTOR0_PX
param show CA_ROTOR0_AX
param show CA_ROTOR0_KM      # 应该 > 0（不能是 0）

# 检查反应力矩系数
for i in 0 1 2 3 4 5 6 7; do param show CA_ROTOR${i}_KM; done
```

**常见问题**：
- CA_ROTOR*_KM 为 0 → 矩阵可能奇异，无法分配
- CA_ROTOR*_AX/AY/AZ 不满足单位向量 (||A|| ≠ 1) → 导致异常力矩
- CA_ROTOR_COUNT ≠ 实际转子数

---

### 5. POSIX 环境下的输入模拟

在树莓派/POSIX 环境下，没有实际的 RC 遥控器，需要通过脚本模拟输入：

#### 5.1 通过 MAVLink Offboard 模式发送命令

使用 MAVSDK 或 Python MAVLink 库：
```python
# 伪代码示例
drone.set_mode(Mode.OFFBOARD)
drone.arm()
drone.offboard.set_attitude(roll=0.1, pitch=0.1, yaw=0, thrust=0.5)
drone.offboard.start()
```

#### 5.2 通过 PX4 内部脚本手动注入

在 PX4 控制台：
```bash
# 注入一个姿态指令
orb_publish vehicle_attitude_setpoint "{timestamp: $(hrt_absolute_time), roll_body: 0.1, pitch_body: 0.1, yaw_body: 0.0, thrust_body: [0.0, 0.0, 0.5]}"
```

---

### 6. 如果上述都正常，但电机仍不动

#### 6.1 检查硬件驱动
```bash
# 检查 wbot_main_driver 是否正常
ps | grep wbot_main_driver

# 查看驱动日志
dmesg | grep -i wbot

# 测试驱动是否能接收 wbot_ctrl_moto
listener wbot_ctrl_moto
# 应该每 50ms 左右有一条消息
```

#### 6.2 检查电源和硬件连接
- 确保树莓派与电机驱动板的通信线缆正确连接
- 确保电机驱动板已通电
- 检查 GPIO/I2C/SPI 总线是否工作

#### 6.3 启用调试日志
```bash
# 启用 wbot_actuator_bridge 的详细日志
param set SYS_LOGGER_HIL_EN 1
# 重启并查看日志
```

---

### 7. 完整诊断脚本

将以下命令序列保存为 `motor_debug.sh`，一键检查所有环节：

```bash
#!/bin/sh
echo "=== Motor Control Debug Checklist ==="

echo "1. System Status"
status

echo "2. Arming Status"
arm_disarm status

echo "3. IMU & EKF"
listener vehicle_attitude -n 3
listener vehicle_angular_velocity -n 1
listener estimator_status -n 1

echo "4. Control Mode"
listener vehicle_control_mode -n 1

echo "5. Manual Input"
listener manual_control_setpoint -n 1

echo "6. Torque/Thrust Setpoints"
listener vehicle_torque_setpoint -n 1
listener vehicle_thrust_setpoint -n 1

echo "7. Actuator Motors (control_allocator output)"
listener actuator_motors -n 1

echo "8. WBOT Motor Command (bridge output)"
listener wbot_ctrl_moto -n 1

echo "9. Control Allocator Status"
listener control_allocator_status -n 1

echo "=== End Debug ==="
```

---

## 常见问题排解

| 症状 | 可能原因 | 解决方案 |
|------|--------|--------|
| 所有话题都无输出 | EKF2 或传感器未启动 | 检查 `listener estimator_status` 和 `listener vehicle_attitude` |
| actuator_motors 全是 0 | uuv_att_control 无输入或未被激活 | 检查 manual_control_setpoint，确保有驱动输入；检查 vehicle_control_mode |
| actuator_motors 有值但 wbot_ctrl_moto 无值 | wbot_actuator_bridge 未正确订阅 | 重启 `wbot_actuator_bridge start` |
| wbot_ctrl_moto 有值但电机不动 | 硬件驱动或连接问题 | 检查树莓派与电机驱动板的通信；测试驱动是否收到命令 |
| CA_ROTOR* 配置为全 0 | 参数文件损坏或未正确加载 | `param reset_all CAL_*`；重新加载机型文件；`reboot` |

---

## 性能调优

一旦电机可以正常响应，进行以下调优：

1. **调整 CA_ROTOR*_KM**：从 0.05 开始，根据实际性能调整到 0.02~0.10
2. **检查饱和情况**：`listener control_allocator_status` 中 `actuator_saturation` 过高表示转子配置需要优化
3. **验证转向**：测试每个 DOF（roll/pitch/yaw/surge/sway/heave），确认推力方向正确
4. **参数微调**：根据日志调整 `UUV_ROLL_P`、`UUV_PITCH_P` 等控制增益

