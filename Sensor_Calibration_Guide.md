
# PX4传感器校准完整指南

## 说明

本文档为PX4飞控系统的传感器校准提供了详细的步骤指导，适用于没有任何基础的用户。请按照文档逐步操作，即可完成传感器校准。

## 校准前准备

### 确保安全
- 将飞行器放置在平稳表面上
- 确保螺旋桨已拆除（如果适用）
- 确保周围没有强磁场干扰源
- 校准期间保持飞行器静止

### 连接到PX4系统
通过串口或QGroundControl连接到PX4系统，打开控制台终端。

## 第一步：检查当前传感器状态

### 启动监听器查看传感器数据
在PX4控制台中执行以下命令，检查各传感器当前状态：

```
listener sensor_gyro
listener sensor_accel
listener sensor_mag
listener vehicle_attitude  // 查看欧拉角（Roll, Pitch, Yaw）
```

或者使用wbot_main_driver命令：

```
wbot_main_driver pd、listener vehicle_attitude
```

你可能会看到类似以下的输出：
```
INFO  [wbot_main_driver] sensor_accel: x=-0.109, y=-0.624, z=9.798 (m/s^2)
INFO  [wbot_main_driver] sensor_gyro: x=0.000, y=0.009, z=0.009 (rad/s)
INFO  [wbot_main_driver] vehicle_attitude: Roll=-3.95 deg, Pitch=-0.57 deg, Yaw=56.31 deg
INFO  [wbot_main_driver] CAL_ACC0_ROT: 64
INFO  [wbot_main_driver] CAL_GYRO0_ROT: 64
```

记录这些初始值，它们将帮助我们了解传感器的当前状态。

## 第二步：重置所有校准参数

在开始校准前，我们需要将所有旋转校准参数重置为默认值，以确保从一个干净的状态开始：

```
param set CAL_ACC0_ROT 0
param set CAL_GYRO0_ROT 0
param set CAL_MAG0_ROT 0
param set SENS_BOARD_ROT 0
```

保存参数设置：
```
param save
```

验证参数是否已正确设置：
```
param show SENS_BOARD_ROT
```

## 第三步：确定传感器安装方向偏差

重新运行检查命令：
```
wbot_main_driver pd、listener vehicle_attitude
```

重点关注 `vehicle_attitude` 的输出：
- Roll（横滚角）：绕机身纵轴的角度
- Pitch（俯仰角）：绕机身横轴的角度
- Yaw（偏航角）：绕垂直轴的角度

当你将飞行器水平放置时，Roll和Pitch值应该接近0度。如果它们不为0，则表示传感器安装方向与预期方向存在偏差。

### 如何判断偏差？
- 如果Roll值为-3.95度，这意味着传感器的安装方向相对于理想方向逆时针偏了约4度
- 如果Pitch值为-0.57度，这意味着传感器的安装方向相对于理想方向向下偏了约0.6度

## 第四步：创建自定义旋转校准（如果需要）

如果你的传感器安装存在较大偏差（Roll或Pitch偏离0度超过5度），你可以创建一个自定义旋转校准值。
如步：
param set CAL_ACC0_ROT 41
param set CAL_GYRO0_ROT 41

### 添加自定义旋转值
在PX4源代码的 `src/lib/conversion/rotation.h` 文件中，有一个旋转查找表，找到自定义的参数param set CAL_ACC0_ROT 41
param set CAL_GYRO0_ROT 41所对应的旋转值进行更改/：

```
static constexpr struct {
    float roll;
    float pitch;
    float yaw;
} rot_lookup[] = {
    {  0,   0,   0 },  // ROTATION_NONE = 0
    {  0,   0,  45 },  // ROTATION_YAW_45 = 1
    {  0,   0,  90 },  // ROTATION_YAW_90 = 2
    // ... 更多旋转值
};
```

为了校正传感器安装偏差，你需要添加一个新的旋转值，其数值是第三步中测量到的偏差值的相反数。

例如，如果测量到 Roll=-3.95°, Pitch=-0.57°，则添加：
```
{  4,   1,   0 },  // 自定义校准旋转，约等于 -(-3.95), -(-0.57), 0
可以直接在本机电脑中使用sensor_calibration_assistant.py，来进行计算需要更改多少，并添加到rotation.h中
```

注意：这个步骤需要修改源代码并重新编译固件，同时修改后的数据需要等待ekf2解算一段时间，才能得到正确的结果。

## 第五步：执行传感器自动校准

### 陀螺仪校准
```
commander calibrate gyro
```
执行此命令时，保持飞行器完全静止，直到校准完成。目标是让sensor_gyro的x、y、z值都接近0。

### 加速度计校准
```
commander calibrate accel
```
按照提示，将飞行器放置在6个不同的方向（每个方向的底部朝下），每个方向保持稳定直到校准提示继续。目标是让水平放置时sensor_accel的x和y接近0，z接近9.8m/s²。如果校准时存在错误，如常见的预期传感器的旋转方向相反，就可以尝试更改param set SENS_BOARD_ROT参数，需要具体的角度配置则在rotation.h中进行配置更改。

### 磁力计校准（如果传感器存在且需要）
```
commander calibrate mag
```
按照提示，围绕每个轴旋转飞行器，形成8字形轨迹。注意：
- 确保周围没有磁性干扰
- 飞行器必须处于未解锁状态
- 地磁强度应在10-100 µT范围内

### 水平校准
```
commander calibrate level
```
将飞行器精确水平放置，保持静止直到校准完成。这告诉PX4当前状态是水平的，可用于微调。

## 第六步：验证校准结果

完成所有校准后，再次运行：
```
wbot_main_driver pd
```

检查输出结果，Roll和Pitch值现在应该更接近0度，表明校准已生效。

## 常见问题解决

### Q: 校准时设备移动导致失败怎么办？
A: 重新开始校准过程，确保在提示静止期间设备完全不动。

### Q: 磁力计校准失败，提示"Compass X is not healthy"？
A: 检查周围是否有磁性物体，确保远离金属表面、电子设备和磁铁。

### Q: 加速度计校准失败？
A: 确保按提示将设备的各个面朝下放置足够时间（通常至少5秒），并在过程中保持静止。

### Q: 校准后Roll/Pitch仍然偏离很大？
A: 可能需要重新检查传感器安装，或考虑创建自定义旋转校准值。

## 注意事项

- 确保校准环境没有磁性干扰
- 校准后建议重启飞行器以确保参数完全生效
param set CAL_ACC0_ROT 41
param set CAL_GYRO0_ROT 41


