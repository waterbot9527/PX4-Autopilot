# Waterbot EKF2 使用说明

本文档说明 Waterbot 上 EKF2 的传感器选择、常见现象与排查方法。适用当前定制固件（wbot_main_driver + 多 IMU 发布 + 关闭磁融合）。

## 1. EKF2 使用哪颗 IMU

EKF2 通过 `sensor_selection` 选中的设备 ID 来确定所用加速度计/陀螺仪。查看方式：

- `listener sensor_selection`
- `listener estimator_status`

重点字段：
- `accel_device_id`
- `gyro_device_id`

这两个 ID 与 `sensor_accel`/`sensor_gyro` 的实例 `device_id` 一一对应。

### 对应关系示例

```
listener sensor_gyro
listener sensor_accel
listener sensor_selection
```

如果 `sensor_selection.gyro_device_id = 14123300`，那么 EKF2 使用的就是 `sensor_gyro` 中 `device_id=14123300` 的实例。

## 2. 传感器优先级与切换

EKF2 选用的 IMU 由传感器投票器决定，投票器优先级来自校准槽位参数：

- `CAL_GYROx_ID` / `CAL_ACCx_ID`：槽位与设备的对应关系
- `CAL_GYROx_PRIO` / `CAL_ACCx_PRIO`：槽位的优先级（0=禁用，100=最高）

推荐切换步骤：

1. 确认槽位与设备 ID 对应关系：
   - `param show CAL_GYRO0_ID`
   - `param show CAL_GYRO1_ID`
   - `param show CAL_ACC0_ID`
   - `param show CAL_ACC1_ID`

2. 设置优先级（示例：禁用槽位0，启用槽位1）：
   - `param set CAL_GYRO0_PRIO 0`
   - `param set CAL_GYRO1_PRIO 100`
   - `param set CAL_ACC0_PRIO 0`
   - `param set CAL_ACC1_PRIO 100`

3. 保存并重启：
   - `param save`
   - `reboot`

重启后使用 `listener sensor_selection` 验证切换结果。

### 2.1 仅发布单颗 IMU（wbot_main_driver 启动参数）

如果不希望校准改动 EKF2 与 IMU 的对应关系，可以让驱动只发布其中一颗 IMU：

- `-i -1`：默认，发布全部 IMU
- `-i 0`：仅发布 dev0 的 IMU
- `-i 1`：仅发布 dev1 的 IMU

注意：如果使用 `-i 1`，请确保 `-d` 至少为 1（例如 `-d 1 -i 1`），否则 dev1 不会被初始化。

## 3. 典型现象与解释

### 3.1 Roll 约 180°
如果机体倒放，`vehicle_attitude` 的 Roll 约 180° 是正常现象，不表示 EKF2 错误。

### 3.2 Yaw 漂移
在关闭磁融合（`EKF2_MAG_TYPE=0`）时，Yaw 只能靠陀螺积分，长期漂移是正常现象。

### 3.3 磁场异常告警
如果磁融合关闭，`estimator_status` 中 `pre_flt_fail_innov_mag_field_disturbed` 可能为 True，可忽略。

## 4. 建议的检查顺序

1. `listener sensor_selection` 确认 EKF2 当前使用 IMU
2. `listener sensor_gyro` / `listener sensor_accel` 对照 `device_id`
3. `listener vehicle_attitude` 验证姿态方向
4. 如需切换 IMU，调整 `CAL_*_PRIO` 并重启

## 5. 常用参数

- `EKF2_MAG_TYPE=0`：关闭磁融合（当前 Waterbot 推荐）
- `CAL_GYROx_PRIO` / `CAL_ACCx_PRIO`：选择主 IMU
- `CAL_GYROx_ID` / `CAL_ACCx_ID`：槽位与设备绑定

## 6. 与代码的对应关系（索引）

- 传感器投票器读取优先级：
  - src/modules/sensors/voted_sensors_update.cpp
- 传感器校准槽位加载：
  - src/modules/sensors/sensors.cpp
- EKF2 读取 `sensor_selection`：
  - src/modules/ekf2/EKF2.cpp
