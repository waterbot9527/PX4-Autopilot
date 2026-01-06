# PX4 uORB (micro Object Request Broker) 说明文档

## 1. 概述

uORB (micro Object Request Broker) 是 PX4 中用于模块间通信的发布/订阅消息系统。它允许不同模块之间异步传输数据，是 PX4 架构的核心组件之一。

### 1.1 uORB 的特点
- **轻量级**: 针对嵌入式系统优化
- **异步**: 发布者和订阅者无需同步
- **多实例**: 支持同一主题的多个实例
- **跨进程**: 支持进程间通信
- **高效**: 最小化数据复制和内存使用

### 1.2 uORB 的用途
- 传感器数据传输
- 控制指令传递
- 状态信息共享
- 诊断信息传递

## 2. 核心概念

### 2.1 发布/订阅模式
uORB 采用发布/订阅模式，其中:
- **发布者 (Publisher)**: 发送数据到特定主题
- **订阅者 (Subscriber)**: 订阅特定主题并接收数据
- **主题 (Topic)**: 定义数据结构和传输协议

### 2.2 消息定义
uORB 消息在 `.msg` 文件中定义，使用简单的语法描述数据结构：

```
uint64 timestamp
float32[3] xyz
uint8[4] quaternion
```

## 3. uORB 在 PX4 中的架构

### 3.1 主要组件
- `uORBManager`: 管理所有主题的发布和订阅
- `uORBDeviceNode`: 表示单个主题的设备节点
- `uORBDeviceMaster`: 管理所有主题的主设备
- `uORBCommunicator`: 处理跨进程通信

### 3.2 消息传输机制
- `Subscription`: 基本订阅类
- `SubscriptionBlocking`: 阻塞式订阅
- `SubscriptionCallback`: 回调式订阅
- `SubscriptionInterval`: 定时间隔订阅
- `SubscriptionMultiArray`: 多实例数组订阅
- `Publication`: 基本发布类
- `PublicationMulti`: 多实例发布

### 3.3 消息格式
- 每个消息都包含时间戳字段
- 支持基本数据类型和数组
- 支持嵌套消息结构

## 4. uORB 消息定义详解

### 4.1 传感器消息
- `SensorCombined.msg`: 组合传感器数据（加速度计、陀螺仪、磁力计等）
- `SensorAccel.msg`: 加速度计数据
- `SensorAccelFifo.msg`: 加速度计FIFO数据
- `SensorGyro.msg`: 陀螺仪数据
- `SensorGyroFifo.msg`: 陀螺仪FIFO数据
- `SensorGyroFft.msg`: 陀螺仪FFT数据
- `SensorMag.msg`: 磁力计数据
- `SensorBaro.msg`: 气压计数据
- `SensorAirflow.msg`: 气流传感器数据
- `SensorOpticalFlow.msg`: 光流传感器数据
- `SensorGps.msg`: GPS传感器数据
- `SensorHygrometer.msg`: 湿度传感器数据
- `VehicleMagnetometer.msg`: 载体磁力计数据
- `VehicleImu.msg`: 载体IMU数据
- `VehicleImuStatus.msg`: 载体IMU状态

### 4.2 状态消息
- `VehicleStatus.msg`: 载体状态
- `VehicleAttitude.msg`: 载体姿态
- `VehicleLocalPosition.msg`: 本地位置
- `VehicleGlobalPosition.msg`: 全球位置
- `VehicleAngularVelocity.msg`: 载体角速度
- `VehicleAcceleration.msg`: 载体加速度
- `Cpuload.msg`: CPU负载
- `EstimatorStatus.msg`: 估计器状态
- `EstimatorStatusFlags.msg`: 估计器状态标志
- `EstimatorInnovations.msg`: 估计器创新数据

### 4.3 控制消息
- `ActuatorControls.msg`: 执行器控制
- `ActuatorOutputs.msg`: 执行器输出
- `VehicleAttitudeSetpoint.msg`: 姿态设定点
- `VehicleLocalPositionSetpoint.msg`: 本地位置设定点
- `VehicleGlobalPositionSetpoint.msg`: 全球位置设定点
- `VehicleRatesSetpoint.msg`: 角速率设定点
- `VehicleTorqueSetpoint.msg`: 转矩设定点
- `VehicleThrustSetpoint.msg`: 推力设定点

### 4.4 导航与任务消息
- `PositionSetpoint.msg`: 位置设定点
- `PositionSetpointTriplet.msg`: 位置设定点三元组
- `Mission.msg`: 任务数据
- `MissionResult.msg`: 任务结果
- `NavigatorMissionItem.msg`: 导航器任务项
- `NavigatorStatus.msg`: 导航器状态

### 4.5 输入消息
- `InputRc.msg`: 遥控输入
- `ManualControlSetpoint.msg`: 手动控制设定点
- `ManualControlSwitches.msg`: 手动控制开关
- `RcChannels.msg`: 遥控通道数据

### 4.6 诊断与调试消息
- `DebugKeyValue.msg`: 调试键值对
- `DebugValue.msg`: 调试值
- `DebugArray.msg`: 调试数组
- `DebugVect.msg`: 调试向量
- `LogMessage.msg`: 日志消息
- `TaskStackInfo.msg`: 任务栈信息

## 5. uORB API 使用方法

### 5.1 订阅消息

```cpp
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_combined.h>

// 订阅 sensor_combined 主题
uORB::Subscription sensor_sub{ORB_ID(sensor_combined)};
sensor_combined_s sensor_data{};

// 检查是否有新数据
if (sensor_sub.updated()) {
    sensor_sub.copy(&sensor_data);
    // 处理数据
    PX4_INFO("acc: [%.2f, %.2f, %.2f]", 
             (double)sensor_data.accelerometer_m_s2[0],
             (double)sensor_data.accelerometer_m_s2[1],
             (double)sensor_data.accelerometer_m_s2[2]);
}
```

### 5.2 订阅多个实例

```cpp
#include <uORB/SubscriptionMultiArray.hpp>

// 订阅最多8个传感器实例
uORB::SubscriptionMultiArray<sensor_accel_s> accel_subs{ORB_ID(sensor_accel), 8};

for (auto &sub : accel_subs) {
    if (sub.updated()) {
        sensor_accel_s accel_data{};
        sub.copy(&accel_data);
        // 处理数据
    }
}
```

### 5.3 发布消息

```cpp
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>

// 创建发布者
uORB::Publication<vehicle_attitude_s> attitude_pub{ORB_ID(vehicle_attitude)};

// 准备数据
vehicle_attitude_s attitude{};
attitude.timestamp = hrt_absolute_time();
// 设置其他字段...

// 发布数据
attitude_pub.publish(attitude);
```

### 5.4 多实例发布

```cpp
#include <uORB/PublicationMulti.hpp>

// 创建多实例发布者
uORB::PublicationMulti<vehicle_imu_s> imu_pub{ORB_ID(vehicle_imu)};

// 发布到特定实例
imu_pub.get().instance();
```

### 5.5 回调式订阅

```cpp
#include <uORB/SubscriptionCallback.hpp>

class MyModule : public ModuleBase<MyModule>
{
public:
    MyModule() : 
        _sensor_sub(this, ORB_ID(sensor_combined))
    {
        _sensor_sub.registerCallback();
    }

private:
    uORB::SubscriptionCallbackWorkItem _sensor_sub;
    
    void Run() override
    {
        if (_sensor_sub.updated()) {
            sensor_combined_s data;
            _sensor_sub.copy(&data);
            // 处理数据
        }
    }
};
```

## 6. uORB 在 WBotMainDriver 中的使用

在我们的 [WBotMainDriver](file:///home/ici/work/PX4-Autopilot/src/drivers/wbot_main_driver/WBotMainDriver.cpp#L36-L36) 实现中，我们添加了对 IMU 和磁力计数据的记录功能：

```cpp
// 在头文件中添加订阅者
uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
uORB::Subscription _mag_sub{ORB_ID(vehicle_magnetometer)};

// 在类中添加数据结构和缓冲区
struct ImuMagData {
    uint64_t timestamp;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
};
std::queue<ImuMagData> _imu_mag_buffer;

// 记录 IMU 和磁力计数据的方法
void WBotMainDriver::log_imu_mag_data()
{
    // 订阅 sensor_combined 主题
    if (_sensor_combined_sub.updated()) {
        sensor_combined_s sensor_data{};
        _sensor_combined_sub.copy(&sensor_data);

        // 记录 IMU 数据到缓冲区
        ImuMagData data;
        data.timestamp = sensor_data.timestamp;
        data.accel_x = sensor_data.accelerometer_m_s2[0];
        data.accel_y = sensor_data.accelerometer_m_s2[1];
        data.accel_z = sensor_data.accelerometer_m_s2[2];
        data.gyro_x = sensor_data.gyro_rad[0];
        data.gyro_y = sensor_data.gyro_rad[1];
        data.gyro_z = sensor_data.gyro_rad[2];

        // 将数据添加到缓冲区
        _imu_mag_buffer.push(data);
    }

    // 订阅 vehicle_magnetometer 主题
    if (_mag_sub.updated()) {
        vehicle_magnetometer_s mag_data{};
        _mag_sub.copy(&mag_data);

        // 更新缓冲区中的磁力计数据
        if (!_imu_mag_buffer.empty()) {
            _imu_mag_buffer.back().mag_x = mag_data.magnetometer_ga[0];
            _imu_mag_buffer.back().mag_y = mag_data.magnetometer_ga[1];
            _imu_mag_buffer.back().mag_z = mag_data.magnetometer_ga[2];
        }
    }
}
```

## 7. uORB 主题命名约定

uORB 主题通常遵循以下命名约定：
- 传感器数据: `sensor_*` (如 `sensor_accel`, `sensor_gyro`)
- 载体状态: `vehicle_*` (如 `vehicle_attitude`, `vehicle_local_position`)
- 执行器: `actuator_*` (如 `actuator_controls`, `actuator_outputs`)
- 估计器: `estimator_*` (如 `estimator_status`, `estimator_innovations`)
- 任务: `mission_*` (如 `mission`, `mission_result`)

## 8. uORB 命令行工具

PX4 提供了 [uorb](file:///home/ici/work/PX4-Autopilot/src/systemcmds/uorb/uorb.cpp#L92-L92) 命令行工具用于调试:

- `uorb status`: 显示所有主题的状态
- `uorb info <topic_name>`: 显示特定主题的详细信息
- `uorb listen <topic_name>`: 监听并显示主题数据
- `uorb listen <topic_name> -n <count>`: 监听指定数量的消息
- `uorb listen <topic_name> -i <instance>`: 监听特定实例

## 9. 性能考虑

### 9.1 内存使用
- 避免频繁创建和销毁订阅者/发布者
- 使用适当大小的队列深度
- 在多实例场景中考虑内存使用

### 9.2 CPU 使用
- 避免在高频率循环中不必要的 `updated()` 检查
- 合理设置队列深度以平衡延迟和性能
- 使用回调机制而不是轮询，当适用时

### 9.3 数据一致性
- 时间戳用于同步不同传感器数据
- 考虑数据的新鲜度和一致性
- 使用适当的采样率匹配传感器硬件

## 10. 最佳实践

1. **使用正确的数据类型**: 确保消息定义中的数据类型与实际使用一致
2. **检查更新**: 在访问数据前始终检查 `updated()` 方法
3. **避免阻塞**: 在实时代码中避免使用阻塞式订阅
4. **资源管理**: 适当管理订阅和发布对象的生命周期
5. **性能优化**: 使用多实例订阅来处理多个传感器
6. **错误处理**: 检查订阅和发布操作的结果
7. **命名约定**: 使用一致的命名约定和清晰的变量名
8. **时间戳**: 始终使用时间戳进行数据同步

## 11. 调试技巧

1. 使用 `uorb listen <topic>` 命令查看实时数据
2. 使用 `uorb status` 检查主题发布/订阅状态
3. 在代码中使用 `PX4_DEBUG` 或 `PX4_INFO` 输出调试信息
4. 检查时间戳以验证数据的新鲜度
5. 使用 `uorb info` 查看主题的发布者和订阅者数量
6. 使用多实例功能调试多个传感器

## 12. 常见问题与解决方案

### 12.1 数据丢失
- 增加队列深度
- 检查处理速度是否足够

### 12.2 数据不同步
- 使用时间戳同步
- 检查传感器采样率

### 12.3 内存不足
- 减少不必要的订阅
- 优化队列大小

## 13. 高级主题

### 13.1 自定义消息类型
创建新的 `.msg` 文件并添加到构建系统中：

```
uint64 timestamp
float32[3] position
float32[3] velocity
```

### 13.2 跨进程通信
uORB 支持跨进程通信，通过 `uORBCommunicator` 接口实现。

### 13.3 动态订阅
在运行时动态创建订阅，根据需要订阅不同的主题。

## 14. 参考资料

- [PX4 官方文档 - uORB](https://docs.px4.io/main/en/middleware/uorb.html)
- [PX4 开发指南 - uORB](https://px4.github.io/px4-guides/docs/)
- [uORB 消息定义](https://github.com/PX4/PX4-Autopilot/tree/main/msg)
- [PX4 API 文档](https://dev.px4.io/main/en/)
- [ROS 与 uORB 集成](https://docs.px4.io/main/en/companion_computer/passthrough.html)

---
*文档最后更新: 2026-01-05*