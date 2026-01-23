
## 1. 校准流程

1.启动px4控制台使用
listener sensor_gyro
listener sensor_accel
listener sensor_mag
listener vehicle_attitude  欧拉角
或者是使用wbot_main_driver pd
查看当前的原始数值，也就是当前状态。
务必保证没有设置任何参数。
param set CAL_ACC0_ROT 0
param set CAL_GYRO0_ROT 0
param set CAL_MAG0_ROT 0
param set SENS_BOARD_ROT 0
param show SENS_BOARD_ROT //检查整体旋转参数设置
param save \\\\保存参数
2.src/lib/conversion/rotation.h
进入这个文件更改static constexpr rot_lookup_t rot_lookup[ROTATION_MAX] = {
41-90是新添加的，0-40是系统原有矩阵

3.根据listener vehicle_attitude输出的数据，对矩阵进行调整
listener vehicle_attitude
对应矩阵的三个数字
如果vehicle_attitude中Roll:  deg, Pitch:  deg, Yaw: deg有偏差
少了多少度就在矩阵中加多少
如Pitch是-10 deg 那就在选定参数的矩阵中加10
如Pitch是+10 deg 那就在选定参数的矩阵中减10

4.参数选定src/lib/conversion/rotation.h中新添加了	ROTATION_ROLL_356_PITCH_45       = 64,// Note: This should be ROTATION_ROLL_180_PITCH_45
其中前面是枚举名后面是矩阵编号
可以用
param set CAL_ACC0_ROT 64
param set CAL_GYRO0_ROT 64
param set CAL_MAG0_ROT 64
param show SENS_BOARD_ROT //整体设置
param set SENS_BOARD_ROT 0
更改后使用
sensors stop
sensors start
确保参数生效
来选择使用目前的矩阵，假设选定了64那么想要更细致的矩阵度数就可以在编号64的矩阵中加减。
校准需要保证Roll:  deg, Pitch:  deg这两个角度接近于0或者+—5度以内。

5.校准
启动px4控制台使用
pxh>
commander calibrate accel  \\加速度校准，需要转动传感器六个面都需要在朝下五秒。sensor_accel: x=-0.096, y=-0.654, z=9.725 (m/s^2)x和y需要接近于0，z需要接近于9.8。

commander calibrate gyro   \\陀螺仪校准，需要全程保持禁止，sensor_gyro: x=-0.001, y=0.007, z=0.011 (rad/s)三项接近于0。

commander calibrate mag    \\磁力计校准需要满足sensor_mag topic 有数据、日志里不报 Found 0 compass、未解锁、地磁强度在~10 µT  < |B| < ~100 µT  xyz的平方开根的数值就是|B| 数值要在这个区间内。

commander calibrate level  \\水平校准，目前位置就是平的，只需要放着不动，告诉px4当前状态就是世界坐标Roll: 0deg, Pitch: 0deg，只能纠正几度内的偏差。
