

# upload cmake :

boards/px4/raspberrypi/cmake/upload.cmake

推荐用 CMake 缓存变量指定上传地址（优先级高于环境变量）：

```bash
cmake -S . -B build/px4_raspberrypi_default -DAUTOPILOT_HOST=192.168.1.154
make px4_raspberrypi_default upload
```

兼容旧方式（仅当未传 `-DAUTOPILOT_HOST` 时生效）：

```bash
export AUTOPILOT_HOST=192.168.1.154
cmake -S . -B build/px4_raspberrypi_default
make px4_raspberrypi_default upload
```
