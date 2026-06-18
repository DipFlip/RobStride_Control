# RobStride Control Python

Python 实现 RobStride 电机控制库，提供简单易用的 API 和丰富的功能。

## 特性

- ✅ **MIT 模式位置控制**：高性能直接扭矩控制
- ✅ **速度控制**：精确的速度闭环控制
- ✅ **实时控制**：50-100Hz 控制频率
- ✅ **交互式界面**：友好的命令行界面
- ✅ **参数调整**：实时调整控制参数
- ✅ **状态监控**：实时显示电机状态

## 安装

### 环境要求

- Python 3.8+
- Linux 系统 (SocketCAN 支持)
- CAN 接口硬件

### 安装依赖

```bash
# 安装系统依赖
sudo apt-get install python3 python3-pip can-utils

# 克隆项目
git clone https://github.com/tianrking/robstride-control.git
cd robstride-control/python

# 安装 Python 依赖
pip install -r requirements.txt

# 或者使用虚拟环境
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## 快速开始

### 位置控制

```bash
# 运行 MIT 位置控制
python3 src/position_control.py 11
```

### RobStride 官方 USB-CAN 适配器

官方 CH340/GD32 USB-CAN 适配器在 Linux 下显示为串口设备（通常为
`/dev/ttyUSB0`），使用 921600 波特率的 RobStride AT 帧协议。

```bash
# 仅检测 ID 1 和 2，不使能电机
python3 src/dual_relative_move.py --probe-only

# ID 1 相对移动 20 度，ID 2 相对移动 100 度
python3 src/dual_relative_move.py \
  --execute \
  --motor-1-id 1 --motor-1-deg 20 \
  --motor-2-id 2 --motor-2-deg 100 \
  --speed-deg-s 20 \
  --kp 15 --kd 0.3 \
  --torque-limit-nm 2
```

该命令会先验证两个电机 ID，从当前编码器位置平滑移动，并在完成或中断时
禁用两个电机的力矩。

### 速度控制

```bash
# 运行速度控制
python3 src/speed_control.py 11
```

## API 使用

### MIT 位置控制

```python
from src.position_control import PositionControllerMIT

# 创建控制器
controller = PositionControllerMIT(motor_id=11)

# 连接电机
controller.connect()

# 设置位置 (角度)
controller.set_angle(90.0)

# 调整控制参数
controller.set_kp(30.0)  # 位置增益
controller.set_kd(0.5)   # 阻尼增益

# 交互式控制
controller.run_interactive()
```

### 速度控制

```python
from src.speed_control import SpeedController

# 创建控制器
controller = SpeedController(motor_id=11)

# 连接电机
controller.connect()

# 设置速度 (rad/s)
controller.set_velocity(5.0)

# 交互式控制
controller.run_interactive()
```

## 交互式命令

### 位置控制命令

- **数字输入**：设置目标角度（度）
  ```
  90     # 顺时针 90 度
  -45    # 逆时针 45 度
  0      # 零点位置
  ```

- **参数调整**：
  ```
  kp 30  # 设置位置增益为 30
  kd 0.8 # 设置阻尼增益为 0.8
  ```

- **特殊命令**：
  ```
  home   # 回零位
  status # 显示状态
  params # 显示参数
  quit   # 退出
  ```

### 速度控制命令

- **数字输入**：设置目标速度 (rad/s)
  ```
  5.0    # 正向 5 rad/s
  -3.0   # 反向 3 rad/s
  0      # 停止
  ```

- **特殊命令**：
  ```
  stop   # 停止电机
  status # 显示状态
  quit   # 退出
  ```

## 控制模式

### MIT 模式 (Mode 0)
- 直接发送位置、速度、扭矩目标
- 50Hz 控制频率
- 适用于高性能机器人应用

### 速度模式 (Mode 2)
- 基于 VELOCITY_TARGET 参数
- 20Hz 控制频率
- 适用于需要精确速度控制的应用

## 参数说明

### 位置控制参数

| 参数 | 范围 | 说明 |
|------|------|------|
| Kp   | 0-500 | 位置增益，控制响应速度 |
| Kd   | 0-5   | 阻尼增益，抑制震荡 |
| 速度限制 | 0-50 rad/s | 最大速度限制 |
| 扭矩限制 | 0-60 Nm | 最大扭矩限制 |

### 速度控制参数

| 参数 | 范围 | 说明 |
|------|------|------|
| Kp   | 0-500 | 速度比例增益 |
| Ki   | 0-100 | 速度积分增益 |
| 速度限制 | 0-50 rad/s | 最大速度限制 |

## 示例程序

```bash
# 基础使用示例
python3 examples/basic_usage.py 11

# 高级控制示例
python3 examples/advanced_control.py 11
```

## 故障排除

### 常见问题

1. **找不到电机**
   ```bash
   # 检查 CAN 连接
   sudo ip link show can0

   # 扫描电机
   python3 -c "from robstride_dynamics import RobstrideBus; print(RobstrideBus.scan_channel('can0'))"
   ```

2. **权限错误**
   ```bash
   # 添加用户到 dialout 组
   sudo usermod -a -G dialout $USER
   # 重启或重新登录
   ```

3. **CAN 接口未设置**
   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set up can0
   ```

## 性能优化

### 控制频率调整

```python
# 在控制循环中调整 sleep 时间
time.sleep(0.02)  # 50Hz (默认)
time.sleep(0.01)  # 100Hz (高性能)
time.sleep(0.05)  # 20Hz (低负载)
```

### 参数调优

1. **从小参数开始**：Kp=10, Kd=0.1
2. **逐步增加 Kp**：直到响应快速但不震荡
3. **调整 Kd**：消除过冲和震荡
4. **检查温度**：确保电机不过热

## 开发

### 运行测试

```bash
# 安装开发依赖
pip install pytest

# 运行测试
pytest tests/
```

### 代码格式化

```bash
# 安装格式化工具
pip install black flake8

# 格式化代码
black src/
flake8 src/
```

## 许可证

MIT License - 详见 [LICENSE](../LICENSE) 文件

## 贡献

欢迎提交 Issue 和 Pull Request！

## 支持

- 📖 [完整文档](../docs/)
- 🐛 [问题反馈](https://github.com/tianrking/robstride-control/issues)
