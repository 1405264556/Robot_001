# e-puck Mini 移动机器人 —— 上位机与固件工程

EPFL（洛桑联邦理工学院）e-puck Mini 教学科研移动机器人的完整上位机程序、固件库和调试工具集。微控制器型号为 dsPIC30F6014A。

---

## 文件夹结构

```
e-puck/
├── library/          # 固件外设驱动库（C 语言，适用于 dsPIC30）
├── program/          # 固件应用程序（烧录到机器人的 .hex 文件）
├── tool/             # PC 端上位机工具
└── README.md         # 本文件
```

---

## 一、library/ —— 固件驱动库

为 e-puck Mini 机器人各外设提供底层驱动和高级功能封装：

| 目录 | 功能说明 |
|------|----------|
| `a_d/` | ADC 模数转换：加速度计、麦克风、接近传感器（红外） |
| `bluetooth/` | 蓝牙串口通信驱动 |
| `camera/` | 摄像头驱动（PO3030K / PO6030K），支持多定时器模式 |
| `codec/` | 音频编解码驱动 |
| `fft/` | 快速傅里叶变换库，用于音频频谱分析 |
| `I2C/` | I2C 总线通信协议 |
| `motor_led/` | 电机、LED 与遥控驱动，支持单定时器/多定时器调度 |
| `uart/` | UART 串口驱动 |
| `matlab/` | MATLAB 串口数据交换接口 |
| `contrib/` | 社区贡献：LIS 传感器扩展、SWIS 通信模块 |

---

## 二、program/ —— 固件应用程序

可直接烧录到机器人的固件程序：

| 目录 | 功能说明 |
|------|----------|
| `BTcom/` | **蓝牙通信固件**（上位机连接必须烧录此程序） |
| `demo/` | 演示程序合集：避障、循线、声源定位、追球、协作等 |
| `bluetooth_mirror/` | 蓝牙镜像/中继程序 |
| `statics libraries maker/` | 静态库生成工程 |

> **预编译 .hex 文件**位于对应目录内，可直接通过 bootloader 烧录。

---

## 三、tool/ —— PC 端上位机工具

### 3.1 e-puck Monitor（Python 版）⭐ 推荐使用

路径：`tool/e-puck_monitor_py/`

**基于 Python/tkinter 的现代化控制面板**，功能完整、界面中文化。

#### 功能

- 📷 摄像头图像采集（单帧 / 连续）
- 📡 红外接近传感器实时显示（8 路）
- 💡 LED 控制（8 颗环形 LED + 身体 LED + 前灯 LED）
- 📐 加速度计姿态与倾角读取
- 🎤 麦克风音频电平
- 🕹️ **虚拟摇杆电机控制**（直观拖拽，松手回弹）
- 🔍 自动扫描 COM 口、自动连接机器人

#### 运行方式

**方式一：直接运行打包好的 exe（无需安装 Python）**

1. 进入 `tool/e-puck_monitor_py/dist/e-puck Monitor/`
2. 双击 `e-puck Monitor.exe`
3. 选择 COM 口 → 点击"连接"

**方式二：从源码运行**

```bash
cd tool/e-puck_monitor_py
pip install pyserial pillow
python epuck_monitor.py
```

**方式三：自行打包**

```bash
cd tool/e-puck_monitor_py
build.bat
```

#### 使用前提

- 机器人已烧录 BTcom 固件（位于 `program/BTcom/`）
- 机器人已通过蓝牙配对（或 USB 串口连接）
- 串口参数：115200 波特率，8 数据位，无校验，1 停止位

#### 蓝牙配对后找不到 COM 口？

- 打开 Windows 设备管理器 → 查看"端口 (COM 和 LPT)"
- 程序支持**自动扫描**所有可用 COM 口并验证是否为 e-puck 机器人

---

### 3.2 e-puck Monitor（原始 C++ Builder 版）

路径：`tool/e-puck_monitor/`

原始 Borland C++ Builder 编写的上位机，需对应编译器构建。仅供参考，建议使用 Python 版本。

---

### 3.3 ePic —— MATLAB 机器人控制器

路径：`tool/ePic/`

在 MATLAB 中控制 e-puck：

```matlab
cd tool/ePic
main
```

---

### 3.4 Bootloader —— 固件烧录工具

路径：`tool/bootloader/`

通过蓝牙将 .hex 固件烧录到机器人：

```bash
epuckupload -f firmware.hex epuck34
```

支持同时向 5 台机器人并行烧录。

---

### 3.5 蓝牙配置脚本 (Linux)

路径：`tool/bluetooth_setup/`

Linux 下的蓝牙 RFCOMM 批量配置脚本，用于同时管理多台机器人。

---

## 快速开始（Windows 用户）

1. **烧录固件**：用 bootloader 将 `program/BTcom/BTcom_default.hex` 烧录到机器人
2. **蓝牙配对**：Windows 设置 → 蓝牙 → 配对 e-puck 机器人（PIN 码默认 0000）
3. **启动控制面板**：运行 `tool/e-puck_monitor_py/dist/e-puck Monitor/e-puck Monitor.exe`
4. **选择 COM 口**：点击"自动扫描"，或手动在下拉菜单中选取
5. **连接**：点击"连接"按钮

---

## 数据存储

所有运行时数据（传感器日志、图像等）存储在应用程序所在目录的 `epuck_data/` 文件夹中，**不会写入 C 盘**。

---

## 技术参数

| 参数 | 值 |
|------|-----|
| 微控制器 | dsPIC30F6014A |
| 串口波特率 | 115200 |
| 摄像头分辨率 | 160×120（灰度） |
| 接近传感器 | 8 路红外 |
| LED | 8 环形 + 身体 + 前灯 |
| 传感器 | 3 轴加速度计、麦克风 |

---

## 参考文献

- EPFL SWIS 小组：http://swis.epfl.ch
- e-puck 官方网站：https://www.epfl.ch/labs/mobots/e-puck/
- 本项目 GitHub：https://github.com/1405264556/Robot_001
