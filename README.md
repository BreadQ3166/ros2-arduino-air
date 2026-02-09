# 项目简介

本项目实现了一个空气质量实时监测桥接器，通过 Arduino 驱动灰尘与气体传感器进行数据采集与打包，利用串口通讯将信息上传至 ROS 2终端，并以 1Hz 的固定频率发布 `/air_quality_data` 话题。

## 技术规格

- **发布频率**：ROS2节点发布频率 1Hz
- **通讯协议**：
  - 传感器层：UART (115200 bps) + ADC
  - Arduino-ROS2传输层：自定义串口协议 `$START,...,$END`

# Arduino

## 1. 硬件清单

| **组件**     | **型号**             | **说明**                                  |
| ------------ | -------------------- | ----------------------------------------- |
| 主控板       | Arduino Mega 2560    | 核心控制器                                |
| 灰尘传感器   | 微雪 GP2Y1010AU0F    | 支持ADC输出                               |
| 气体传感器   | TVOC气体传感器模组   | 支持 UART 串口输出                        |
| 电平转化模块 | 5V转3.3V四路电平转化 | Arduino(5V)与气体传感器(3.3V)信号电平转化 |

## 2. 接线定义

### 2.1 微雪-GP2Y1010AU0F 灰尘传感器

IO电平为5V，可与Arduino直接连接

| Arduino   | 灰尘传感器 | 管脚描述          |
| --------- | ---------- | ----------------- |
| 5V        | VCC        | 电源正(2.5V-5.5V) |
| GND       | GND        | 电源地            |
| Analog A0 | AOUT       | 电压模拟量输出    |
| Digital 7 | ILED       | 传感器内部LED驱动 |

### 2.2 微雪-TVOC气体传感器模组

IO电平为3.3V，四根信号线通过电平转换模块进行5V-3.3V电平转化。传感器上电后需要等待 2 分钟，传感器预热完成。

| Arduino          | 气体传感器 | 管脚描述                             |
| ---------------- | ---------- | ------------------------------------ |
| 5V               | VCC        | 5V 电源正                            |
| GND              | GND        | 电源地                               |
| Digital 19 (RX1) | TXD        | UART 输出                            |
| Digital 18 (TX1) | RXD        | UART 输入                            |
| Digital 9        | RST        | 复位引脚，低电平有效                 |
| Digital 8        | ALM        | 报警引脚，TVOC超过2ppm，引脚自动拉低 |

## 3. 参数配置

打开`sensor/sensor.ino`

微雪-GP2Y1010AU0F 灰尘传感器，由于无精准的pm2.5测量仪作为依据，因此参考厂家的经验值。

```Plain
#define        COV_RATIO            0.172     //转换系数
#define        NO_DUST_VOLTAGE      400       //无尘环境下的基准电压（单位：mV）
#define        SYS_VOLTAGE          5020.0    //供电电压（单位：mV）
```

微雪-TVOC气体传感器模组数据已由厂家校准，故无需调参

## 4. 固件烧录

1. **环境准备**：确保电脑已安装 [Arduino IDE](https://www.google.com/url?sa=E&q=https%3A%2F%2Fwww.arduino.cc%2Fen%2Fsoftware)。
2. **打开工程**：进入 sensor文件夹，双击打开 sensor.ino。
3. **开发板设置**：
   
   使用下载线连接电脑与Arduino，在工具-> 开发板 (Board) 中选择 Arduino Mega or Mega 2560。
4. **编译并上传**：依次点击左上角的验证、上传按钮，等待下方显示“上传成功”。

# ROS2端

## 1. 环境配置

在开始之前，确保你已经安装了：
- Ubuntu 22.04 & ROS 2 Humble
- Python 3.10+
- `pyserial` 库

```Plain
# 安装串口库
pip3 install pyserial
# 打开串口
sudo chmod 666 /dev/ttyACM0
```

## 2. 安装教程

```Plain
# 克隆仓库
git clone https://github.com/BreadQ3166/ros2-arduino-air.git
# 进入工作空间
cd air_ws
# 编译
colcon build --packages-select aq_msgs
colcon build --packages-select aq_driver
# 刷新
source install/setup.bash
# 运行传感器节点
ros2 run aq_driver serial_reader
```

参数配置，打开/air_ws/src/aq_driver/aq_driver/serial_reader.py

| 参数名 | 类型   | 默认值       | 说明            |
| ------ | ------ | ------------ | --------------- |
| port   | string | /dev/ttyACM0 | Arduino串口地址 |

## 3. 消息格式

- **发布话题**: /air_quality_data
- **消息类型**: aq_msgs/msg/AirQuality

## 4. 查看数据

```Plain
# 查看话题信息
ros2 topic echo /air_quality_data
ros2 run plotjuggler plotjuggler
```

![air](https://github.com/BreadQ3166/ros2-arduino-air/blob/main/air.png?raw=true)

