# 项目简介

本项目实现了一个空气质量实时监测桥接器，通过 Arduino 驱动灰尘与气体传感器进行数据采集与打包，利用串口通讯将信息上传至 ROS 2终端，并以 1Hz 的固定频率发布 `/air_quality_data` 话题。

## 技术规格

- **发布频率**：ROS2节点频率 1Hz
- **通讯协议**：
  - 传感器层：UART (115200 bps) + ADC
  - Arduino-ROS2传输层：自定义串口协议 `$START,...,$END`

# Arduino

1. ## 硬件清单

| **组件**     | **型号**             | **说明**                                  |
| ------------ | -------------------- | ----------------------------------------- |
| 主控板       | Arduino Mega 2560    | 核心控制器                                |
| 灰尘传感器   | 微雪 GP2Y1010AU0F    | 红外粉尘检测传感器                        |
| 气体传感器   | TVOC气体传感器模组   | 支持 UART 串口输出                        |
| 电平转化模块 | 5V转3.3V四路电平转化 | Arduino(5V)与气体传感器(3.3V)信号电平转化 |

1. ## 接线定义

### 微雪-GP2Y1010AU0F 灰尘传感器

IO电平为5V，可与Arduino直接连接

| Arduino   | 灰尘传感器 | 管脚描述          |
| --------- | ---------- | ----------------- |
| 5V        | VCC        | 电源正(2.5V-5.5V) |
| GND       | GND        | 电源地            |
| Analog A0 | AOUT       | 电压模拟量输出    |
| Digital 7 | ILED       | 传感器内部LED驱动 |

### 微雪-TVOC气体传感器模组

IO电平为3.3V，四根信号线通过电平转换模块进行5V-3.3V电平转化

| Arduino          | 气体传感器 | 管脚描述                             |
| ---------------- | ---------- | ------------------------------------ |
| 5V               | VCC        | 5V 电源正                            |
| GND              | GND        | 电源地                               |
| Digital 19 (RX1) | TXD        | UART 输出                            |
| Digital 18 (TX1) | RXD        | UART 输入                            |
| Digital 9        | RST        | 复位引脚，低电平有效                 |
| Digital 8        | ALM        | 报警引脚，TVOC超过2ppm，引脚自动拉低 |

1. ## 参数配置

打开`sensor/sensor.ino`

微雪-GP2Y1010AU0F 灰尘传感器，由于无精准的pm2.5测量仪作为依据，因此参考厂家的经验值。

```Plain
#define        COV_RATIO            0.172     //转换系数
#define        NO_DUST_VOLTAGE      400       //无尘环境下的基准电压（单位：mV）
#define        SYS_VOLTAGE          5020.0    //供电电压（单位：mV）
```

微雪-TVOC气体传感器模组数据已由厂家校准，故无需调参

# ROS2端

## 环境配置

```Plain
# 安装串口库
pip3 install pyserial
# 打开串口
sudo chmod 666 /dev/ttyACM0
```

## 启动串口桥接节点

```Plain
# 进入工作空间
cd air_ws/src
# 构建自定义消息格式
colcon build --packages-select aq_msgs
# 构建串口转发
colcon build --packages-select aq_driver
# 刷新环境
source install/setup.bash
# 运行传感器节点
ros2 run aq_driver serial_reader
```

## 查看数据

```Plain
# 查看话题信息
ros2 topic echo /air_quality_data
ros2 run plotjuggler plotjuggler
```

![img](https://jcn67m1rifze.feishu.cn/space/api/box/stream/download/asynccode/?code=MjExYTZiOTk2OWY0YTVhMDczOTE4ZGI2NDdjMGIxODlfbFRCWXlyQmpCbVMwU2FZWDdQVXR2SDNBNDI4WjlGeGhfVG9rZW46WWJsb2JydHpVb0ttZkV4MlJsOWM1dzdnbmJoXzE3NzA2MjI2MTc6MTc3MDYyNjIxN19WNA)