| 支持的设备 | ESP32-C2 |
| ---------- | -------- |

# _ESPNOW 开关 项目_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

这个是基于ESP-NOW协议的纽扣电池供电开关方案，支持一个到三个按键，实现的功能有：绑定设备，解绑设备，单击和长按。

# 硬件需求
* 一块ESP32-C2开发板作为终端设备
* 一块ESP32-C2按钮开关
* 一个USB Program
  
## 如何使用本程序

### 硬件及IDF文件修改
使用Firmware文件夹下的固件替换component/esp_phy lib/esp32c2里面的库
menuconfig 中 选择 store phy calibration data in NVS
在 phy_init.c 中修改代码
把    esp_phy_calibration_mode_t calibration_mode = PHY_RF_CAL_PARTIAL; 
换成   esp_phy_calibration_mode_t calibration_mode = PHY_RF_CAL_NONE; 

### menconfig配置
在 menconfig - Espnow Lite Options - Target Fuction 中配置一个设备为开关，
配置发射器的按键GPIO引脚
配置发射器的LED与蜂鸣器GPIO引脚

在 menconfig - Espnow Lite Options - Target Fuction 中配置 一个设备为灯泡
配置接收器的LED引脚


状态触发的定义
单击 按下按钮 0.07s < t < 1s
双击 0.5s 内连续按压2次
长按 按下事件 > 1s 
绑定设备  长按按钮>7s
解绑所有设备 长按按钮>13s


## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
