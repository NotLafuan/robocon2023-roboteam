# ROBOCON 2023
![robocon](assets/000.png)
## Installation
### 1. Install Visual Studio Code:
```shell
snap install code --classic
```
### 2. Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [PlatformIO](https://platformio.org/install/ide?install=vscode).
### 3. Install udev rules [here](https://docs.platformio.org/en/stable/core/installation/udev-rules.html) for PlatformIO.
### 4. Run these command for installing ROS packages:
```shell
sudo apt install ros-noetic-rosserial-arduino
sudo apt install ros-noetic-rosserial
```
### 5. Install these extensions from the VSCode Marketplace:
```
ROS
PlatformIO IDE
```

## Setup ROS
### 1. Clone the repo
```shell
git clone https://github.com/NotLafuan/robocon2023-roboteam.git
cd robocon2023-roboteam
```
### 2. Build the workspace
```shell
catkin_make
```
### 3. Source the workspace in `.bashrc`.
```shell
echo "source ~/robocon2023-roboteam/devel/setup.bash" >> ~/.bashrc
```
Your directory may be different depending on where the repo is cloned.
### 4. Install Python libraries.
```shell
pip install -r requirements.txt
```
## Setup PlatformIO
### 1. Add a project to PlatformIO.
1. Open `PIO Home` by clicking the `home` icon on the bottom bar.
2. Go to `Project` tab.
3. Click `Add Existing` and choose the project directory.

Directories:
```
robocon2023-roboteam/src/elephant_robot/elephant_stm32
robocon2023-roboteam/src/elephant_robot/elephant_esp32
robocon2023-roboteam/src/rabbit_robot/rabbit_stm32
```
### 2. Build the projects.
1. Go to any project in `PIO Home` and click `Open`.
2. Build the project by clicking the `check mark (✓)` icon on the bottom bar. This will download the necessary packages and libraries.
3. Repeat this for all the projects.
### 3. Fix the broken library.
For `elephant_robot/elephant_stm32` and `rabbit_robot/rabbit_stm32`:
1. Go to `.pio/libdeps/blackpill_f401cc/Rosserial Arduino Library/src`.
2. Open `ArduinoHardware.h`.
3. Change line 54 to:
```c
#elif defined(USE_USBCON)
  // Arduino Leonardo USB Serial Port
  #define SERIAL_CLASS USBSerial
```

For `elephant_robot/elephant_esp32`:
1. Go to `.pio/libdeps/esp32doit-devkit-v1/Rosserial Arduino Library/src`.
2. Open `ros.h`.
3. Change line 40 to:
```c
#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoHardware.h"
```
## Usage
### Uploading code
1. Select a project from `PIO Home`.
2. Upload using the `arrow` icon on the bottom bar.
### Running ROS
1. Make sure both ROS and workspace have been sourced in `.bashrc`.
2. Open a terminal and run:
```shell
roslaunch elephant_robot elephant.launch
```
## Notes
- `elephant_robot/elephant_stm32` is for the elephant's movement and positioning.
- `elephant_robot/elephant_esp32` is for the elephant's lifter, feeder and launcher.
- `rabbit_robot/rabbit_stm32` is for the rabbit's lifter, feeder and launcher.
- Rabbit's movement script is not in this repo (yet?).
- Connect the controller before launching the elephant script.
- The angle sensor on the elephant robot resets every time the stm32 resets.