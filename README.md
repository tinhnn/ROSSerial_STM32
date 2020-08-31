# ROSSerial_STM32
ROS serial for STM32 mcu

Deploy to board "STM32F746 Nucleo"

## How to run
1. Open and build source with STM32CubeIDE

2. On your computer start roscore and rosserial_python:
- Terminal 1
```
$ roscore
```
- Terminal 2
```
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```
- Terminal 3
```
$ rostopic list
/chatter
/diagnostics
/rosout
/rosout_agg
$ rostopic echo /chatter
data: "Welcome to ROS Serial for STM32f7 series"
```
## Reference
* http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
* http://wiki.ros.org/rosserial_client/Tutorials/Adding%20Support%20for%20New%20Hardware
* https://github.com/yoneken/rosserial_stm32
* https://github.com/fdila/rosserial_stm32f7
