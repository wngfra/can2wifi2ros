# can2wifi2ros
Arduino CAN-to-WiFi via ROS2 project

## Dependencies
* (Arduino CAN)[https://github.com/sandeepmistry/arduino-CAN]
* WiFiNINA

## Hardware
* (Arduino MKR Wifi 1010)[https://store.arduino.cc/arduino-mkr-wifi-1010]
* (Arduino MKR CAN Shield)[https://store.arduino.cc/arduino-mkr-can-shield]

## Notes
* Default client ip is `192.168.0.100:10240`, communicated via `UDP`
* In `python` folder there is a testing python script to print out the messages heard at the port
* Copy `can_wifi` and  `tactile_msg` to your ros2 workspace `src` directory and compile