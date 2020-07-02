# can2wifi_ros2
Arduino CAN-to-WiFi via ROS2 project

## Dependencies
* (Arduino CAN)[https://github.com/sandeepmistry/arduino-CAN]
* (ros2arduino)[https://github.com/ROBOTIS-GIT/ros2arduino]
* WiFiNINA

## Hardware
* (Arduino MKR Wifi 1010)[https://store.arduino.cc/arduino-mkr-wifi-1010]
* (Arduino MKR CAN Shield)[https://store.arduino.cc/arduino-mkr-can-shield]

## Notes
* Default client ip is `192.168.0.100:10240`, communicated via `UDP`
* In `python` folder there is a testing python script to print out the messages heard at the port

## TODO
* Publish messags to ROS2 via DDS, currently only sending message to a specific ip via UDP