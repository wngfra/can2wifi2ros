# can2wifi2ros
A ROS2 package for CAN2Wifi Arduino module

## Dependencies
* (Arduino CAN)[https://github.com/sandeepmistry/arduino-CAN]
* WiFiNINA

## Hardware
* (Arduino MKR Wifi 1010)[https://store.arduino.cc/arduino-mkr-wifi-1010]
* (Arduino MKR CAN Shield)[https://store.arduino.cc/arduino-mkr-can-shield]

## Notes
* Default client ip is `192.168.0.100:10240`, communicated via `UDP`
* (CPM-Finger)[https://www.cyskin.com/cpm-finger-the-finger-for-textile-manipulation/] decoder is embedded into the Arduino codes; correct taxel order is also presented

## TODO
* `test` to be implemented (maybe never...)