# can2wifi2ros
A ROS2 package for CAN2Wifi Arduino module

## Features
* Receives raw data of CPM-finger via UDP in bytes and publishes converted integer array to ROS2 network
* Automatic calibration (leave the sensor untouched during initial calibration stage)
* Node state manager as a service; available states are listed in `tactile_signal_publisher.py`

## Dependencies
* [Arduino CAN](https://github.com/sandeepmistry/arduino-CAN)
* [WiFiNINA](https://www.arduino.cc/en/Reference/WiFiNINA)

## Hardware
* [Arduino MKR Wifi 1010](https://store.arduino.cc/arduino-mkr-wifi-1010)
* [Arduino MKR CAN Shield](https://store.arduino.cc/arduino-mkr-can-shield)

## Notes
* Default client ip is `192.168.0.100:10240`, communicated via `UDP`
* [CPM-Finger](https://www.cyskin.com/cpm-finger-the-finger-for-textile-manipulation/) decoder is embedded into the Arduino codes; correct taxel order is also presented
* Set client ip to `0.0.0.0` for the container with host network