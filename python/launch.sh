podman run -it --name ros2 --privileged -p 192.168.0.100:10240:10240/udp -v /var/home/alex/Arduino/can2wifi_ros2/python:/mnt/python -w /mnt/python osrf/ros:foxy-desktop python3 UdpRecv.py 0.0.0.0 10240

