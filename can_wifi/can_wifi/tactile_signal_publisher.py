import socket
import sys

import rclpy
from rclpy.node import Node

from tactile_msgs.msg import TactileSignal


class TactileSignalPublisher(Node):

    def __init__(self):
        super().__init__('tactile_signal_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip', "192.168.0.100"),
                ('port', 10240)
            ]
        )
        (ip, port) = self.get_parameters(['ip', 'port'])
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((str(ip.value), int(port.value)))

        self.publisher_ = self.create_publisher(TactileSignal, '/tactile_signals', 10)
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TactileSignal()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        data, addr = self.sock.recvfrom(1024)
        msg.addr = addr[0] + ":" + str(addr[1])
        msg.data = [int.from_bytes(
            data[i:i+2], 'big', signed=False) for i in range(0, len(data), 2)]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    pub = TactileSignalPublisher()

    rclpy.spin(pub)

    # pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
