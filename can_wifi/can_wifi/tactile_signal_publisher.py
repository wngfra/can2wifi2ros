import socket
import sys

import rclpy
from rclpy.node import Node

from tactile_msg.msg import TactileSignal


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

        self.sock = sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))

        self.publisher_ = self.create_publisher(TactileSignal, 'topic', 10)
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TactileSignal()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now()
        data, addr = sock.recvfrom(1024)
        msg.addr = addr[0] + ":" + addr[1]
        msg.data = values = [int.from_bytes(
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
