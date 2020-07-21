# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

from collections import deque
import socket
import sys


import numpy as np
import rclpy
from rclpy.node import Node

from tactile_msgs.msg import TactileSignal
from tactile_msgs.srv import ChangeState


class TactileSignalPublisher(Node):

    def __init__(self):
        super().__init__('tactile_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip', "0.0.0.0"),
                ('port', 10240)
            ]
        )
        (ip, port) = self.get_parameters(['ip', 'port'])

        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_.bind((str(ip.value), int(port.value)))

        self.state_ = 0
        self.calibration_queue_ = deque(maxlen=30) # Number of samples used for calibration
        self.reference_values_ = np.zeros(16)

        self.publisher_ = self.create_publisher(
            TactileSignal, '/tactile_signals', 10)
        self.srv_ = self.create_service(
            ChangeState, '/tactile_publisher/change_state', self.change_state_callback)

        # Publisher rate 0.03s
        self.timer_ = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        data, addr = self.sock_.recvfrom(1024)
        values = [int.from_bytes(data[i:i+2], 'big', signed=False) for i in range(0, len(data), 2)]

        if self.state_ == 1 and len(self.calibration_queue_) == self.calibration_queue_.maxlen:
            self.reference_values_ = np.average(self.calibration_queue_, axis=0)

            msg = TactileSignal()
            msg.header.frame_id = 'world'
            msg.header.stamp = self.get_clock().now().to_msg()
            try:
                msg.addr = addr[0] + ":" + str(addr[1])
                msg.data = np.array(values, dtype=np.int32) - self.reference_values_.astype(np.int32)
                self.publisher_.publish(msg)
            except:
                self.get_logger().error("Error tactile data: incorrect array length or overflow.")
        elif self.state_ == 0:
            if len(values) == 16:
                self.calibration_queue_.append(values)
        else:
            self.get_logger().error("Unknown state or uncalibrated sensor!")

    def change_state_callback(self, request, response):
        if request.transition != self.state_:
            try:
                self.state_ = request.transition

                response.success = True
                response.info = "OK"
            except:
                response.success = False
                response.info = "Undefined state!"
        else:
            response.success = True
            response.info = "No transition needed!"

        return response


def main(args=None):
    rclpy.init(args=args)
    pub = TactileSignalPublisher()
    rclpy.spin(pub)
    ub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
