# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

from collections import deque
import socket
import sys

import numpy as np
import rclpy
from rclpy.node import Node

from tactile_interfaces.msg import TactileSignal
from tactile_interfaces.srv import ChangeState


THRESHOLD_MU = 1
THRESHOLD_SIGMA = 1


STATE_LIST = {
    0:  'calibration',
    1:  'recording',
    50: 'standby',
    99: 'termination'
}


class TactileSignalPublisher(Node):
    """
    A node class for tactile signal publisher.
    The node receives tactile signals in bytes via UDP and converts the data to array and publish to ROS2 network.
    Runtime node state switch is implemented.
    """

    def __init__(self):
        super().__init__('tactile_publisher')

        # Parameters are set via ROS2 parameter server.

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip',               "0.0.0.0"),
                ('port',             10240),
                ('calibration_size', 30),
                ('mode',             None)
            ]
        )
        ip = str(self.get_parameter('ip').value)
        port = int(self.get_parameter('port').value)
        calibration_size = int(self.get_parameter('calibration_size').value)
        mode = str(self.get_parameter('mode').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))

        self.node_state = 0

        # Queue of data for calibration
        self.calibration_queue = deque(maxlen=calibration_size)
        self.reference_value = np.zeros(16)

        self.publisher = self.create_publisher(
            TactileSignal, 'tactile_signals', 10)
        self.service = self.create_service(
            ChangeState, 'tactile_publisher/change_state', self.change_node_state_callback)

        # Publisher rate 0.03s
        callback_func = self.sim_callback if mode == 'sim' else self.timer_callback
        self.timer = self.create_timer(0.03, callback_func)

        self.get_logger().info('Node started in state: calibration')

    def sim_callback(self):
        msg = TactileSignal()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()

    def timer_callback(self):
        data, addr = self.sock.recvfrom(1024)
        values = [int.from_bytes(data[i:i+2], 'big', signed=False)
                  for i in range(0, len(data), 2)]

        if len(values) == 16:
            if self.node_state == 0:  # calibration state
                self.calibration_queue.append(values)
            elif self.node_state == 1:  # recording state
                if len(self.calibration_queue) == self.calibration_queue.maxlen:
                    self.reference_value = np.average(
                        self.calibration_queue, axis=0)

                    msg = TactileSignal()
                    msg.header.frame_id = 'world'
                    msg.header.stamp = self.get_clock().now().to_msg()
                    try:
                        data = np.array(values, dtype=np.int32) - \
                            self.reference_value.astype(np.int32)
                        data[data <= 3] = 0.0
                        if np.mean(data) <= THRESHOLD_MU and np.var(data) >= THRESHOLD_SIGMA**2:
                            data.fill(0)

                        msg.addr = addr[0] + ":" + str(addr[1])
                        msg.data = data
                        self.publisher.publish(msg)
                    except Exception as error:
                        self.get_logger().error(str(error))
                else:
                    self.get_logger().error("Uncalibrated sensor!")
                    raise Exception("Calibration queue is not filled.")
            elif self.node_state == 50:  # standby state
                pass
            elif self.node_state == 99:  # termination state
                self.get_logger().warn("Tactile publisher terminated.")
                self.destroy_node()

    def change_node_state_callback(self, request, response):
        if request.transition != self.node_state:
            try:
                if request.transition in STATE_LIST.keys():
                    self.node_state = request.transition

                    response.success = True
                    response.info = "OK"

                    self.get_logger().info(
                        "Changed to state: {}".format(STATE_LIST[self.node_state]))
                else:
                    raise Exception("Undefined state")
            except Exception as error:
                response.success = False
                response.info = str(error)

                self.get_logger().error("Wrong transition! Reverting to state: calibration")
                self.node_state = 0

        return response


def main(args=None):
    rclpy.init(args=args)
    pub = TactileSignalPublisher()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
