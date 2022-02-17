# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import serial
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from tactile_interfaces.msg import TactileSignal
from tactile_interfaces.srv import ChangeState


STATE_LIST = {0: "calibration", 1: "recording",
              50: "standby", 99: "termination"}


class TactileSignalPublisher(Node):
    """
    A node class for tactile signal publisher.
    The node receives tactile signals in bytes via UDP and converts the data to array and publish to ROS2 network.
    Runtime node state switch is implemented.
    """

    def __init__(self):
        super().__init__("tactile_publisher")

        # Parameters are set via ROS2 parameter server.

        self.declare_parameters(
            namespace="",
            parameters=[
                ("port", "/dev/ttyACM0"),
                ("buffer_size", 96),
            ],
        )

        self.port = str(self.get_parameter("port").value)
        buffer_size = int(self.get_parameter("buffer_size").value)

        # Open serial port
        self.serial = serial.Serial(self.port, 1152000, timeout=0, rtscts=1)
        self.serial.open()
        self.node_state = 0

        # Data buffer for calibration
        self.buffer = deque(maxlen=buffer_size)
        self.reference_value = np.zeros(16)

        # Create the publisher and service host
        self.publisher = self.create_publisher(
            TactileSignal, "tactile_signals", 10)
        self.service = self.create_service(
            ChangeState,
            "tactile_publisher/change_state",
            self.change_node_state_callback,
        )

        # Publisher rate 0.03s
        self.timer = self.create_timer(0.03, self.timer_callback)

        # self.get_logger().info("Node started in state: calibration")

    def timer_callback(self):
        s = self.serial.read(32)
        vals = np.array(
            [int.from_bytes(s[i: i + 2], "big")
             for i in range(0, len(s), 2)],
            dtype=np.int32,
        )

        try:
            if self.node_state == 0:  # calibration state
                self.buffer.append(vals)
                # Once the buffer is full, compute the average values as reference
                if len(self.buffer) == self.buffer.maxlen:
                    self.reference_value = np.mean(
                        self.buffer, axis=0, dtype=np.int32)
                    self.node_state = 1  # Change to recording state
                    self.get_logger().info("Calibration finished!")
            elif self.node_state == 1:  # recording state
                if len(self.buffer) < self.buffer.maxlen:
                    self.get_logger().warn("Calibration unfinished!")
                vals -= self.reference_value

                # Prepare TactileSignal message
                msg = TactileSignal()
                msg.addr = self.port
                msg.header.frame_id = "world"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.data = vals
                msg.mean = np.mean(vals)
                self.publisher.publish(msg)
            elif self.node_state == 50:  # standby state
                pass
            elif self.node_state == 99:  # termination state
                self.get_logger().warn("Tactile publisher terminated.")
                self.serial.close()
                self.destroy_node()
        except Exception as error:
            self.get_logger().error(str(error))

    def change_node_state_callback(self, request, response):
        if request.transition != self.node_state and request.transition in STATE_LIST.keys():
            self.node_state = request.transition
            response.success = True
            response.info = "OK"
            self.get_logger().info(
                "Changed to state: {}".format(
                    STATE_LIST[self.node_state])
            )
            if self.node_state == 0:
                self.buffer.clear()
        else:
            raise Exception("Node state cannot be changed!")

        return response


def main(args=None):
    rclpy.init(args=args)
    pub = TactileSignalPublisher()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
