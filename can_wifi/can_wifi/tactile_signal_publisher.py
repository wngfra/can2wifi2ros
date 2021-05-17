# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import socket

import numpy as np
import rclpy
from rclpy.node import Node

from tactile_interfaces.msg import TactileSignal
from tactile_interfaces.srv import ChangeState


STATE_LIST = {0: "calibration", 1: "recording", 50: "standby", 99: "termination"}


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
                ("ip", "0.0.0.0"),
                ("port", 10240),
                ("buffer_size", 96),
                ("mode", "processed"),
            ],
        )
        ip = str(self.get_parameter("ip").value)
        port = int(self.get_parameter("port").value)
        buffer_size = int(self.get_parameter("buffer_size").value)
        mode = str(self.get_parameter("mode").value)

        # Open UDP socket and bind the port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.mode = mode
        self.node_state = 0

        # Data buffer for calibration
        self.buffer = np.zeros((buffer_size, 16))
        self.reference_value = np.zeros(16)
        self.buffer_count = 0

        # Create the publisher and service host
        self.publisher = self.create_publisher(TactileSignal, "tactile_signals", 10)
        self.service = self.create_service(
            ChangeState,
            "tactile_publisher/change_state",
            self.change_node_state_callback,
        )

        # Publisher rate 0.03s
        self.timer = self.create_timer(0.03, self.timer_callback)

        self.get_logger().info("Node started in state: calibration")

    def process(self, values):
        processed_ = values - self.reference_value
        # Fix a faulty taxel
        processed_[8] = 0
        processed_[8] = np.mean(processed_)
        return processed_

    def timer_callback(self):
        data, addr = self.sock.recvfrom(256)
        values = np.array(
            [
                int.from_bytes(data[i : i + 2], "big")
                for i in range(0, len(data), 2)
            ],
            dtype=np.int32,
        )
        # self.get_logger().info("{}-byte raw data {}".format(len(data), values))

        try:
            if self.node_state == 0:  # calibration state
                self.buffer[:-1] = self.buffer[1:]
                self.buffer[-1] = values
                self.buffer_count += 1

                # Once the buffer is filled, compute the average values as reference
                if self.buffer_count == self.buffer.shape[0]:
                    self.reference_value = np.mean(self.buffer, axis=0, dtype=np.int32)
                    self.get_logger().info("Calibration finished!")
            elif self.node_state == 1:  # recording state
                if self.buffer_count < self.buffer.shape[0]:
                    self.get_logger().warn("Calibration unfinished!")

                # Prepare TactileSignal message
                msg = TactileSignal()
                msg.addr = addr[0] + ":" + str(addr[1])
                msg.header.frame_id = "world"
                msg.header.stamp = self.get_clock().now().to_msg()

                if self.mode == "processed":
                    values = self.process(values)

                msg.data = values
                msg.mean = np.mean(values)
                self.publisher.publish(msg)
            elif self.node_state == 50:  # standby state
                pass
            elif self.node_state == 99:  # termination state
                self.get_logger().warn("Tactile publisher terminated.")
                self.destroy_node()
        except Exception as error:
            self.get_logger().error(str(error))

    def change_node_state_callback(self, request, response):
        if request.transition != self.node_state:
            try:
                if request.transition in STATE_LIST.keys():
                    self.node_state = request.transition

                    response.success = True
                    response.info = "OK"

                    self.get_logger().info(
                        "Changed to state: {}".format(STATE_LIST[self.node_state])
                    )

                    if self.node_state == 0:
                        self.buffer_count = 0

                else:
                    raise Exception("Undefined state")
            except Exception as error:
                response.success = False
                response.info = str(error)

                self.get_logger().error(
                    "Wrong transition! Reverting to state: calibration"
                )
                self.node_state = 0

        return response


def main(args=None):
    rclpy.init(args=args)
    pub = TactileSignalPublisher()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
