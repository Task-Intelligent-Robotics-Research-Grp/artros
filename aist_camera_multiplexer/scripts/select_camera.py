#!/usr/bin/env python3

import rclpy, sys, threading
from rclpy.node              import Node
from rclpy.executors         import ExternalShutdownException
from aist_camera_multiplexer import CameraMultiplexerClient


class CameraSelector(Node):
    def __init__(self, name):
        super().__init__(name)

        self._multiplexer = CameraMultiplexerClient(self, 'camera_multiplexer')

        cli_thread = threading.Thread(target=self.interactive)
        cli_thread.daemon = True
        cli_thread.start()

    def interactive(self):
        while rclpy.ok():
            key = input('[active: %s]> '
                        % self._multiplexer.active_camera_name)

            if key == 'q':
                break
            elif not self._multiplexer.activate_camera(key):
                print('Unknown camera[%s]' % key)
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    try:
        selector = CameraSelector('camera_selector')
        rclpy.spin(selector)
    except (KeyboradInterrupt, ExternalShutdownException):
        pass
