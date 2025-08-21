#!/usr/bin/env python3

import rclpy, sys, threading
from rclpy.node              import Node
from rclpy.executors         import ExternalShutdownException
from aist_camera_multiplexer import RealsenseMultiplexerClient


class RealsenseSelector(Node):
    def __init__(self, name, activated_camera_name):
        super().__init__(name)

        self._multiplexer = RealsenseMultiplexerClient(self,
                                                       'camera_multiplexer')

        cli_thread = threading.Thread(target=self.interactive,
                                      args=(activated_camera_name,))
        cli_thread.daemon = True
        cli_thread.start()

    def interactive(self, activated_camera_name):
        while rclpy.ok():
            if activated_camera_name is not None:
                print('activating %s... ' % activated_camera_name, end='')
                if self._multiplexer.activate_camera(activated_camera_name):
                    print('succeeded')
                else:
                    print('failed')
                break

            key = input('[active: %s]> '
                        % self._multiplexer.active_camera_name)

            if key == 'q':
                break
            elif not self._multiplexer.activate_camera(key):
                print('Unknown camera[%s]' % key)
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()

    activated_camera_name = sys.argv[1] if len(sys.argv) > 1 else None

    try:
        selector = RealsenseSelector('realsense_selector',
                                     activated_camera_name)
        rclpy.spin(selector)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
