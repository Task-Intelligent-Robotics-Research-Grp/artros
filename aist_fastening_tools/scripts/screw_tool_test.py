#!/usr/bin/env python3

import rclpy, sys, threading
import numpy as np
from rclpy.node                  import Node
from aist_fastening_tools.client import ScrewTool


class ScrewToolTest(Node):
    def __init__(self, name):
        super().__init__(name)

        tool_name = self.declare_parameter('tool_name',
                                           'screw_tool_m3_fastening').value
        self._screw_tool = ScrewTool(self, tool_name)
        self.get_logger().info('started')

        cli_thread = threading.Thread(target=self.interactive)
        cli_thread.daemon = True
        cli_thread.start()

    def interactive(self):
        while rclpy.ok():
            print('====')
            print('  q: quit this program')
            print('  t: tighten the screw')
            print('  l: loosen the screw')
            print('  w: wait for tightening/loosening completed')
            print('  c: cancel tightening/loosening')
            print('  s: set tool speed')

            key = input('[speed: %d]> ' % self._screw_tool.parameters['speed'])

            if key == 'q':
                break
            elif key == 't':
                self._screw_tool.tighten()
            elif key == 'l':
                self._screw_tool.loosen()
            elif key == 'w':
                self._screw_tool.wait()
            elif key == 'c':
                self._screw_tool.cancel()
            elif key == 's':
                speed = np.clip(float(input('  speed? ')), 0.0, 1.0)
                self._screw_tool.parameters = {'speed': speed}
            else:
                print('Unknown command[%s]' % key)
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    test = ScrewToolTest('screw_tool_test')
    rclpy.spin(test)
