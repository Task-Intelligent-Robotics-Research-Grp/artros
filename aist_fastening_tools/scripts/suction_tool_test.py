#!/usr/bin/env python3

import rclpy, sys, threading
from rclpy.executors      import MultiThreadedExecutor
from rclpy.node           import Node
from aist_fastening_tools import SuctionTool


class SuctionToolTest(Node):
    def __init__(self, name):
        super().__init__(name)

        tool_name = self.declare_parameter('tool_name', 'suction_tool').value
        self._suction_tool = SuctionTool(self, tool_name)
        self.get_logger().info('started')

        cli_thread = threading.Thread(target=self.interactive)
        cli_thread.daemon = True
        cli_thread.start()

    def interactive(self):
        while rclpy.ok():
            print('====')
            print('  q: quit this program')
            print('  g: grasp')
            print('  r: release')
            print('  m: set min period')
            print('  w: wait for ten seconds')
            print('  c: cancel')

            key = input('[suck_min_period=%f]> '
                        % self._suction_tool.parameters['suck_min_period'])

            if key == 'q':
                break
            elif key == 'g':
                self._suction_tool.grasp()
            elif key == 'r':
                self._suction_tool.release()
            elif key == 'm':
                suck_min_period = float(input('  suck_min_period? '))
                self._suction_tool.parameters = {'suck_min_period':
                                                 suck_min_period}
            elif key == 'c':
                self._suction_tool.cancel()
            else:
                print('Unknown command[%s]' % key)
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        rclpy.init(args=sys.argv)

        test = SuctionToolTest('suction_tool_test')
        executor = MultiThreadedExecutor()
        executor.add_node(test)
        executor.spin()
    except Exception as e:
        print('*** Terminate the node due to exception: %s' % e)
