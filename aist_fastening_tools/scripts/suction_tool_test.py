#!/usr/bin/env python3

import rclpy, sys, threading
from rclpy.executors             import MultiThreadedExecutor
from rclpy.node                  import Node
from aist_fastening_tools.client import SuctionTool


class SuctionToolTest(Node):
    def __init__(self, name):
        super().__init__(name)

        device_name = self.declare_parameter('device_name',
                                             'suction_tool').value
        self._suction_tool = SuctionTool(self, device_name)
        self.get_logger().info('started')

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        while rclpy.ok():
            print('====')
            print('  q: quit this program')
            print('  g: grasp')
            print('  r: release')
            print('  m: set min period')
            print('  w: wait for result for two seconds')
            print('  c: cancel')

            key = input('[suck_min_period=%f]> '
                        % self._suction_tool.parameters['suck_min_period'])

            if key == 'q':
                break
            elif key == 'g':
                self._suction_tool.grasp(timeout_sec=0.0)
            elif key == 'r':
                self._suction_tool.release()
            elif key == 'm':
                suck_min_period = float(input('  suck_min_period? '))
                self._suction_tool.parameters = {'suck_min_period':
                                                 suck_min_period}
            elif key == 'w':
                status, result = self._gripper.wait(timeout_sec=2.0)
                print(result)
            elif key == 'c':
                self._suction_tool.cancel_goal()
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
