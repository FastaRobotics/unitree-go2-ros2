#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class MoveGazeboObject(Node):
    def __init__(self):
        super().__init__('move_gazebo_object')
        self.client = self.create_client(SetModelState, '/gazebo/set_model_state')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_model_state service...')

        self.move_object()

    def move_object(self):
        request = SetModelState.Request()
        request.model_state = ModelState()
        request.model_state.model_name = 'unit_sphere'  # 🔹 Change to your object nam

        # ✅ Set velocity (Make object move)
        request.model_state.twist.linear.x = 0.5  # Move forward
        request.model_state.twist.linear.y = 0.0
        request.model_state.twist.linear.z = 0.0

        request.model_state.reference_frame = "world"

        self.client.call_async(request)
        self.get_logger().info("✅ Object moved in Gazebo!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveGazeboObject()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
