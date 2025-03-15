#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class NormalImageProcessor(Node):
    def __init__(self):
        super().__init__("black_object_detector")
        self.bridge = CvBridge()
        self.normal_image = None
        self.coord_goal = None
        self.image_width = 800  # ✅ Change this to match your camera resolution

        # ✅ Corrected Image topic
        normal_image_topic = "/camera1/image_raw"

        # ✅ Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            normal_image_topic,
            self.normal_image_callback,
            10  # QoS depth
        )

        # ✅ Publish detected object coordinates
        self.goal_distance_publisher = self.create_publisher(Float32MultiArray, "coordinate_of_goal", 10)

        # ✅ Publish velocity commands for heading correction
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # ✅ PID Controller parameters
        self.kp = 0.005  # Proportional gain
        self.ki = 0.0001  # Integral gain
        self.kd = 0.001  # Derivative gain

        self.prev_error = 0.0
        self.integral = 0.0

        self.msg = Float32MultiArray()
        self.get_logger().info(f"✅ Subscribed to {normal_image_topic}")

    def normal_image_callback(self, data):
        try:
            # ✅ Convert ROS Image message to OpenCV format (BGR)
            self.normal_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # ✅ Convert image to HSV color space
            hsv_image = cv2.cvtColor(self.normal_image, cv2.COLOR_BGR2HSV)

            # ✅ Define color range for black object detection
            black_lower = np.array([0, 0, 0], np.uint8)
            black_upper = np.array([50, 50, 50], np.uint8)

            # ✅ Create mask for black objects
            black_mask = cv2.inRange(hsv_image, black_lower, black_upper)

            # ✅ Morphological transformations to remove noise
            kernel = np.ones((5, 5), np.uint8)
            black_mask = cv2.dilate(black_mask, kernel)

            # ✅ Find contours of black objects
            contours_black, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # ✅ Initialize detection flag
            goal_detected = False
            cx, cy = -1, -1  # Default values if no object is found

            # ✅ Process black contours (goal objects)
            for contour in contours_black:
                area = cv2.contourArea(contour)
                if area > 500:  # ✅ Ignore small noise
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])  # Center X
                        cy = int(M["m01"] / M["m00"])  # Center Y
                        self.coord_goal = [cx, cy]
                        goal_detected = True

                        cv2.drawContours(self.normal_image, [contour], 0, (255, 0, 0), 4)
                        cv2.putText(self.normal_image, "Black Object", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

                        # ✅ Publish detected center coordinates
                        self.msg.data = [float(cx), float(cy)]
                        self.goal_distance_publisher.publish(self.msg)

            if not goal_detected:
                self.coord_goal = [-1.0, -1.0]  # ✅ Ensure values are floats
                self.msg.data = self.coord_goal
                self.goal_distance_publisher.publish(self.msg)

            # ✅ Run PID controller for heading correction
            self.correct_heading(cx)

            # ✅ Show processed image
            self.visualize_normal_image()

        except Exception as e:
            self.get_logger().error(f"🚨 Error processing image: {e}")

    def correct_heading(self, cx):
        """
        PID Controller to correct robot heading based on detected object's center.
        """
        if cx == -1:  # No object detected
            return

        # ✅ Compute error (center of image - detected object center)
        image_center_x = self.image_width // 2
        error = image_center_x - cx

        # ✅ PID calculations
        self.integral += error
        derivative = error - self.prev_error
        angular_velocity = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # ✅ Create Twist message for heading correction
        cmd = Twist()
        cmd.angular.z = angular_velocity  # Rotate to center object in camera view
        cmd.linear.x = 1.0  # Move forward slowly

        # ✅ Publish velocity command
        self.cmd_vel_publisher.publish(cmd)

        # ✅ Update previous error for next iteration
        self.prev_error = error

    def visualize_normal_image(self):
        if self.normal_image is not None:
            cv2.imshow("Camera1 View", self.normal_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = NormalImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
