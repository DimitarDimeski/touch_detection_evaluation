#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os


class DotGridDisplayNode(Node):
    def __init__(self):
        super().__init__('dot_grid_display_node')
        self.get_logger().info('Starting dot grid display node...')

        # Parameters
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('rows', 5)
        self.declare_parameter('cols', 5)
        self.declare_parameter('dot_radius', 5)
        self.declare_parameter('top_offset', 100)
        self.declare_parameter('bottom_offset', 100)
        self.declare_parameter('left_offset', 100)
        self.declare_parameter('right_offset', 100)
        self.declare_parameter('output_yaml', 'calibration.yaml')

        # Get parameter values
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.rows = self.get_parameter('rows').value
        self.cols = self.get_parameter('cols').value
        self.dot_radius = self.get_parameter('dot_radius').value
        self.top_offset = self.get_parameter('top_offset').value
        self.bottom_offset = self.get_parameter('bottom_offset').value
        self.left_offset = self.get_parameter('left_offset').value
        self.right_offset = self.get_parameter('right_offset').value
        self.output_yaml = self.get_parameter('output_yaml').value

        # Create images
        self.image = self.create_dot_grid_image()
        self.window_name = "Fullscreen Dot Grid"

        # Display image in fullscreen
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            self.window_name,
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN
        )
        cv2.imshow(self.window_name, self.image)

        # Timer to keep window alive and check shutdown conditions
        self.timer = self.create_timer(0.5, self.update_window)
        self.get_logger().info("Dot grid displayed. Node will stay open until shutdown.")

    def create_dot_grid_image(self):
        # White background
        image = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255

        usable_width = self.width - self.left_offset - self.right_offset - self.dot_radius
        usable_height = self.height - self.top_offset - self.bottom_offset - self.dot_radius    

        x_spacing = usable_width / (self.cols - 1)
        y_spacing = usable_height / (self.rows - 1)

        for r in range(self.rows):
            for c in range(self.cols):
                cx = int(self.left_offset + c * x_spacing)
                cy = int(self.top_offset + r * y_spacing)

                cv2.circle(
                    image,
                    (cx, cy),
                    self.dot_radius,
                    (0, 0, 0),   # black dots
                    thickness=-1
                )


        return image

    def update_window(self):
        # Alternate between normal and inverted image
        image_to_show =  self.image

        # Shutdown if calibration file exists
        if os.path.isfile(self.output_yaml):
            self.get_logger().info("Calibration complete. Shutting down dot grid display node.")
            rclpy.shutdown()
            return

        # Check if window was closed
        if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
            self.get_logger().info("Window closed manually. Shutting down node.")
            rclpy.shutdown()
        else:
            cv2.imshow(self.window_name, image_to_show)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.get_logger().info("Dot grid display node shutting down.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DotGridDisplayNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
