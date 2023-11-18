#!/usr/bin/env python3

from ar_slam_interfaces.srv import LoadImages
import rclpy
from rclpy.node import Node


class LoadImagesClientAsync(Node):
    def __init__(self):
        super().__init__('load_images_client_async')
        self.cli = self.create_client(LoadImages, 'load_images')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadImages.Request()

    def send_request(self, img_fns):
        self.req.img_fns = img_fns
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
