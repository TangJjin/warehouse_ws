#!/usr/bin/env python3
# barcode_capture_saver.py [output_dir]
#
# Subscribes to the qr_vision_node barcode capture topic (/drone/image by
# default) and saves each JPEG payload to a file, printing the barcode string.
# Used to verify that QR/OCR decoding and the hover-end capture publish work.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   source ~/warehouse_ws/install/setup.bash
#   python3 barcode_capture_saver.py /tmp/captures
import os
import sys

import rclpy
from rclpy.node import Node

from drone_msgs.msg import BarcodeCapture


class BarcodeCaptureSaver(Node):
    def __init__(self, output_dir):
        super().__init__('barcode_capture_saver')
        self.output_dir = output_dir
        self.count = 0
        self.subscription = self.create_subscription(
            BarcodeCapture,
            '/drone/image',
            self.on_capture,
            10)
        self.get_logger().info(
            'saving /drone/image captures to %s', output_dir)

    def on_capture(self, msg):
        self.count += 1
        path = os.path.join(self.output_dir, 'capture_%03d.jpg' % self.count)
        with open(path, 'wb') as file:
            file.write(bytes(msg.image_data))
        self.get_logger().info(
            '[%d] barcode=%s bytes=%d saved=%s',
            self.count,
            msg.barcode,
            len(msg.image_data),
            path)


def main():
    rclpy.init()
    output_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/captures'
    os.makedirs(output_dir, exist_ok=True)
    node = BarcodeCaptureSaver(output_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
