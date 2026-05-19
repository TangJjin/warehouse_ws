#!/usr/bin/env python3

import sys
import time
import argparse
import importlib

import rclpy
from rclpy.node import Node


def import_message_type(type_string: str):
    parts = type_string.split('/')
    if len(parts) != 3 or parts[1] != 'msg':
        raise ValueError(f'Invalid message type: {type_string}')

    package_name = parts[0]
    message_name = parts[2]

    module = importlib.import_module(f'{package_name}.msg')
    return getattr(module, message_name)


class TopicWaiter(Node):
    def __init__(self, topic_name: str, msg_type, timeout_sec: float):
        super().__init__('topic_waiter')
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.timeout_sec = timeout_sec
        self.received = False
        self.start_time = time.time()

        self.subscription = self.create_subscription(
            self.msg_type,
            self.topic_name,
            self.callback,
            10
        )

    def callback(self, msg):
        self.received = True
        self.get_logger().info(f'Received first message from {self.topic_name}')

    def is_timeout(self) -> bool:
        return (time.time() - self.start_time) >= self.timeout_sec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True, help='topic name')
    parser.add_argument('--type', required=True, help='message type, e.g. std_msgs/msg/String')
    parser.add_argument('--timeout', type=float, default=15.0, help='timeout in seconds')
    args = parser.parse_args()

    try:
        msg_type = import_message_type(args.type)
    except Exception as exc:
        print(f'Failed to import message type {args.type}: {exc}', file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = TopicWaiter(args.topic, msg_type, args.timeout)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)

            if node.received:
                node.get_logger().info(f'Topic ready: {args.topic}')
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(0)

            if node.is_timeout():
                node.get_logger().error(f'Timeout waiting for topic: {args.topic}')
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted while waiting for topic')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(130)


if __name__ == '__main__':
    main()