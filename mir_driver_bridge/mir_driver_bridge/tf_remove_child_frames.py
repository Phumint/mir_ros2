#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2016 The Cartographer Authors
# Copyright 2018 DFKI GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ... [Original Apache 2.0 License preserved] ...
# ROS 2 Port: Assistant

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class TFRemoveChildFrames(Node):
    def __init__(self):
        super().__init__('tf_remove_child_frames')
        
        # 1. Declare and read the parameter list
        self.declare_parameter('remove_frames', [])
        self.remove_frames = self.get_parameter('remove_frames').value
        
        if not self.remove_frames:
            self.get_logger().warn("No frames specified in 'remove_frames'. Node will not filter anything.")

        # 2. Setup QoS Profiles
        # Standard dynamic TF (frequent updates)
        qos_dynamic = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Latched static TF (Equivalent to latch=True in ROS 1)
        qos_static = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 3. Setup Publishers
        self.tf_pub = self.create_publisher(TFMessage, 'tf_out', qos_dynamic)
        self.tf_static_pub = self.create_publisher(TFMessage, 'tf_static_out', qos_static)

        # 4. Setup Subscribers
        self.tf_sub = self.create_subscription(TFMessage, 'tf_in', self.tf_cb, qos_dynamic)
        self.tf_static_sub = self.create_subscription(TFMessage, 'tf_static_in', self.tf_static_cb, qos_static)

    def tf_cb(self, msg):
        # Filter the transforms list based on the child_frame_id
        msg.transforms = [t for t in msg.transforms if t.child_frame_id.lstrip('/') not in self.remove_frames]
        
        # If there are still transforms left after filtering, publish them
        if msg.transforms:
            self.tf_pub.publish(msg)

    def tf_static_cb(self, msg):
        # Same logic, but for static transforms
        msg.transforms = [t for t in msg.transforms if t.child_frame_id.lstrip('/') not in self.remove_frames]
        
        if msg.transforms:
            self.tf_static_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRemoveChildFrames()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()