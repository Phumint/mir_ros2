#!/usr/bin/env python3

# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Martin Günther
# ROS 2 Port: Assistant

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, HistoryPolicy

class Rep117Filter(Node):
    def __init__(self):
        super().__init__('rep117_filter')
        
        # Publisher (Standard QoS is usually fine for outputting to local nodes)
        qos_publisher = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(LaserScan, 'scan_filtered', qos_publisher)
        
        # Subscriber (Use sensor_data QoS for incoming hardware streams)
        self.sub = self.create_subscription(
            LaserScan, 
            'scan', 
            self.callback, 
            qos_profile_sensor_data
        )

    def callback(self, msg):
        """
        Convert laser scans to REP 117 standard.
        See http://www.ros.org/reps/rep-0117.html
        """
        ranges_out = []
        for dist in msg.ranges:
            if dist < msg.range_min:
                # assume "reading too close to measure"
                ranges_out.append(float("-inf"))
            elif dist > msg.range_max:
                # assume "reading of no return (outside sensor range)"
                ranges_out.append(float("inf"))
            else:
                ranges_out.append(dist)

        # Replace the ranges array and publish
        msg.ranges = ranges_out
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Rep117Filter()
    
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