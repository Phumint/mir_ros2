import rclpy
from rclpy.node import Node
import websocket
import json
import threading
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class MiRBridge(Node):
    def __init__(self):
        super().__init__('mir_bridge_node')

        # Parameters
        self.declare_parameter('mir_ip', '192.168.12.20')
        self.declare_parameter('mir_port', 9090)
        
        self.mir_ip = self.get_parameter('mir_ip').value
        self.port = self.get_parameter('mir_port').value

        # ROS 2 Publishers & TF
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS 2 Subscriber (Command Velocity)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # WebSocket Connection
        self.ws_url = f"ws://{self.mir_ip}:{self.port}"
        self.ws = None
        self.connect_to_mir()

    def connect_to_mir(self):
        """Starts the WebSocket connection in a separate thread."""
        self.get_logger().info(f"Connecting to MiR at {self.ws_url}...")
        
        # Define callbacks
        self.ws = websocket.WebSocketApp(self.ws_url,
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close)
        
        # Run in a thread so it doesn't block the ROS spin loop
        wst = threading.Thread(target=self.ws.run_forever)
        wst.daemon = True
        wst.start()

    def on_open(self, ws):
        self.get_logger().info("Connected to MiR WebSocket!")
        
        # Tell rosbridge (on MiR) that we want to subscribe to /odom
        # Note: You might need to check the exact topic name on the MiR (e.g., /odom or /odom_enc)
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/odom",
            "type": "nav_msgs/Odometry"
        }
        ws.send(json.dumps(subscribe_msg))

    def on_message(self, ws, message):
        """Handle incoming JSON messages from MiR."""
        data = json.loads(message)
        
        # Check if the message is about the topic we care about
        if 'topic' in data and data['topic'] == '/odom':
            self.process_odom(data['msg'])

    def process_odom(self, msg_data):
        """Translate JSON Odom -> ROS 2 Odom + TF"""
        
        # 1. Create ROS 2 Odometry Message
        odom_msg = Odometry()
        
        # CRITICAL: Overwrite timestamp with current ROS 2 time
        # Using the robot's time will cause TF errors due to clock desync
        now = self.get_clock().now().to_msg()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Extract Position (JSON keys match ROS 1 msg structure)
        geo = msg_data['pose']['pose']
        odom_msg.pose.pose.position.x = float(geo['position']['x'])
        odom_msg.pose.pose.position.y = float(geo['position']['y'])
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.x = float(geo['orientation']['x'])
        odom_msg.pose.pose.orientation.y = float(geo['orientation']['y'])
        odom_msg.pose.pose.orientation.z = float(geo['orientation']['z'])
        odom_msg.pose.pose.orientation.w = float(geo['orientation']['w'])

        # Publish the Odometry message
        self.odom_pub.publish(odom_msg)

        # 2. Broadcast Transform (odom -> base_link)
        # Nav2 requires this TF to function
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg):
        """Translate ROS 2 Twist -> JSON -> Send to MiR"""
        if self.ws and self.ws.sock and self.ws.sock.connected:
            # Construct rosbridge publish message
            cmd_msg = {
                "op": "publish",
                "topic": "/cmd_vel",
                "msg": {
                    "linear": {
                        "x": msg.linear.x,
                        "y": 0.0,
                        "z": 0.0
                    },
                    "angular": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": msg.angular.z
                    }
                }
            }
            self.ws.send(json.dumps(cmd_msg))

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().warn("WebSocket Connection Closed")

def main(args=None):
    rclpy.init(args=args)
    node = MiRBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()