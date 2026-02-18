import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import json
import threading
import time
import copy
from websocket import WebSocketApp

# ROS 2 Message Conversion Tools
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

# Import necessary message types
from geometry_msgs.msg import Twist, Pose, PoseStamped, PolygonStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Header
from diagnostic_msgs.msg import DiagnosticArray

# --- FILTERS (The "Script B" Magic) ---

def filter_prepend_tf_prefix(msg_dict, prefix):
    """Recursively prepends a prefix to all frame_ids in the message dict."""
    if not prefix:
        return msg_dict
    
    if isinstance(msg_dict, dict):
        for key, value in msg_dict.items():
            if key == 'header' and 'frame_id' in value:
                # Don't prefix 'map' frame, usually global
                if value['frame_id'].strip('/') != 'map':
                    original = value['frame_id'].strip('/')
                    value['frame_id'] = f"{prefix}/{original}"
            
            elif key == 'child_frame_id': # Common in Odometry and TF
                original = value.strip('/')
                msg_dict[key] = f"{prefix}/{original}"
            
            else:
                filter_prepend_tf_prefix(value, prefix)
                
    elif isinstance(msg_dict, list):
        for item in msg_dict:
            filter_prepend_tf_prefix(item, prefix)
            
    return msg_dict

def filter_remove_tf_prefix(msg_dict, prefix):
    """Removes the prefix from ROS 2 messages before sending to MiR."""
    if not prefix:
        return msg_dict
        
    # Implementation simplified for outbound commands (usually just Header frame_id)
    if 'header' in msg_dict and 'frame_id' in msg_dict['header']:
        fid = msg_dict['header']['frame_id']
        if fid.startswith(prefix):
            msg_dict['header']['frame_id'] = fid.replace(f"{prefix}/", "")
    return msg_dict

# --- CONFIGURATION CLASS ---

class TopicConfig:
    def __init__(self, topic, msg_type, direction='OUT', latch=False, dict_filter=None):
        self.topic = topic
        self.msg_type = msg_type
        self.direction = direction  # 'OUT' (Robot->ROS) or 'IN' (ROS->Robot)
        self.latch = latch
        self.dict_filter = dict_filter

# --- MAIN NODE ---

class MiRBridge(Node):
    def __init__(self):
        super().__init__('mir_bridge_ros2')

        # 1. Declare Parameters
        self.declare_parameter('mir_ip', '192.168.12.20')
        self.declare_parameter('mir_port', 9090)
        self.declare_parameter('tf_prefix', '') # Leave empty for single robot
        
        self.ip = self.get_parameter('mir_ip').value
        self.port = self.get_parameter('mir_port').value
        self.tf_prefix = self.get_parameter('tf_prefix').value
        
        if self.tf_prefix:
            self.get_logger().info(f"Operating with TF Prefix: {self.tf_prefix}")

        # 2. Define Topics (Production List)
        # We wrap the generic filter with our specific prefix
        self.topics = [
            # -- OUT (Sensors) --
            TopicConfig('/odom', Odometry, 'OUT'),
            TopicConfig('/scan', LaserScan, 'OUT'),
            TopicConfig('/b_scan', LaserScan, 'OUT'),
            TopicConfig('/f_scan', LaserScan, 'OUT'),
            TopicConfig('/imu_data', Imu, 'OUT'),
            TopicConfig('/robot_pose', Pose, 'OUT'),
            TopicConfig('/tf', TFMessage, 'OUT'),
            TopicConfig('/tf_static', TFMessage, 'OUT', latch=True),
            TopicConfig('/diagnostics', DiagnosticArray, 'OUT'),
            
            # -- IN (Commands) --
            TopicConfig('/cmd_vel', Twist, 'IN'),
            TopicConfig('/move_base_simple/goal', PoseStamped, 'IN'),
            TopicConfig('/initialpose', PoseStamped, 'IN'), # For resetting AMCL
        ]

        # 3. Setup Internals
        self.ws_url = f"ws://{self.ip}:{self.port}"
        self.ws = None
        self.ws_lock = threading.Lock()
        
        self.pubs = {} 
        self.subs = {} 

        # 4. Initialize ROS Publishers/Subscribers
        self._setup_ros_interfaces()

        # 5. Start Connection
        self.ws_thread = threading.Thread(target=self.run_ws)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def _setup_ros_interfaces(self):
        """Creates all ROS 2 publishers and subscribers based on config."""
        for cfg in self.topics:
            # Add prefix to ROS topic name if needed, or keep standard
            # Usually we publish to /mir_prefix/odom if prefix is set
            ros_topic_name = cfg.topic
            if self.tf_prefix and not ros_topic_name.startswith('/'):
                 ros_topic_name = f"{self.tf_prefix}/{cfg.topic}"

            if cfg.direction == 'OUT':
                # Robot -> ROS (Publisher)
                qos = QoSProfile(depth=10)
                if cfg.latch:
                    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                
                pub = self.create_publisher(cfg.msg_type, ros_topic_name, qos)
                self.pubs[cfg.topic] = pub
                
            elif cfg.direction == 'IN':
                # ROS -> Robot (Subscriber)
                self.subs[cfg.topic] = self.create_subscription(
                    cfg.msg_type,
                    ros_topic_name,
                    lambda msg, c=cfg: self.ros_to_mir_callback(msg, c),
                    10
                )

    def run_ws(self):
        """Auto-reconnecting WebSocket Loop"""
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to {self.ws_url}...")
                self.ws = WebSocketApp(self.ws_url,
                                       on_open=self.on_open,
                                       on_message=self.on_message,
                                       on_error=self.on_error,
                                       on_close=self.on_close)
                self.ws.run_forever()
            except Exception as e:
                self.get_logger().error(f"Connection failed: {e}")
            
            self.get_logger().warn("Reconnecting in 5s...")
            time.sleep(5)

    def on_open(self, ws):
        self.get_logger().info("Connection Established!")
        for cfg in self.topics:
            # Use the ROS 1 type name
            pkg = cfg.msg_type.__module__.split('.')[0]
            ros1_type = f"{pkg}/{cfg.msg_type.__name__}"
            
            if cfg.direction == 'OUT':
                ws.send(json.dumps({"op": "subscribe", "topic": cfg.topic, "type": ros1_type}))
            elif cfg.direction == 'IN':
                # IMPORTANT: MiR needs to know you are going to publish /cmd_vel
                # Note: Even if we send TwistStamped data, we advertise the topic name as it exists on the MiR
                actual_type = "geometry_msgs/TwistStamped" if cfg.topic == '/cmd_vel' else ros1_type
                ws.send(json.dumps({"op": "advertise", "topic": cfg.topic, "type": actual_type}))

    def on_message(self, ws, message):
        """Handle incoming data from MiR."""
        try:
            data = json.loads(message)
            
            if 'topic' not in data or 'msg' not in data:
                return

            topic = data['topic']
            msg_dict = data['msg']
            
            if topic in self.pubs:
                # ==========================================
                # PATCH 1: REMOVE 'seq' (Fixes the crash)
                # ==========================================
                # Fix main header
                if 'header' in msg_dict and 'seq' in msg_dict['header']:
                    del msg_dict['header']['seq']

                # Fix TF headers (Transforms are lists of headers)
                if 'transforms' in msg_dict:
                    for t in msg_dict['transforms']:
                        if 'header' in t and 'seq' in t['header']:
                            del t['header']['seq']

                # ==========================================
                # PATCH 2: FIX TIME (secs -> sec)
                # ==========================================
                if 'header' in msg_dict and 'stamp' in msg_dict['header']:
                    stamp = msg_dict['header']['stamp']
                    if 'secs' in stamp: stamp['sec'] = stamp.pop('secs')
                    if 'nsecs' in stamp: stamp['nanosec'] = stamp.pop('nsecs')
                
                if 'transforms' in msg_dict:
                    for t in msg_dict['transforms']:
                        if 'header' in t and 'stamp' in t['header']:
                            s = t['header']['stamp']
                            if 'secs' in s: s['sec'] = s.pop('secs')
                            if 'nsecs' in s: s['nanosec'] = s.pop('nsecs')

                # ==========================================
                # PATCH 3: FIX DIAGNOSTICS (Byte conversion)
                # ==========================================
                if topic == '/diagnostics' and 'status' in msg_dict:
                    for status in msg_dict['status']:
                        if 'level' in status:
                             # ROS 2 expects a byte, but JSON gives int. 
                             # We force it to bytes.
                             status['level'] = bytes([status['level']])

                # 1. Apply TF Prefix Filter
                if self.tf_prefix:
                    msg_dict = filter_prepend_tf_prefix(msg_dict, self.tf_prefix)

                # 2. Convert Dictionary -> ROS 2 Message
                target_type = next((t.msg_type for t in self.topics if t.topic == topic), None)
                if not target_type: return

                msg = target_type()
                set_message_fields(msg, msg_dict)

                # 3. Force Timestamp Update (Optional: keeps data "fresh")
                current_time = self.get_clock().now().to_msg()
                if hasattr(msg, 'header'):
                    msg.header.stamp = current_time
                
                if topic == '/tf' or topic == '/tf_static':
                    for t in msg.transforms:
                        t.header.stamp = current_time

                self.pubs[topic].publish(msg)

        except Exception as e:
            # Print the specific error to help debug
            self.get_logger().warn(f"Parse error on {topic}: {e}")

    def ros_to_mir_callback(self, msg, cfg):
        try:
            # 1. Convert ROS 2 message to Dict
            msg_dict = message_to_ordereddict(msg)
            
            # 2. MATCH THE DFKI FILTER: Wrap Twist into TwistStamped
            if cfg.topic == '/cmd_vel':
                # MiR >= 2.7 expects geometry_msgs/TwistStamped
                # We construct the ROS 1-style dictionary manually
                ros1_type = "geometry_msgs/TwistStamped"
                payload = {
                    'header': {
                        'seq': 0,
                        'stamp': {'secs': int(time.time()), 'nsecs': 0},
                        'frame_id': ''
                    },
                    'twist': msg_dict  # This contains the linear/angular dicts
                }
            else:
                # Standard logic for other topics
                pkg = cfg.msg_type.__module__.split('.')[0]
                name = cfg.msg_type.__name__
                ros1_type = f"{pkg}/{name}"
                payload = msg_dict

            # 3. Construct the ROS 1 Bridge JSON
            out_msg = {
                'op': 'publish',
                'topic': cfg.topic,
                'type': ros1_type,
                'msg': payload
            }
            
            if self.ws:
                self.ws.send(json.dumps(out_msg))
                
        except Exception as e:
            self.get_logger().warn(f"Failed to send to MiR: {e}")

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket Error: {error}")

    def on_close(self, ws, status, msg):
        self.get_logger().warn("WebSocket Closed")

def main(args=None):
    rclpy.init(args=args)
    node = MiRBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
