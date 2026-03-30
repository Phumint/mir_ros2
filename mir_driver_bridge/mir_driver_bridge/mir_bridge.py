import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import json
import orjson  # Add this for high-speed parsing
import threading
import queue
import time
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
    def __init__(self, topic, msg_type, direction='OUT', latch=False, dict_filter=None, ros_topic=None):
        self.topic = topic
        self.msg_type = msg_type
        self.direction = direction  # 'OUT' (Robot->ROS) or 'IN' (ROS->Robot)
        self.latch = latch
        self.dict_filter = dict_filter
        self.ros_topic = ros_topic or topic

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

        # [NEW] Thread-safe queue for handling incoming WS messages quickly
        self.msg_queue = queue.Queue(maxsize=100)

        # 2. Define Topics (Production List)
        self.topics = [
            # -- OUT (Sensors) --
            TopicConfig('/odom', Odometry, 'OUT', ros_topic='/odometry/filtered'),
            # TopicConfig('/scan', LaserScan, 'OUT'),
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

        # [NEW] Start Worker Thread for JSON parsing
        self.worker_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.worker_thread.start()

        # 5. Start Connection
        self.ws_thread = threading.Thread(target=self.run_ws)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def _setup_ros_interfaces(self):
        """Creates all ROS 2 publishers and subscribers based on config."""
        for cfg in self.topics:
            ros_topic_name = cfg.ros_topic
            if self.tf_prefix and not ros_topic_name.startswith('/'):
                 ros_topic_name = f"{self.tf_prefix}/{cfg.topic}"

            if cfg.direction == 'OUT':
                if cfg.latch:
                    # 1. /tf_static (Latched, Reliable for late-joiners)
                    qos = QoSProfile(
                        depth=10,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE
                    )
                elif cfg.topic == '/tf':
                    # 2. /tf MUST be Reliable in ROS 2, or standard nodes reject it!
                    qos = QoSProfile(
                        depth=100, 
                        durability=DurabilityPolicy.VOLATILE,
                        reliability=ReliabilityPolicy.RELIABLE
                    )
                else:
                    # 3. /scan, /odom, /imu_data (High-speed sensors: Best Effort, drop old data)
                    qos = QoSProfile(
                        depth=1,  
                        durability=DurabilityPolicy.VOLATILE,
                        reliability=ReliabilityPolicy.BEST_EFFORT
                    )
                
                pub = self.create_publisher(cfg.msg_type, ros_topic_name, qos)
                self.pubs[cfg.topic] = pub
                
            elif cfg.direction == 'IN':
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
            pkg = cfg.msg_type.__module__.split('.')[0]
            ros1_type = f"{pkg}/{cfg.msg_type.__name__}"
            
            if cfg.direction == 'OUT': 
                ws.send(json.dumps({"op": "subscribe", "topic": cfg.topic, "type": ros1_type}))
            elif cfg.direction == 'IN':
                actual_type = "geometry_msgs/TwistStamped" if cfg.topic == '/cmd_vel' else ros1_type
                ws.send(json.dumps({"op": "advertise", "topic": cfg.topic, "type": actual_type}))

    def on_message(self, ws, message):
        """[NEW] Producer: Just drops raw text into the queue instantly."""
        try:
            self.msg_queue.put_nowait(message)
        except queue.Full:
            self.get_logger().warn("Bridge Queue Full! Dropping data.")

    def _process_queue(self):
        """[NEW] Consumer: Handles incoming data and translation in the background."""
        while rclpy.ok():
            try:
                raw_message = self.msg_queue.get(timeout=1.0)
                data = orjson.loads(raw_message)
                
                if 'topic' not in data or 'msg' not in data:
                    continue

                topic = data['topic']
                msg_dict = data['msg']
                
                if topic in self.pubs:
                    # ==========================================
                    # PATCH 1: REMOVE 'seq' (Fixes the crash)
                    # ==========================================
                    if 'header' in msg_dict and 'seq' in msg_dict['header']:
                        del msg_dict['header']['seq']

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
                    # SURGICAL TF FILTER (Odometry ONLY)
                    # ==========================================
                    if topic == '/tf' and 'transforms' in msg_dict:
                        filtered_transforms = []
                        for t in msg_dict['transforms']:
                            child_frame = t.get('child_frame_id', '')
                            
                            # Only bridge the core odometry from the MiR base
                            if child_frame == 'base_footprint':
                                filtered_transforms.append(t)
                                
                        # If it's anything else (wheels, lasers), drop it.
                        # The laptop's robot_state_publisher handles the rest!
                        if not filtered_transforms:
                            continue
                        
                        msg_dict['transforms'] = filtered_transforms

                    # ==========================================
                    # PATCH 3: FIX DIAGNOSTICS (Byte conversion)
                    # ==========================================
                    if topic == '/diagnostics' and 'status' in msg_dict:
                        for status in msg_dict['status']:
                            if 'level' in status:
                                status['level'] = bytes([status['level']])

                    # 1. Apply TF Prefix Filter
                    if self.tf_prefix:
                        msg_dict = filter_prepend_tf_prefix(msg_dict, self.tf_prefix)

                    # 2. Convert Dictionary -> ROS 2 Message
                    target_type = next((t.msg_type for t in self.topics if t.topic == topic), None)
                    if not target_type: continue

                    msg = target_type()
                    set_message_fields(msg, msg_dict)

                    # # ==========================================
                    # # LASER FILTER: Stop Multiplexing!
                    # # ==========================================
                    # if topic == '/scan' and hasattr(msg, 'header'):
                    #     # If it is NOT the front laser, drop it into the void
                    #     if msg_dict.get('header', {}).get('frame_id', '') != 'front_laser_link':
                    #         continue 
                            
                    #     # # If it IS the front laser, calculate latency
                    #     # msg_time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
                    #     # current_time = self.get_clock().now().nanoseconds / 1e9
                    #     # latency_ms = (current_time - msg_time) * 1000.0
                    #     # self.get_logger().info(f"Latency on /scan: {latency_ms:.2f} ms")
                    # # ==========================================

                    # # ==========================================
                    # # THE FIX: RE-STAMP ALL INCOMING DATA
                    # # Overwrite the delayed MiR timestamps with current laptop time
                    # # ==========================================
                    # now_msg = self.get_clock().now().to_msg()
                    
                    # if hasattr(msg, 'header'):
                    #     msg.header.stamp = now_msg
                        
                    # if topic == '/tf' or topic == '/tf_static':
                    #     if hasattr(msg, 'transforms'):
                    #         for t in msg.transforms:
                    #             t.header.stamp = now_msg
                    # # ==========================================

                    # 3. Publish to ROS 2
                    self.pubs[topic].publish(msg)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().warn(f"Worker parse error: {e}")

    def ros_to_mir_callback(self, msg, cfg):
        try:
            # 1. Convert ROS 2 message to Dict
            msg_dict = message_to_ordereddict(msg)
            
            # 2. MATCH THE DFKI FILTER: Wrap Twist into TwistStamped
            if cfg.topic == '/cmd_vel':
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
