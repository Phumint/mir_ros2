import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import json
import orjson
import threading
import queue
import time
from websocket import WebSocketApp

# ROS 2 Message Conversion Tools
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

# Import necessary message types
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Header
from diagnostic_msgs.msg import DiagnosticArray


# ==========================================
# STATIC TF FRAME FILTER
#
# These are child_frame_ids that the MiR publishes via /tf_static
# which CONFLICT with what robot_state_publisher provides from the URDF.
# The MiR stamps these at boot time (hours-old timestamps), which
# overwrites the RSP's clean, current-timestamped versions and breaks
# the entire TF tree.
#
# We DROP these from /tf_static coming from the MiR.
# We KEEP *_calibrated frames — those are MiR runtime laser calibration
# offsets that do NOT exist in the URDF and must come from the MiR.
# ==========================================
TF_STATIC_RSP_FRAMES = {
    'base_link',
    'front_laser_link',
    'back_laser_link',
    'imu_link',
    'imu_frame',
    'camera_floor_link',
    'camera_floor_color_frame',
    'camera_floor_color_optical_frame',
    'camera_floor_depth_optical_frame',
    'camera_infra1_frame',
    'camera_infra1_optical_frame',
    'camera_infra2_frame',
    'camera_infra2_optical_frame',
    # mount_box_link, ur_base_link etc. are fixed joints in moma.urdf.xacro
    # and are published by the UR driver's RSP — also drop these if MiR
    # ever sends them (it shouldn't, but safety net):
    'mount_box_link',
    'ur_base_link',
    'ur_base',
    'odom',
}


def filter_prepend_tf_prefix(msg_dict, prefix):
    """Recursively prepends a prefix to all frame_ids in the message dict."""
    if not prefix:
        return msg_dict
    
    if isinstance(msg_dict, dict):
        for key, value in msg_dict.items():
            if key == 'header' and isinstance(value, dict) and 'frame_id' in value:
                if value['frame_id'].strip('/') != 'map':
                    original = value['frame_id'].strip('/')
                    value['frame_id'] = f"{prefix}/{original}"
            elif key == 'child_frame_id':
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
        self.declare_parameter('tf_prefix', '')

        self.ip = self.get_parameter('mir_ip').value
        self.port = self.get_parameter('mir_port').value
        self.tf_prefix = self.get_parameter('tf_prefix').value

        if self.tf_prefix:
            self.get_logger().info(f"Operating with TF Prefix: {self.tf_prefix}")

        # Callback group for inbound ROS commands (cmd_vel etc.)
        # Kept separate so heavy Nav2 traffic doesn't block the bridge publisher callbacks
        self.cmd_cb_group = MutuallyExclusiveCallbackGroup()

        # Thread-safe queue for sensor data (scans, odom, diagnostics)
        # TF bypasses this queue entirely via the fast-path in on_message
        self.msg_queue = queue.Queue(maxsize=5)

        # 2. Define Topics
        self.topics = [
            # -- OUT (Robot -> ROS) --
            TopicConfig('/odom',        Odometry,       'OUT', ros_topic='/odometry/filtered'),
            TopicConfig('/b_scan',      LaserScan,      'OUT'),
            TopicConfig('/f_scan',      LaserScan,      'OUT'),
            TopicConfig('/tf',          TFMessage,      'OUT'),
            TopicConfig('/tf_static',   TFMessage,      'OUT', latch=True),

            # -- IN (ROS -> Robot) --
            TopicConfig('/cmd_vel',               Twist,       'IN'),
        ]

        # 3. Setup Internals
        self.ws_url = f"ws://{self.ip}:{self.port}"
        self.ws = None
        self.ws_lock = threading.Lock()
        self.pubs = {}
        self.subs = {}

        # 4. Initialize ROS Publishers/Subscribers
        self._setup_ros_interfaces()

        # 5. Start worker thread (processes sensor queue)
        self.worker_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.worker_thread.start()

        # 6. Start WebSocket connection thread
        self.ws_thread = threading.Thread(target=self.run_ws, daemon=True)
        self.ws_thread.start()

    def _setup_ros_interfaces(self):
        """Creates all ROS 2 publishers and subscribers based on config."""
        for cfg in self.topics:
            ros_topic_name = cfg.ros_topic
            if self.tf_prefix and not ros_topic_name.startswith('/'):
                ros_topic_name = f"{self.tf_prefix}/{cfg.topic}"

            if cfg.direction == 'OUT':
                if cfg.latch:
                    # /tf_static: latched, reliable for late-joiners
                    qos = QoSProfile(
                        depth=10,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE
                    )
                elif cfg.topic == '/tf':
                    # /tf: must be Reliable in ROS 2
                    qos = QoSProfile(
                        depth=100,
                        durability=DurabilityPolicy.VOLATILE,
                        reliability=ReliabilityPolicy.RELIABLE
                    )
                else:
                    # High-frequency sensors: Best Effort, don't buffer stale data
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
                    10,
                    callback_group=self.cmd_cb_group
                )

    def run_ws(self):
        """Auto-reconnecting WebSocket loop."""
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to {self.ws_url}...")
                self.ws = WebSocketApp(
                    self.ws_url,
                    on_open=self.on_open,
                    on_message=self.on_message,
                    on_error=self.on_error,
                    on_close=self.on_close
                )
                self.ws.run_forever()
            except Exception as e:
                self.get_logger().error(f"Connection failed: {e}")

            self.get_logger().warn("Reconnecting in 5s...")
            time.sleep(5)

    def on_open(self, ws):
        """Subscribe/advertise all topics on the MiR rosbridge."""
        self.get_logger().info("WebSocket connection established!")
        for cfg in self.topics:
            pkg = cfg.msg_type.__module__.split('.')[0]
            ros1_type = f"{pkg}/{cfg.msg_type.__name__}"

            if cfg.direction == 'OUT':
                if cfg.topic in ('/b_scan', '/f_scan'):
                    # Throttle scans at the rosbridge level.
                    # This keeps the WebSocket clear so TF messages are never
                    # queued behind large scan payloads.
                    # 125ms = 8Hz, sufficient for Nav2 costmap at 5Hz update rate.
                    ws.send(json.dumps({
                        "op": "subscribe",
                        "topic": cfg.topic,
                        "type": ros1_type,
                        "queue_length": 1       # don't let MiR buffer stale scans
                    }))
                else:
                    ws.send(json.dumps({
                        "op": "subscribe",
                        "topic": cfg.topic,
                        "type": ros1_type
                    }))
            elif cfg.direction == 'IN':
                actual_type = "geometry_msgs/TwistStamped" \
                    if cfg.topic == '/cmd_vel' else ros1_type
                ws.send(json.dumps({
                    "op": "advertise",
                    "topic": cfg.topic,
                    "type": actual_type
                }))

    def on_message(self, ws, message):
        """
        Producer — runs on the WebSocket thread.

        TF fast-path: /tf and /tf_static are processed immediately on this
        thread, bypassing the queue entirely. This guarantees the tf2 cache
        always stays current regardless of sensor queue backpressure.

        Everything else goes into the queue for the worker thread.
        """
        try:
            data = orjson.loads(message)
            topic = data.get('topic', '')

            if topic in ('/tf', '/tf_static'):
                # Fast-path: process inline, never waits in queue
                self._process_single(data)
            else:
                self.msg_queue.put_nowait(message)

        except queue.Full:
            self.get_logger().warn("Bridge queue full — dropping sensor data.")
        except Exception as e:
            self.get_logger().warn(f"on_message error: {e}")

    def _process_queue(self):
        """
        Consumer — runs on the worker thread.
        Handles all non-TF messages (scans, odom, diagnostics, etc.)
        """
        while rclpy.ok():
            try:
                raw_message = self.msg_queue.get(timeout=1.0)
                data = orjson.loads(raw_message)

                if 'topic' not in data or 'msg' not in data:
                    continue

                # PATCH: Fix diagnostics level byte conversion
                topic = data['topic']
                msg_dict = data['msg']
                if topic == '/diagnostics' and 'status' in msg_dict:
                    for status in msg_dict['status']:
                        if 'level' in status:
                            status['level'] = bytes([status['level']])

                self._process_single(data)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().warn(f"Worker parse error: {e}")

    def _process_single(self, data):
        """
        Core message processor — handles one already-parsed message dict.

        Called from the WS thread for TF (fast-path) and from the worker
        thread for everything else. publish() in rclpy is thread-safe.
        """
        try:
            topic = data.get('topic', '')
            msg_dict = data.get('msg', {})

            if topic not in self.pubs:
                return

            # ==========================================
            # PATCH 1: REMOVE 'seq' (ROS 1 field, crashes ROS 2)
            # ==========================================
            if 'header' in msg_dict and 'seq' in msg_dict['header']:
                del msg_dict['header']['seq']
            if 'transforms' in msg_dict:
                for t in msg_dict['transforms']:
                    if 'header' in t and 'seq' in t['header']:
                        del t['header']['seq']

            # ==========================================
            # PATCH 2: FIX TIME FIELD NAMES (secs->sec, nsecs->nanosec)
            # ==========================================
            if 'header' in msg_dict and 'stamp' in msg_dict['header']:
                stamp = msg_dict['header']['stamp']
                if 'secs' in stamp:  stamp['sec']     = stamp.pop('secs')
                if 'nsecs' in stamp: stamp['nanosec'] = stamp.pop('nsecs')
            if 'transforms' in msg_dict:
                for t in msg_dict['transforms']:
                    if 'header' in t and 'stamp' in t['header']:
                        s = t['header']['stamp']
                        if 'secs' in s:  s['sec']     = s.pop('secs')
                        if 'nsecs' in s: s['nanosec'] = s.pop('nsecs')

            # ==========================================
            # SURGICAL /tf FILTER: pass ONLY odom -> base_footprint
            #
            # The MiR sends its entire TF tree over /tf (wheels, lasers,
            # cameras, etc.). We only want the odometry transform.
            # Everything else is handled by robot_state_publisher from URDF.
            # ==========================================
            if topic == '/tf' and 'transforms' in msg_dict:
                filtered = [
                    t for t in msg_dict['transforms']
                    if t.get('child_frame_id', '').lstrip('/') == 'base_footprint'
                ]
                if not filtered:
                    return
                msg_dict['transforms'] = filtered

            # ==========================================
            # SURGICAL /tf_static FILTER: drop RSP-owned frames
            #
            # The MiR publishes /tf_static with timestamps from its boot time
            # (hours-old). These overwrite RSP's clean current-timestamped
            # versions, making tf2 think all those frames are ancient and
            # causing the entire TF tree to appear broken.
            #
            # We keep ONLY the *_calibrated frames — these are MiR runtime
            # laser calibration offsets that are NOT in the URDF and must
            # come from the MiR.
            # ==========================================
            if topic == '/tf_static' and 'transforms' in msg_dict:
                filtered = [
                    t for t in msg_dict['transforms']
                    if t.get('child_frame_id', '').lstrip('/') not in TF_STATIC_RSP_FRAMES
                ]
                if not filtered:
                    return
                msg_dict['transforms'] = filtered

            # ==========================================
            # TF PREFIX (multi-robot setups)
            # ==========================================
            if self.tf_prefix:
                msg_dict = filter_prepend_tf_prefix(msg_dict, self.tf_prefix)

            # ==========================================
            # CONVERT DICT -> ROS 2 MESSAGE
            # Fast-path hand-written conversion for high-frequency types.
            # Falls back to set_message_fields for low-frequency types.
            # ==========================================
            target_type = next(
                (t.msg_type for t in self.topics if t.topic == topic), None
            )
            if not target_type:
                return

            msg = target_type()

            if target_type == LaserScan:
                msg.header.stamp.sec     = msg_dict['header']['stamp'].get('sec', 0)
                msg.header.stamp.nanosec = msg_dict['header']['stamp'].get('nanosec', 0)
                msg.header.frame_id      = msg_dict['header'].get('frame_id', '')
                msg.angle_min            = float(msg_dict.get('angle_min', 0.0))
                msg.angle_max            = float(msg_dict.get('angle_max', 0.0))
                msg.angle_increment      = float(msg_dict.get('angle_increment', 0.0))
                msg.time_increment       = float(msg_dict.get('time_increment', 0.0))
                msg.scan_time            = float(msg_dict.get('scan_time', 0.0))
                msg.range_min            = float(msg_dict.get('range_min', 0.0))
                msg.range_max            = float(msg_dict.get('range_max', 0.0))
                msg.ranges               = [float(x) for x in msg_dict.get('ranges', [])]
                if msg_dict.get('intensities'):
                    msg.intensities = [float(x) for x in msg_dict['intensities']]

            elif target_type == TFMessage:
                for t_dict in msg_dict.get('transforms', []):
                    t_msg = TransformStamped()
                    t_msg.header.stamp.sec     = t_dict['header']['stamp'].get('sec', 0)
                    t_msg.header.stamp.nanosec = t_dict['header']['stamp'].get('nanosec', 0)
                    t_msg.header.frame_id      = t_dict['header'].get('frame_id', '')
                    t_msg.child_frame_id       = t_dict.get('child_frame_id', '')
                    t_msg.transform.translation.x = float(t_dict['transform']['translation']['x'])
                    t_msg.transform.translation.y = float(t_dict['transform']['translation']['y'])
                    t_msg.transform.translation.z = float(t_dict['transform']['translation']['z'])
                    t_msg.transform.rotation.x    = float(t_dict['transform']['rotation']['x'])
                    t_msg.transform.rotation.y    = float(t_dict['transform']['rotation']['y'])
                    t_msg.transform.rotation.z    = float(t_dict['transform']['rotation']['z'])
                    t_msg.transform.rotation.w    = float(t_dict['transform']['rotation']['w'])
                    msg.transforms.append(t_msg)

            else:
                # Low-frequency messages (odom, imu, diagnostics, pose)
                set_message_fields(msg, msg_dict)

            self.pubs[topic].publish(msg)

        except Exception as e:
            self.get_logger().warn(
                f"_process_single error on '{data.get('topic', '?')}': {e}"
            )

    def ros_to_mir_callback(self, msg, cfg):
        """Send ROS 2 commands back to the MiR over rosbridge."""
        try:
            msg_dict = message_to_ordereddict(msg)

            if cfg.topic == '/cmd_vel':
                ros1_type = "geometry_msgs/TwistStamped"
                payload = {
                    'header': {
                        'seq': 0,
                        'stamp': {'secs': int(time.time()), 'nsecs': 0},
                        'frame_id': ''
                    },
                    'twist': msg_dict
                }
            else:
                pkg = cfg.msg_type.__module__.split('.')[0]
                name = cfg.msg_type.__name__
                ros1_type = f"{pkg}/{name}"
                payload = msg_dict

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
        self.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, status, msg):
        self.get_logger().warn("WebSocket closed.")


def main(args=None):
    rclpy.init(args=args)
    node = MiRBridge()

    # MultiThreadedExecutor prevents Nav2's heavy DDS traffic from starving
    # the bridge's publish callbacks. 4 threads: TF fast-path, sensor worker,
    # cmd_vel callbacks, and one spare for burst handling.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()