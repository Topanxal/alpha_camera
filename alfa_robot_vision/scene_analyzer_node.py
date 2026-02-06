import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import Image, CameraInfo
from logistics_interfaces.srv import AnalyzeScene
from logistics_interfaces.msg import BoxStrategy
from .utils.yolo_wrapper import YoloWrapper
from .utils.geometry_tools import GeometryProcessor

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class SceneAnalyzerNode(Node):
    def __init__(self):
        super().__init__('scene_analyzer')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', ''),
                ('target_frame', 'base_link'),
                ('camera_optical_frame', 'head_camera_optical_frame'),
                ('confidence_threshold', 0.5),
                ('debug_mode', False),
                ('layer_height_threshold', 1.2) # Z height threshold to distinguish layers (meters)
            ]
        )
        
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('camera_optical_frame').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.layer_thresh = self.get_parameter('layer_height_threshold').get_parameter_value().double_value

        # Model Path Logic
        share_dir = get_package_share_directory('alfa_robot_vision')
        models_dir = os.path.join(share_dir, 'models')
        if not self.model_path:
            candidate = os.path.join(models_dir, 'box_best.pt')
            self.model_path = candidate if os.path.exists(candidate) else os.path.join(models_dir, 'box_last.pt')
        elif not os.path.isabs(self.model_path):
            self.model_path = os.path.join(models_dir, self.model_path)
        
        # Init Components
        self.bridge = CvBridge()
        self.yolo = YoloWrapper(self.model_path, self.conf_thresh)
        self.geo_processor = None
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Service & Publisher
        self.srv = self.create_service(AnalyzeScene, 'analyze_scene', self.handle_analyze_scene)
        self.debug_pub = self.create_publisher(Image, 'scene_debug_image', 10)
        
        # Subscribers
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        
        self.create_subscription(Image, '/head_camera/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/head_camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/head_camera/camera_info', self.info_callback, 10)
        
        # Timer for Debug Auto-Trigger (10Hz)
        self.create_timer(0.1, self.debug_loop)
        
        self.get_logger().info('Scene Analyzer Initialized.')

    def debug_loop(self):
        if self.debug_mode and self.latest_rgb is not None:
             # Create a dummy request to trigger the logic
             class DummyRequest:
                 trigger = True
             class DummyResponse:
                 success = False
                 targets = []
             
             # Call handler directly (bypass service interface for debug view)
             try:
                self.handle_analyze_scene(DummyRequest(), DummyResponse())
             except Exception:
                pass

    def rgb_callback(self, msg):
        self.latest_rgb = msg

    def depth_callback(self, msg):
        self.latest_depth = msg
        
    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.geo_processor = GeometryProcessor(msg)
            self.get_logger().info('Camera Info Received.')

    def handle_analyze_scene(self, request, response):
        if not request.trigger:
            response.success = False
            return response

        if self.latest_rgb is None or self.latest_depth is None or self.geo_processor is None:
            response.success = False
            self.get_logger().warn("Missing images or camera info")
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
            
            # YOLO Detection
            detections = self.yolo.detect(cv_image) # Detect all classes (boxes)
            
            strategies = []
            debug_info = [] # Store debug info for each detection
            
            for i, det in enumerate(detections):
                # 3D Projection
                pose_cam = None
                dims = Vector3()
                fail_reason = None
                
                # Use Mask or BBox center
                if det['mask_poly'] is not None:
                    geo_res = self.geo_processor.process_box(cv_depth, det['mask_poly'])
                    if geo_res:
                        cx, cy, cz = geo_res['center']
                        yaw_rad = np.radians(geo_res['yaw'])
                        quat = euler_to_quaternion(0, 0, yaw_rad)
                        
                        pose_cam = Pose()
                        pose_cam.position.x = cx
                        pose_cam.position.y = cy
                        pose_cam.position.z = cz
                        pose_cam.orientation = quat
                    else:
                        fail_reason = "Bad Mask Depth"
                else:
                    # Fallback to BBox center
                    bbox = det['bbox']
                    u = int((bbox[0] + bbox[2]) / 2)
                    v = int((bbox[1] + bbox[3]) / 2)
                    if 0 <= u < cv_depth.shape[1] and 0 <= v < cv_depth.shape[0]:
                        d_val = cv_depth[v, u] * 0.001
                        if d_val > 0.1:
                            x, y, z = self.geo_processor.pixel_to_3d(u, v, d_val)
                            pose_cam = Pose()
                            pose_cam.position.x = x
                            pose_cam.position.y = y
                            pose_cam.position.z = z
                            pose_cam.orientation.w = 1.0
                        else:
                            fail_reason = "Depth too close/invalid"
                    else:
                        fail_reason = "BBox out of bounds"

                if pose_cam:
                    # Transform to Base Frame
                    pose_base = self.transform_pose(pose_cam, self.source_frame, self.target_frame)
                    
                    if pose_base:
                        strategy = BoxStrategy()
                        strategy.id = f"box_{i}"
                        
                        if pose_base.position.z > self.layer_thresh:
                            strategy.layer_index = 1 # Top
                            strategy.strategy = "SIDE_PULL"
                            strategy.approach_pose = pose_base 
                        else:
                            strategy.layer_index = 0 # Bottom
                            strategy.strategy = "TOP_PICK"
                            strategy.approach_pose = pose_base

                        strategies.append(strategy)
                        debug_info.append(f"{strategy.strategy} (L{strategy.layer_index})")
                    else:
                        fail_reason = "TF Error"
                        debug_info.append(fail_reason)
                else:
                    if not fail_reason: fail_reason = "Unknown 3D Fail"
                    debug_info.append(fail_reason)

            # Debug Visualization
            if self.debug_mode:
                self.publish_debug_image(cv_image, detections, debug_info)

        except Exception as e:
            self.get_logger().error(f"Analysis Failed: {e}")
            response.success = False
            
        return response

    def transform_pose(self, pose, source_frame, target_frame):
        try:
            # Manual TF Implementation to avoid tf2_geometry_msgs issues
            transform_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            
            t = transform_stamped.transform.translation
            r = transform_stamped.transform.rotation
            
            # Convert Quaternion to Rotation Matrix (using simple formula to avoid scipy dependency issues if any)
            # R = ...
            # But let's use scipy since we have Open3D which depends on it anyway.
            from scipy.spatial.transform import Rotation as R
            
            q_tf = [r.x, r.y, r.z, r.w]
            rot_tf = R.from_quat(q_tf)
            trans_tf = np.array([t.x, t.y, t.z])
            
            # Source Pose
            q_src = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            rot_src = R.from_quat(q_src)
            trans_src = np.array([pose.position.x, pose.position.y, pose.position.z])
            
            # Apply Transform
            # P_target = T_tf * P_src
            # Orientation: R_target = R_tf * R_src
            # Position: t_target = R_tf * t_src + t_tf
            
            rot_target = rot_tf * rot_src
            trans_target = rot_tf.apply(trans_src) + trans_tf
            
            # Construct Result Pose
            res_pose = Pose()
            res_pose.position.x = trans_target[0]
            res_pose.position.y = trans_target[1]
            res_pose.position.z = trans_target[2]
            
            q_target = rot_target.as_quat()
            res_pose.orientation.x = q_target[0]
            res_pose.orientation.y = q_target[1]
            res_pose.orientation.z = q_target[2]
            res_pose.orientation.w = q_target[3]
            
            return res_pose

        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")
            return None

    def publish_debug_image(self, cv_image, detections, debug_info):
        debug_img = cv_image.copy()
        for i, det in enumerate(detections):
            bbox = [int(x) for x in det['bbox']]
            cv2.rectangle(debug_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            
            info_str = "Unknown"
            if i < len(debug_info):
                info_str = debug_info[i]
            
            label = f"{det['class']} {info_str}"
            cv2.putText(debug_img, label, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        self.debug_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SceneAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
