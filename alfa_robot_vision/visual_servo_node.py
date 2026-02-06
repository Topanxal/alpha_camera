import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from logistics_interfaces.msg import ServoFeedback
from logistics_interfaces.srv import SetServoMode
from .utils.geometry_tools import GeometryProcessor

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug_mode', False)
            ]
        )
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        
        self.current_mode = "IDLE" # IDLE, FACE_ALIGN, TOP_ALIGN
        
        # Comm
        self.srv = self.create_service(SetServoMode, 'set_servo_mode', self.handle_set_mode)
        self.feedback_pub = self.create_publisher(ServoFeedback, 'servo_feedback', 10)
        self.debug_pub = self.create_publisher(Image, 'servo_debug_image', 10)
        
        # Subs
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        self.geo_processor = None
        
        self.create_subscription(Image, '/head_camera/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/head_camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/head_camera/camera_info', self.info_callback, 10)
        
        self.create_timer(0.05, self.control_loop) # 20Hz
        
        self.get_logger().info('Visual Servo Node Initialized.')

    def rgb_callback(self, msg):
        self.latest_rgb = msg

    def depth_callback(self, msg):
        self.latest_depth = msg
        
    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.geo_processor = GeometryProcessor(msg)

    def handle_set_mode(self, request, response):
        if request.mode in ["IDLE", "FACE_ALIGN", "TOP_ALIGN"]:
            self.current_mode = request.mode
            response.success = True
            self.get_logger().info(f"Servo Mode set to: {self.current_mode}")
        else:
            response.success = False
            self.get_logger().warn(f"Invalid mode: {request.mode}")
        return response

    def control_loop(self):
        if self.current_mode == "IDLE":
            return
            
        if self.latest_rgb is None or self.latest_depth is None:
            return

        feedback = ServoFeedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.current_mode = self.current_mode
        feedback.target_locked = False
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
            
            if self.current_mode == "FACE_ALIGN":
                self.process_face_align(cv_img, cv_depth, feedback)
            elif self.current_mode == "TOP_ALIGN":
                self.process_top_align(cv_img, cv_depth, feedback)
                
            self.feedback_pub.publish(feedback)
            
            if self.debug_mode:
                debug_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f"Servo Loop Error: {e}")

    def process_face_align(self, img, depth, feedback):
        # 1. Simple Blob/Contour detection to find the closest object center
        # For simplicity, let's assume we are looking at the center of the image
        # In a real scenario, we would use YOLO tracking or simple color/depth segmentation
        
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        
        # Check depth at center to see if we are looking at something
        d_val = depth[cy, cx] * 0.001
        
        if d_val > 0.1 and d_val < 2.0:
            feedback.target_locked = True
            feedback.distance_to_surface = d_val
            
            # Error is 0 because we define the target as what's in front (Visual Servoing usually minimizes error to a target feature)
            # Actually, we want to align to a specific box. 
            # Simplified Logic: Assume the robot is roughly pointing at the box.
            # We calculate the surface normal at the center.
            
            # Use GeometryProcessor to get normal
            # Define a small ROI around center
            roi_poly = np.array([[cx-20, cy-20], [cx+20, cy-20], [cx+20, cy+20], [cx-20, cy+20]])
            if self.geo_processor:
                geo_res = self.geo_processor.process_box(depth, roi_poly)
                if geo_res:
                    # Normal vector (nx, ny, nz)
                    normal = geo_res['normal'] 
                    # Calculate Pitch/Yaw error relative to Camera Z-axis (0, 0, 1)
                    # Ideal normal should be (0, 0, -1) pointing towards camera? Or (0,0,1)?
                    # Surface normal usually points OUT of the surface. Camera looks down -Z (or +Z depending on frame).
                    # Let's assume ideal normal is parallel to optical axis.
                    
                    # Simple angle calculation
                    # ... (Implementation details for angle error)
                    feedback.error_angle_pitch = 0.0 # Placeholder
                    feedback.error_angle_yaw = 0.0   # Placeholder
            
            cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)

    def process_top_align(self, img, depth, feedback):
        # Top pick alignment logic
        # Detect rectangle, align orientation
        pass

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
