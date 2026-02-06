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
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        
        # 1. Distance Control
        # Side Pull: We want to be very close, e.g., 20cm
        target_dist = 0.20 
        
        # Define a central ROI for distance and normal estimation
        roi_size = 40
        roi_poly = np.array([
            [cx-roi_size, cy-roi_size], 
            [cx+roi_size, cy-roi_size], 
            [cx+roi_size, cy+roi_size], 
            [cx-roi_size, cy+roi_size]
        ])
        
        if self.geo_processor:
            geo_res = self.geo_processor.process_box(depth, roi_poly)
            if geo_res:
                feedback.target_locked = True
                
                # Z Distance (using center of ROI)
                current_dist = geo_res['center'][2]
                feedback.distance_to_surface = current_dist
                # Error X/Y (We want ROI center to be at Image Center, which it is by definition of ROI)
                # But in reality, we should track a feature. 
                # For now, assume we are centering on the surface in front.
                feedback.error_x = 0.0
                feedback.error_y = 0.0
                
                # Angle Error (Normal Alignment)
                # Normal is [nx, ny, nz]
                # Camera optical axis is [0, 0, 1] (or -1 depending on frame convention)
                # We want normal to be parallel to Z axis -> nx=0, ny=0
                
                normal = geo_res['normal']
                # Simple approximation for small angles:
                # pitch_err ~ ny
                # yaw_err ~ nx
                feedback.error_angle_pitch = float(normal[1])
                feedback.error_angle_yaw = float(normal[0])
                
                # Draw Info
                cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(img, f"Dist: {current_dist:.3f}m", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Draw Normal (Projected)
                p1 = (cx, cy)
                p2 = (int(cx + normal[0]*100), int(cy + normal[1]*100))
                cv2.arrowedLine(img, p1, p2, (0, 0, 255), 2)
            else:
                feedback.target_locked = False
                cv2.putText(img, "No Depth/Surface", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    def process_top_align(self, img, depth, feedback):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        
        # Top Pick Strategy: Patch-based Grasping
        # Search 3x3 grid around center to find flattest surface
        
        grid_size = 60 # pixels
        best_score = float('inf')
        best_patch = None
        
        # Offsets: -1, 0, 1
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                # Patch Center
                px = cx + i * grid_size
                py = cy + j * grid_size
                
                # Extract Depth ROI
                d_roi = depth[py-20:py+20, px-20:px+20]
                if d_roi.size == 0: continue
                
                # Filter valid depth
                valid_d = d_roi[(d_roi > 100) & (d_roi < 2000)] # 0.1m to 2.0m
                
                if len(valid_d) > 100:
                    # Score = Variance (Lower is better/flatter)
                    score = np.std(valid_d)
                    
                    # Penalize distance from center (We prefer center grasp)
                    dist_penalty = (abs(i) + abs(j)) * 5.0 # Weight
                    final_score = score + dist_penalty
                    
                    if final_score < best_score:
                        best_score = final_score
                        best_patch = (px, py)
        
        if best_patch:
            feedback.target_locked = True
            
            # Calculate Errors relative to Image Center
            # We want to move camera so that Image Center aligns with Best Patch
            # Error = Target - Current
            # Current = Image Center
            # Target = Best Patch
            # But in visual servoing: error = feature_coord - principal_point
            
            feedback.error_x = float(best_patch[0] - cx) # Pixel error
            feedback.error_y = float(best_patch[1] - cy)
            
            # Get Depth at Best Patch
            d_val = depth[best_patch[1], best_patch[0]] * 0.001
            feedback.distance_to_surface = d_val
            
            # Draw
            cv2.circle(img, best_patch, 8, (255, 0, 0), -1) # Blue dot for target
            cv2.line(img, (cx, cy), best_patch, (255, 255, 0), 1)
            cv2.putText(img, "Best Patch", best_patch, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        else:
            feedback.target_locked = False
            cv2.putText(img, "Scanning...", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
        # Draw Crosshair
        cv2.line(img, (cx-10, cy), (cx+10, cy), (0, 255, 0), 1)
        cv2.line(img, (cx, cy-10), (cx, cy+10), (0, 255, 0), 1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
