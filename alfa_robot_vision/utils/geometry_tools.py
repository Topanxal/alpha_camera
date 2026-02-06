import numpy as np
import cv2

try:
    import open3d as o3d
except ImportError:
    # print("Warning: 'open3d' not found. 3D plane fitting will be disabled.")
    o3d = None

class GeometryProcessor:
    def __init__(self, camera_info_msg):
        """
        Initialize the GeometryProcessor with ROS CameraInfo.
        
        Args:
            camera_info_msg (sensor_msgs.msg.CameraInfo): Camera intrinsic parameters.
        """
        # ROS CameraInfo K matrix is flattened [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = camera_info_msg.k[0]
        self.fy = camera_info_msg.k[4]
        self.cx = camera_info_msg.k[2]
        self.cy = camera_info_msg.k[5]

    def pixel_to_3d(self, u, v, depth):
        """
        Simple Pinhole projection for a single point.
        """
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return (x, y, z)

    def process_box(self, depth_image, mask_poly, depth_scale=0.001):
        """
        Process a single detected box/mask to find its 3D pose.
        Fallback to simple averaging if Open3D is not available.
        """
        h, w = depth_image.shape
        
        # 1. Create a binary mask from the polygon
        mask_img = np.zeros((h, w), dtype=np.uint8)
        pts = mask_poly.astype(np.int32)
        cv2.fillPoly(mask_img, [pts], 1)
        
        # 2. Extract valid depth points within the mask
        valid_indices = np.where((mask_img == 1) & (depth_image > 0))
        
        if len(valid_indices[0]) < 20: # Threshold
            return None
            
        v_coords = valid_indices[0] # y
        u_coords = valid_indices[1] # x
        
        # Convert depth to meters
        depth_values = depth_image[v_coords, u_coords].astype(np.float32) * depth_scale
        
        # Filter out noise
        valid_depth_mask = (depth_values > 0.1) & (depth_values < 5.0)
        if np.sum(valid_depth_mask) < 20:
            return None
            
        v_coords = v_coords[valid_depth_mask]
        u_coords = u_coords[valid_depth_mask]
        depth_values = depth_values[valid_depth_mask]

        # 3. Back-project pixels to 3D points
        z = depth_values
        x = (u_coords - self.cx) * z / self.fx
        y = (v_coords - self.cy) * z / self.fy
        
        points = np.stack((x, y, z), axis=-1)

        # --- BRANCH: Open3D Available ---
        if o3d is not None:
            try:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                pcd_down = pcd.voxel_down_sample(voxel_size=0.01)
                
                if len(pcd_down.points) >= 10:
                    plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=50)
                    if len(inliers) >= 10:
                        a, b, c, d = plane_model
                        normal = np.array([a, b, c])
                        if c > 0: normal = -normal # Point to camera
                        
                        inlier_cloud = pcd_down.select_by_index(inliers)
                        centroid = inlier_cloud.get_center()
                        
                        # Yaw calculation
                        yaw = np.degrees(np.arctan2(normal[0], normal[2])) # a, c
                        yaw_adjusted = yaw
                        if yaw_adjusted > 90: yaw_adjusted -= 180
                        elif yaw_adjusted < -90: yaw_adjusted += 180
                        
                        return {
                            "center": centroid,
                            "normal": normal,
                            "yaw": yaw_adjusted,
                            "plane_model": plane_model
                        }
            except Exception as e:
                print(f"Open3D fitting failed: {e}")
                pass

        # --- BRANCH: Fallback (Simple Average) ---
        # If Open3D missing or fitting failed
        
        # Calculate Centroid
        cx = np.mean(x)
        cy = np.mean(y)
        cz = np.mean(z)
        
        # Default Normal (Facing Camera)
        normal = np.array([0.0, 0.0, -1.0])
        yaw = 0.0
        
        return {
            "center": np.array([cx, cy, cz]),
            "normal": normal,
            "yaw": yaw,
            "plane_model": None
        }
