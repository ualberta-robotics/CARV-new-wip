import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import os
import collections

from quest3carv.tracker import StereoPointTracker

from geometry_msgs.msg import Point
from quest3carv_interfaces.msg import KeyframeData # Our new custom message

class SpatialReconstructionNode(Node):
    def __init__(self):
        super().__init__('spatial_recon')
        self.bridge = CvBridge()
        
        # Configuration for Keyframe Debugging
        self.save_kf_images = True
        self.show_kf_images = True
        self.save_ply_clouds = False
        self.output_dir = "/workspace/debug/clouds"
        if self.save_ply_clouds and not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # ==========================================================
        # --- PROJECT ARIA FISHEYE624 CALIBRATION ---
        # ==========================================================
        
        self.aria_left_params = np.array([
            2.45950108e+02, 2.54271173e+02, 2.54760927e+02, 3.72268602e-02,
           -5.18844382e-02, 8.34567042e-02, -9.76545331e-02, 4.11587224e-02,
           -6.17269189e-03, -3.22907861e-04, -1.49661012e-03, 6.26109463e-04,
            3.06379489e-04, 1.55605608e-03, -2.63267357e-04
        ])
        
        self.aria_right_params = np.array([
            2.46631443e+02, 2.53178304e+02, 2.55425239e+02, 2.99043970e-02,
           -3.35379923e-02, 5.12976316e-02, -6.40272088e-02, 2.33274370e-02,
           -2.53216762e-03, -2.39597559e-03, -5.96400095e-04, 2.57583573e-03,
            2.43233388e-04, 7.54034682e-04, -3.12675361e-05
        ])

        # Right Camera Pose relative to Left
        T_right = np.array([0.12641746, 0.01815863, -0.04207057])
        R_right = np.array([
            [ 0.77033105, -0.26845385,  0.57837929],
            [ 0.26786756,  0.95937992,  0.0885276 ],
            [-0.57865105,  0.08673349,  0.81095022]
        ])

        # Baseline is the Euclidean distance of the translation vector
        self.true_baseline = np.linalg.norm(T_right) 

        # Define your ideal Virtual Pinhole Cameras (P1, P2) 
        # Using an average focal length and centered principal point for a 512x512 frame
        ideal_f = 246.0 
        self.P1 = np.array([[ideal_f, 0., 256., 0.],
                            [0., ideal_f, 256., 0.],
                            [0., 0., 1., 0.]])
        
        self.P2 = np.array([[ideal_f, 0., 256., -ideal_f * self.true_baseline],
                            [0., ideal_f, 256., 0.],
                            [0., 0., 1., 0.]])

        self.R1 = np.eye(3) 
        self.R2 = R_right

        # Generate the warping maps using the custom Aria mathematical model at 512x512
        self.map_l_x, self.map_l_y = self.generate_aria_rectification_maps(
            self.aria_left_params, self.R1, self.P1, image_size=(512, 512)
        )
        self.map_r_x, self.map_r_y = self.generate_aria_rectification_maps(
            self.aria_right_params, self.R2, self.P2, image_size=(512, 512)
        )

        rectified_K = self.P1[:3, :3] 
        self.tracker = StereoPointTracker(rectified_K, baseline=self.true_baseline, logger=self.get_logger())
        # ==========================================================
        # ==========================================================

        # Keyframe Logic State
        self.last_kf_pose = None
        self.keyframes = [] # List of (Image, Pose, Points)
        self.dist_threshold = 0.02 # cm
        self.rot_threshold = 2.0  # degrees
        self.kf_observation_counts = collections.defaultdict(int)
        self.min_kf_observations = 3  # The Drastic Measure

        # Keyframe Publisher
        self.kf_pub = self.create_publisher(KeyframeData, 'quest3carv/keyframe', 10)

        # Synchronized Subscribers
        self.sub_l = message_filters.Subscriber(self, Image, 'quest3/left/raw')
        self.sub_r = message_filters.Subscriber(self, Image, 'quest3/right/raw')
        self.sub_p = message_filters.Subscriber(self, PoseStamped, 'quest3/pose')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_l, self.sub_r, self.sub_p], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.process_bundle)

    def generate_aria_rectification_maps(self, aria_params, R_rect, P_rect, image_size=(512, 512)):
        """
        Generates OpenCV-compatible map_x and map_y for cv2.remap() using 
        Project Aria's FisheyeRadTanThinPrism (Fisheye624) model.
        """
        f = aria_params[0]
        cx, cy = aria_params[1], aria_params[2]
        k = aria_params[3:9]
        p = aria_params[9:11]
        s = aria_params[11:15]

        h, w = image_size
        u_rect, v_rect = np.meshgrid(np.arange(w), np.arange(h))

        # 1. Unproject virtual rectified pixels to 3D rays (Linear Model)
        fx_rect, fy_rect = P_rect[0, 0], P_rect[1, 1]
        cx_rect, cy_rect = P_rect[0, 2], P_rect[1, 2]

        x_c = (u_rect - cx_rect) / fx_rect
        y_c = (v_rect - cy_rect) / fy_rect
        z_c = np.ones_like(x_c)
        
        rays_rect = np.stack((x_c, y_c, z_c), axis=-1)

        # 2. Apply inverse rectification rotation to get to the physical camera frame
        rays_phys = np.einsum('ij,hwj->hwi', R_rect.T, rays_rect)
        x, y, z = rays_phys[..., 0], rays_phys[..., 1], rays_phys[..., 2]

        # 3. Convert to Polar Coordinates
        r_xy = np.sqrt(x**2 + y**2)
        theta = np.arctan2(r_xy, z)
        
        safe_r_xy = np.where(r_xy == 0, 1e-8, r_xy)
        cos_phi = x / safe_r_xy
        sin_phi = y / safe_r_xy

        # 4. Radial Distortion
        theta2 = theta**2
        r_theta = theta + k[0]*theta**3 + k[1]*theta**5 + k[2]*theta**7 + k[3]*theta**9 + k[4]*theta**11 + k[5]*theta**13
        
        # Base polar projections
        u_r = r_theta * cos_phi
        v_r = r_theta * sin_phi

        # 5. Tangential Distortion
        r_theta2 = r_theta**2
        t_x = p[0]*(2*u_r**2 + r_theta2) + 2*p[1]*u_r*v_r
        t_y = p[1]*(2*v_r**2 + r_theta2) + 2*p[0]*u_r*v_r

        # 6. Thin-Prism Distortion
        tp_x = s[0]*r_theta2 + s[1]*(r_theta2**2)
        tp_y = s[2]*r_theta2 + s[3]*(r_theta2**2)

        # 7. Final Distorted Pixel Location
        u_dist = f * (u_r + t_x + tp_x) + cx
        v_dist = f * (v_r + t_y + tp_y) + cy

        return u_dist.astype(np.float32), v_dist.astype(np.float32)

    def visualize_keyframe(self, img, points_3d, points_2d, ids_3d, camera_pose):
        """
        Processes an annotated keyframe image with point distances
        Can optionally save to disk or display live via OpenCV.
        """
        # Fast exit if we aren't displaying or saving
        if not self.save_kf_images and not self.show_kf_images:
            return
        
        # Extract the camera's translation vector (x, y, z) in world space
        camera_pos = camera_pose[:3, 3]

        # Upscale factor for better text resolution
        scale = 1
        h, w = img.shape[:2]
        debug_img = cv2.resize(img, (int(w * scale), int(h * scale)))

        for i in range(len(points_2d)):
            pt = (int(points_2d[i][0] * scale), int(points_2d[i][1] * scale))
            
            # Calculate Euclidean distance from the camera
            dist = np.linalg.norm(points_3d[i] - camera_pos)
            pid = int(ids_3d[i])
            
            # Draw point marker
            cv2.circle(debug_img, pt, 4, (0, 255, 0), -1)
            
            # Format label: "Dist: X.Xm | Age: Y"
            label = f"{pid} | {dist:.2f}m"
            cv2.putText(debug_img, label, (pt[0] + 5, pt[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        if self.save_kf_images:
            kf_idx = len(self.keyframes)
            os.makedirs("/workspace/debug/keyframes/", exist_ok=True)
            filename = f"/workspace/debug/keyframes/keyframe_{kf_idx:03d}.jpg"
            cv2.imwrite(filename, debug_img)
            #self.get_logger().info(f"Saved debug keyframe: {filename}")

        if self.show_kf_images:
            cv2.imshow("Keyframe Triggered", debug_img)
            cv2.waitKey(1)  # Required to pump OpenCV GUI events

    def save_global_ply(self):
        """
        Gathers all keyframe points and camera positions, 
        and saves a colored PLY file.
        """
        if not self.keyframes:
            return

        world_points = []
        colors = []

        for kf in self.keyframes:
            pose = kf['pose']        
            pts_world = kf['points'] # THESE ARE ALREADY WORLD POINTS!

            # 1. Add Camera Position (Red)
            cam_pos = pose[:3, 3]
            world_points.append(cam_pos)
            colors.append((255, 0, 0)) # Red for camera

            # 2. Add Points (Blue)
            for pt in pts_world:
                world_points.append(pt)
                colors.append((0, 0, 255)) 

        # Write to PLY file
        kf_idx = len(self.keyframes)
        filepath = os.path.join(self.output_dir, f"cloud_kf_{kf_idx:03d}.ply")
        
        with open(filepath, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(world_points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            
            for p, c in zip(world_points, colors):
                f.write(f"{p[0]} {p[1]} {p[2]} {c[0]} {c[1]} {c[2]}\n")
        
        #self.get_logger().info(f"Saved global cloud to {filepath}")

    def is_significant_move(self, current_pose_msg):
        if self.last_kf_pose is None: return True
        
        # 1. Translation check
        curr_p = np.array([current_pose_msg.position.x, current_pose_msg.position.y, current_pose_msg.position.z])
        last_p = np.array([self.last_kf_pose.position.x, self.last_kf_pose.position.y, self.last_kf_pose.position.z])
        if np.linalg.norm(curr_p - last_p) > self.dist_threshold:
            return True
            
        # 2. Rotation check
        curr_q = [current_pose_msg.orientation.x, current_pose_msg.orientation.y, current_pose_msg.orientation.z, current_pose_msg.orientation.w]
        last_q = [self.last_kf_pose.orientation.x, self.last_kf_pose.orientation.y, self.last_kf_pose.orientation.z, self.last_kf_pose.orientation.w]
        
        r_curr = R.from_quat(curr_q)
        r_last = R.from_quat(last_q)
        relative_rot = r_curr.inv() * r_last
        if relative_rot.magnitude() > np.radians(self.rot_threshold):
            return True
            
        return False

    def process_bundle(self, msg_l, msg_r, msg_p):
        img_l = self.bridge.imgmsg_to_cv2(msg_l, "bgr8")
        img_r = self.bridge.imgmsg_to_cv2(msg_r, "bgr8")
        
        # --- NEW: Iron out distortion AND mathematically parallelize the cameras ---
        img_l = cv2.remap(img_l, self.map_l_x, self.map_l_y, cv2.INTER_LINEAR)
        img_r = cv2.remap(img_r, self.map_r_x, self.map_r_y, cv2.INTER_LINEAR)
        
        # Convert PoseStamped to 4x4 for the tracker
        curr_q = [msg_p.pose.orientation.x, msg_p.pose.orientation.y, msg_p.pose.orientation.z, msg_p.pose.orientation.w]
        curr_t = [msg_p.pose.position.x, msg_p.pose.position.y, msg_p.pose.position.z]
        
        head_mat = np.eye(4)
        head_mat[:3, :3] = R.from_quat(curr_q).as_matrix()
        head_mat[:3, 3] = curr_t
        
        # --- NEW: Extrinsic Offset (Head to Left Camera) ---
        # Shifts the origin ~32mm Left, ~15mm Down, ~30mm Forward (where the camera is more or less relative to the head)
        T_head_to_cam = np.eye(4)
        T_head_to_cam[0, 3] = -0.032  # Left
        T_head_to_cam[1, 3] = -0.015  # Down (OpenXR Y is Up)
        T_head_to_cam[2, 3] = -0.030  # FIX: OpenXR -Z is Forward!
        
        # Apply the offset in the local frame
        mat = head_mat @ T_head_to_cam
        
        # FIX: Remove the following two lines from your old code!
        # They were erasing the translation offset and rotation you just calculated.
        # mat[:3, :3] = R.from_quat(curr_q).as_matrix() 
        # mat[:3, 3] = curr_t
        
        # FIX: Apply the stereo rectification rotation (R1)
        # We need the pose of the RECTIFIED camera in world space, not the physical one.
        mat[:3, :3] = mat[:3, :3] @ self.R1.T
        
        # Step A: Ingest (Transient Tracking)
        self.tracker.ingest_frame(img_l, img_r, mat)
        
        # Step B: Keyframe Logic (Mapping)
        if self.is_significant_move(msg_p.pose):
            self.last_kf_pose = msg_p.pose
            
            # Extract high-confidence points visible from this keyframe
            points_3d, points_2d, ids_3d = self.tracker.get_confident_points()

            n_pts = len(points_3d)

            for i, pt in enumerate(points_3d):
                # Point in a keyframe
                pid = int(ids_3d[i])
                self.kf_observation_counts[pid] += 1

            if (n_pts < 1):
                #self.get_logger().info(f"No points in this keyframe")
                return
            
            # Call the updated visualization functions
            self.visualize_keyframe(img_l, points_3d, points_2d, ids_3d, mat)
            if self.save_ply_clouds:
                self.save_global_ply()
            
            # Proceed with adding to keyframes list
            self.keyframes.append({
                'image': img_l,
                'pose': mat,
                'points': points_3d
            })
            
            # ROS 2 Bridge: Send to C++ Carving Node
            kf_msg = KeyframeData()
            kf_msg.header.stamp = self.get_clock().now().to_msg()
            kf_msg.header.frame_id = "world"
            kf_msg.image = self.bridge.cv2_to_imgmsg(img_l, encoding="bgr8")
            
            # --- NEW: Send the OFFSET camera pose, not the raw head pose ---
            kf_msg.camera_pose.position.x = float(mat[0, 3])
            kf_msg.camera_pose.position.y = float(mat[1, 3])
            kf_msg.camera_pose.position.z = float(mat[2, 3])
            
            # Convert the 3x3 rotation matrix back to a quaternion
            offset_q = R.from_matrix(mat[:3, :3]).as_quat()
            kf_msg.camera_pose.orientation.x = float(offset_q[0])
            kf_msg.camera_pose.orientation.y = float(offset_q[1])
            kf_msg.camera_pose.orientation.z = float(offset_q[2])
            kf_msg.camera_pose.orientation.w = float(offset_q[3])
            
            cam_pos = np.array([msg_p.pose.position.x, msg_p.pose.position.y, msg_p.pose.position.z])

            # --- PREPARE FRUSTUM CULLING MATRICES ---
            T_w2c = np.linalg.inv(mat)
            K = self.P1[:3, :3]
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            
            # Use the actual camera origin for distance, not the raw head pose
            cam_pos = mat[:3, 3] 

            culled_dist = 0
            culled_frustum = 0
            culled_age = 0

            actual_pts = 0
            
            first_fail_logged = False

            for i, pt in enumerate(points_3d):
                dist = np.linalg.norm(pt - cam_pos) 
                
                if 0.2 < dist < 3.5:
                    # Project world point into this camera's local frame
                    p_cam = T_w2c[:3, :3] @ pt + T_w2c[:3, 3]
                    
                    # Strictly ensure the point is in front of the camera
                    if p_cam[2] < -0.1: 
                        # Standard projection without the abs() hack
                        u = (fx * p_cam[0] / -p_cam[2]) + cx
                        v = (fy * p_cam[1] / -p_cam[2]) + cy
                        
                        # 10-pixel safety margin
                        if 10 <= u < 502 and 10 <= v < 502:
                            p = Point()
                            p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
                            kf_msg.points.append(p)
                            kf_msg.point_ids.append(int(ids_3d[i]))
                            actual_pts += 1
                        else:
                            culled_frustum += 1
                            if not first_fail_logged:
                                self.get_logger().warn(f"CULL REASON (Frustum): 3D Local={p_cam}, Projected 2D=(u:{u:.1f}, v:{v:.1f})")
                                first_fail_logged = True
                else:
                    culled_dist += 1
                    if not first_fail_logged:
                        self.get_logger().warn(f"CULL REASON (Distance): {dist:.2f} meters")
                        first_fail_logged = True
            
            if actual_pts == 0:
                self.get_logger().info(f"All pts filtered out")
                return
                
            self.kf_pub.publish(kf_msg)
            self.get_logger().info(f"Keyframe: Sent {len(kf_msg.points)} | Culled Dist: {culled_dist} | Culled Frustum: {culled_frustum}| Culled Age: {culled_age}")

def main():
    rclpy.init()
    rclpy.spin(SpatialReconstructionNode())

if __name__ == '__main__':
    main()