import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import MapData
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys

from geometry_msgs.msg import Point
from quest3carv_interfaces.msg import KeyframeData 

class RtabmapBridgeNode(Node):
    def __init__(self):
        super().__init__('rtabmap_bridge')
        self.bridge = CvBridge()
        self.last_processed_node_id = -1
        
        # Publisher to the existing C++ Carving Node
        self.kf_pub = self.create_publisher(KeyframeData, 'quest3carv/keyframe', 10)
        
        # Subscribe to RTAB-Map's dense graph output
        self.sub_map = self.create_subscription(
            MapData,
            '/rtabmap/mapData',
            self.map_data_callback,
            10
        )
        self.get_logger().info("RTAB-Map to CARV Bridge Node Started.")

    def map_data_callback(self, msg: MapData):
        # RTAB-Map sends the graph; we only care about newly added nodes (keyframes)
        for node_data in msg.nodes:
            if node_data.id <= self.last_processed_node_id:
                continue
                
            self.last_processed_node_id = node_data.id
            
            # Ensure the node contains valid tracked features
            if not node_data.wordIds or len(node_data.wordIds) == 0:
                continue

            kf_msg = KeyframeData()
            kf_msg.header.stamp = self.get_clock().now().to_msg()
            kf_msg.header.frame_id = "world" # RTAB-Map optimizes into the map/world frame
            
            # 1. Extract Optimized Camera Pose
            kf_msg.camera_pose = node_data.pose

            # 2. Extract Image (If texturing is still needed)
            # RTAB-Map compresses images in NodeData by default to save bandwidth
            if len(node_data.image) > 0:
                try:
                    cv_img = self.bridge.compressed_imgmsg_to_cv2(node_data.image, "bgr8")
                    
                    # --- DEBUG OVERLAY ---
                    overlay_text = f"KF: {node_data.id} | Points: {len(node_data.wordIds)}"
                    cv2.putText(cv_img, overlay_text, (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    cv2.imshow("Bridge Debug Output", cv_img)
                    cv2.waitKey(1)
                    # ---------------------
                    
                    kf_msg.image = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                except Exception as e:
                    self.get_logger().error(f"Image decode failed: {e}")

            # 3. Extract 3D Metric Points and IDs
            # wordPts is a PointCloud2 aligned exactly with the integer array wordIds
            points_generator = point_cloud2.read_points(node_data.wordPts, field_names=("x", "y", "z"), skip_nans=True)
            
            valid_points = 0
            for point, word_id in zip(points_generator, node_data.wordIds):
                p = Point()
                p.x, p.y, p.z = float(point[0]), float(point[1]), float(point[2])
                
                kf_msg.points.append(p)
                kf_msg.point_ids.append(int(word_id))
                valid_points += 1

            if valid_points > 0:
                self.kf_pub.publish(kf_msg)
                self.get_logger().info(f"Keyframe {node_data.id}: Sent {valid_points} metric points to carver.")

def main():
    rclpy.init()
    rclpy.spin(RtabmapBridgeNode())

if __name__ == '__main__':
    main()