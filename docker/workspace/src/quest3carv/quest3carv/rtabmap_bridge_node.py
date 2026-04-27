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
            '/mapData',
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
            if not node_data.word_id_keys or len(node_data.word_id_keys) == 0:
                continue

            kf_msg = KeyframeData()
            kf_msg.header.stamp = self.get_clock().now().to_msg()
            kf_msg.header.frame_id = "world" # RTAB-Map optimizes into the map/world frame
            
            # 1. Extract Optimized Camera Pose
            kf_msg.camera_pose = node_data.pose

            # 2. Extract Image (If texturing is still needed)
            # RTAB-Map compresses images in NodeData by default to save bandwidth
            if len(node_data.data.left_compressed) > 0:
                try:
                    img_array = np.frombuffer(node_data.data.left_compressed, np.uint8)
                    cv_img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    kf_msg.image = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                except Exception as e:
                    self.get_logger().error(f"Image decode failed: {e}")

            # 3. Extract 3D Metric Points and IDs
            valid_points = 0
            for pt, word_id in zip(node_data.word_pts, node_data.word_id_keys):
                p = Point()
                p.x, p.y, p.z = float(pt.x), float(pt.y), float(pt.z)
                
                kf_msg.points.append(p)
                kf_msg.point_ids.append(int(word_id) & 0xFFFFFFFF)
                valid_points += 1

            if valid_points > 0:
                self.kf_pub.publish(kf_msg)
                self.get_logger().info(f"Keyframe {node_data.id}: Sent {valid_points} metric points to carver.")

def main():
    rclpy.init()
    rclpy.spin(RtabmapBridgeNode())

if __name__ == '__main__':
    main()