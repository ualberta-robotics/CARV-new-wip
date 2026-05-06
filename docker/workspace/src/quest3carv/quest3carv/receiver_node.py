import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
from cv_bridge import CvBridge
import queue
import time

# Aria Imports
import aria.sdk_gen2 as sdk_gen2
import aria.stream_receiver as receiver
from projectaria_tools.core.sensor_data import ImageData, ImageDataRecord, FrontendOutput

class UnifiedAriaRosNode(Node):
    def __init__(self, profile_name="profile9"):
        super().__init__('quest3_unified_receiver')
        self.bridge = CvBridge()
        
        # ROS Publishers
        self.left_pub = self.create_publisher(Image, 'quest3/left/raw', 10)
        self.right_pub = self.create_publisher(Image, 'quest3/right/raw', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'quest3/pose', 10)

        # Thread-safe queues (No multiprocessing needed!)
        self.left_q = queue.Queue(maxsize=2)
        self.right_q = queue.Queue(maxsize=2)
        self.pose_q = queue.Queue(maxsize=5)

        # 1. Setup Aria Device
        self.device = self.setup_device(profile_name)
        
        # 2. Setup Aria Receiver & Callbacks
        self.stream_receiver = self.setup_receiver()

        # 3. Start ROS Polling Timer
        self.create_timer(1/30.0, self.poll_and_publish)
        self.get_logger().info("Unified Aria ROS2 Node Started")

    def setup_device(self, profile_name):
        device_client = sdk_gen2.DeviceClient()
        config = sdk_gen2.DeviceClientConfig()
        device_client.set_client_config(config)
        device = device_client.connect()

        streaming_config = sdk_gen2.HttpStreamingConfig()
        streaming_config.profile_name = profile_name
        streaming_config.streaming_interface = sdk_gen2.StreamingInterface.USB_NCM
        device.set_streaming_config(streaming_config)
        device.start_streaming()
        return device

    def setup_receiver(self):
        config = sdk_gen2.HttpServerConfig()
        config.address = "0.0.0.0"
        config.port = 6768

        stream_receiver = receiver.StreamReceiver(
            enable_image_decoding=True, enable_raw_stream=False
        )
        stream_receiver.set_server_config(config)

        # Register direct class methods as callbacks
        stream_receiver.register_slam_callback(self.image_callback)
        stream_receiver.register_vio_callback(self.vio_callback)
        stream_receiver.start_server()
        return stream_receiver

    # --- ARIA CALLBACKS (Run on background SDK threads) ---
    def image_callback(self, image_data: ImageData, image_record: ImageDataRecord):
        cam_id_str = str(image_record.camera_id).lower()
        img_array = image_data.to_numpy_array().copy() # Copy out of C++ memory
        
        try:
            if 'right' in cam_id_str or '2' in cam_id_str:
                self.right_q.put_nowait(img_array)
            elif 'left' in cam_id_str or '1' in cam_id_str:
                self.left_q.put_nowait(img_array)
        except queue.Full:
            pass # Drop frame if ROS can't keep up

    def vio_callback(self, vio_data: FrontendOutput):
        ts = vio_data.capture_timestamp_ns
        t = np.array(vio_data.transform_odometry_bodyimu.translation()).flatten()
        rotation = vio_data.transform_odometry_bodyimu.rotation()
        
        try:
            quat = rotation.toQuat() 
            qx, qy, qz, qw = quat.x(), quat.y(), quat.z(), quat.w()
        except AttributeError:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0 

        try:
            self.pose_q.put_nowait((ts, t[0], t[1], t[2], qx, qy, qz, qw))
        except queue.Full:
            pass

    # --- ROS PUBLISHER (Runs on ROS Main Thread) ---
    def poll_and_publish(self):
        now = self.get_clock().now().to_msg()

        # Handle Left Image
        if not self.left_q.empty():
            l_img = self.left_q.get()
            msg = self.bridge.cv2_to_imgmsg(l_img, "mono8") # Assuming grayscale based on your encoding code
            msg.header.stamp = now
            msg.header.frame_id = "quest3_left"
            self.left_pub.publish(msg)

        # Handle Right Image
        if not self.right_q.empty():
            r_img = self.right_q.get()
            msg = self.bridge.cv2_to_imgmsg(r_img, "mono8")
            msg.header.stamp = now
            msg.header.frame_id = "quest3_right"
            self.right_pub.publish(msg)

        # Handle Poses
        while not self.pose_q.empty():
            lp = self.pose_q.get()
            p_msg = PoseStamped()
            p_msg.header.stamp = now # Or calculate ROS time from lp[0] (timestamp)
            p_msg.header.frame_id = "world"
            p_msg.pose.position.x, p_msg.pose.position.y, p_msg.pose.position.z = lp[1], lp[2], lp[3]
            p_msg.pose.orientation.x, p_msg.pose.orientation.y, p_msg.pose.orientation.z, p_msg.pose.orientation.w = lp[4], lp[5], lp[6], lp[7]
            self.pose_pub.publish(p_msg)

    def cleanup(self):
        self.device.stop_streaming()
        time.sleep(0.5)
        self.stream_receiver.stop_server()


def main():
    rclpy.init()
    node = UnifiedAriaRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down SDK...")
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()