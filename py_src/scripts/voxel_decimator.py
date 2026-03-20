import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import open3d as o3d

class VoxelDecimatorNode(Node):
    def __init__(self):
        super().__init__('voxel_decimator_node')
        
        # 1. Declare the resolution of your grid
        # 0.05 means 5 cm cubes. You can change this on the fly!
        self.declare_parameter('voxel_size', 0.05)
        
        # 2. Setup Publisher and Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pcloud/events',
            self.pc_callback,
            10)
            
        self.publisher = self.create_publisher(
            PointCloud2, 
            '/pcloud/voxelized', 
            10)
            
        self.get_logger().info("Smart Decimator active! Waiting for raw events...")

    def pc_callback(self, msg):
        # Fetch the live voxel size parameter
        voxel_size = self.get_parameter('voxel_size').value

        # Step 1: Extract raw points safely
        if msg.width == 0 or msg.height == 0:
            return

        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        point_list = [[p[0], p[1], p[2]] for p in points_gen]

        if not point_list:
            return

        raw_points = np.array(point_list, dtype=np.float64)
        original_count = len(raw_points)

        # Step 2: The Open3D Magic (Voxel Grid Downsampling)
        # We load the numpy array into an Open3D point cloud object
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(raw_points)
        
        # Apply the voxel filter
        clean_cloud, ind = o3d_cloud.remove_radius_outlier(nb_points=4, radius=0.05)
        downsampled_cloud = clean_cloud.voxel_down_sample(voxel_size=voxel_size)
        
        # Extract the surviving points back to a numpy array
        filtered_points = np.asarray(downsampled_cloud.points)
        new_count = len(filtered_points)

        if new_count == 0:
            return

        # Step 3: Publish the clean data back to ROS
        # CRITICAL: We copy the exact timestamp and frame_id from the incoming message
        # so the TF tree knows exactly where and when this data belongs!
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        # Convert back to a ROS PointCloud2 message
        clean_msg = pc2.create_cloud_xyz32(header, filtered_points)
        self.publisher.publish(clean_msg)

        # Calculate compression ratio for the logs
        reduction = (1.0 - (new_count / original_count)) * 100
        self.get_logger().debug(
            f"Decimated: {original_count} pts -> {new_count} pts ({reduction:.1f}% reduction) | Voxel: {voxel_size}m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VoxelDecimatorNode()
    
    # We use a slightly different spin method here to keep the node fast
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()