import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PCACalculatorNode(Node):
    def __init__(self):
        super().__init__('pca_calculator_node')
        
        # 1. Declare and get ROI parameters
        self.declare_parameter('x_min', -0.5)
        self.declare_parameter('x_max', 0.5)
        self.declare_parameter('y_min', 0.5)
        self.declare_parameter('y_max', 1.5)
        self.declare_parameter('z_min', -0.5)
        self.declare_parameter('z_max', 0.5)
        self.declare_parameter('n_frames', 20)  # <-- New parameter for averaging
        
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.n_frames = self.get_parameter('n_frames').value

        # 2. Setup Accumulators for averaging
        self.frame_count = 0
        self.sum_dim1 = 0.0
        self.sum_dim2 = 0.0
        self.sum_dim3 = 0.0
        self.sum_pts = 0
        self.axis1_votes = {"X": 0, "Y": 0, "Z": 0}
        self.axis2_votes = {"X": 0, "Y": 0, "Z": 0}

        # 3. Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pcloud/events',
            self.pc_callback,
            10)
            
        self.get_logger().info(f"PCA Evaluator initialized. Averaging every {self.n_frames} frames...")

    def pc_callback(self, msg):
        # Step 1: Safely extract points
        if msg.width == 0 or msg.height == 0:
            return

        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        point_list = [[p[0], p[1], p[2]] for p in points_gen]

        if not point_list:
            return

        xyz = np.array(point_list, dtype=np.float64)

        # Step 2: Apply the Region of Interest (ROI) Mask
        mask = (
            (xyz[:, 0] >= self.x_min) & (xyz[:, 0] <= self.x_max) &
            (xyz[:, 1] >= self.y_min) & (xyz[:, 1] <= self.y_max) &
            (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max)
        )
        
        roi_points = xyz[mask]
        
        if len(roi_points) < 25:
            return

        # 1. Compute Covariance and Eigen-decomposition
        mean = np.mean(roi_points, axis=0)
        centered_points = roi_points - mean
        cov_matrix = np.cov(centered_points, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Sort descending (l1 > l2 > l3)
        idx = eigenvalues.argsort()[::-1]
        val = eigenvalues[idx]
        vec = eigenvectors[:, idx]

        # 2. Convert Eigenvalues to Physical Dimensions (m -> cm)
        dim1 = np.sqrt(12 * val[0]) * 100
        dim2 = np.sqrt(12 * val[1]) * 100
        dim3 = np.sqrt(12 * val[2]) * 100

        # 3. Determine Orientations
        dir1_idx = np.argmax(np.abs(vec[:, 0]))
        dir2_idx = np.argmax(np.abs(vec[:, 1]))
        
        axis_names = ["X", "Y", "Z"]
        primary_axis = axis_names[dir1_idx]
        secondary_axis = axis_names[dir2_idx]

        # 4. ACCUMULATE THE DATA
        self.sum_dim1 += dim1
        self.sum_dim2 += dim2
        self.sum_dim3 += dim3
        self.sum_pts += len(roi_points)
        self.axis1_votes[primary_axis] += 1
        self.axis2_votes[secondary_axis] += 1
        
        self.frame_count += 1

        # 5. PRINT AVERAGES IF BATCH IS COMPLETE
        if self.frame_count >= self.n_frames:
            # Calculate averages
            avg_dim1 = self.sum_dim1 / self.n_frames
            avg_dim2 = self.sum_dim2 / self.n_frames
            avg_dim3 = self.sum_dim3 / self.n_frames
            avg_pts = int(self.sum_pts / self.n_frames)
            
            # Find the most consistent axes via "majority vote"
            best_axis1 = max(self.axis1_votes, key=self.axis1_votes.get)
            best_axis2 = max(self.axis2_votes, key=self.axis2_votes.get)

            # Print results
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"AVERAGED DIMENSIONS OVER {self.n_frames} FRAMES (Avg Pts: {avg_pts})")
            self.get_logger().info(f"AVG LENGTH: {avg_dim1:5.1f} cm (mostly {best_axis1}-axis)")
            self.get_logger().info(f"AVG DEPTH:  {avg_dim2:5.1f} cm (mostly {best_axis2}-axis) <-- AVG BLUR")
            self.get_logger().info(f"AVG WIDTH:  {avg_dim3:5.1f} cm (thickness/noise)")

            if avg_dim2 > 2.0:
                self.get_logger().warn(f"Significant Avg Blur (> 2cm) detected on {best_axis2}-axis!")
            self.get_logger().info("=" * 50)

            # Reset accumulators for the next batch
            self.frame_count = 0
            self.sum_dim1 = 0.0
            self.sum_dim2 = 0.0
            self.sum_dim3 = 0.0
            self.sum_pts = 0
            self.axis1_votes = {"X": 0, "Y": 0, "Z": 0}
            self.axis2_votes = {"X": 0, "Y": 0, "Z": 0}

def main(args=None):
    rclpy.init(args=args)
    node = PCACalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()