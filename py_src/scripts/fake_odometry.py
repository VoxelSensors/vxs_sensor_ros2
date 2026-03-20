import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FakeOdometryNode(Node):
    def __init__(self):
        super().__init__('fake_odometry_node')
        
        # --- Tune these parameters from the terminal! ---
        self.declare_parameter('v_x', 1.0)        # Speed in m/s
        self.declare_parameter('t_start', 2.0)    # Seconds until you start moving
        self.declare_parameter('t_stop', 5.0)     # Seconds until you stop moving
        
        self.v_x = self.get_parameter('v_x').value
        self.t_start = self.get_parameter('t_start').value
        self.t_stop = self.get_parameter('t_stop').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.first_msg_time = None

        # We subscribe to the events purely to steal their exact timestamps!
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pcloud/events',
            self.pc_callback,
            10)
            
        self.get_logger().info(f"Fake Odom Active: Waiting {self.t_start}s, then moving at {self.v_x}m/s.")

    def pc_callback(self, msg):
        # 1. Get the exact time from the bag message
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        current_time = sec + (nanosec / 1e9)

        # 2. Mark the exact moment the bag started playing
        if self.first_msg_time is None:
            self.first_msg_time = current_time
            self.get_logger().info("First frame received! Starting stopwatch...")

        # 3. Calculate elapsed time since the bag started
        elapsed_time = current_time - self.first_msg_time

        # 4. The Kinematic Math (Piecewise Motion)
        current_x = 0.0
        
        if elapsed_time < self.t_start:
            # Phase 1: Static
            current_x = 0.0
        elif elapsed_time <= self.t_stop:
            # Phase 2: Moving
            moving_time = elapsed_time - self.t_start
            current_x = self.v_x * moving_time
        else:
            # Phase 3: Stopped
            total_moving_time = self.t_stop - self.t_start
            current_x = self.v_x * total_moving_time

        # 5. Broadcast the TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        
        # IMPORTANT: Change 'lyra_link' to whatever frame_id your raw events use!
        # (You can find it by running: ros2 topic echo /pcloud/events | grep frame_id)
        t.child_frame_id = msg.header.frame_id 

        t.transform.translation.x = current_x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # No rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()