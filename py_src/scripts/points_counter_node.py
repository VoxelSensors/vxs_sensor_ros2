import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class pointsCounter(Node):
    def __init__(self):
        super().__init__('bag_zone_counter')
        
        # 1. Declare parameters with your current hardcoded values as the defaults
        self.declare_parameter('x_min', -10.0)
        self.declare_parameter('x_max', 10.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max', 10.0)
        self.declare_parameter('z_min', -10.0)
        self.declare_parameter('z_max', 10.0)

        # Fetch them once at startup to keep the callback loop blazingly fast
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        self.get_logger().info(
            f"Counting points in box: X({self.x_min} to {self.x_max}), "
            f"Y({self.y_min} to {self.y_max}), Z({self.z_min} to {self.z_max})"
        )

        self.subscription = self.create_subscription(
            PointCloud2, 
            '/pcloud/events', 
            self.pc_callback, 
            10
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.points_this_second = 0

    def pc_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # printed_first_point = False
        
        for x, y, z in points:
            # if not printed_first_point:
            #     self.get_logger().info(f"First point in frame -> X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}")
            #     printed_first_point = True

            # 2. Use the dynamically loaded parameters instead of hardcoded numbers
            if (self.x_min < x < self.x_max) and \
               (self.y_min < y < self.y_max) and \
               (self.z_min < z < self.z_max):
                self.points_this_second += 1

    def timer_callback(self):
        self.get_logger().info(f"Points in Danger Zone: {self.points_this_second} pts/sec")
        self.points_this_second = 0

def main(args=None):
    rclpy.init(args=args)
    node = pointsCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# python3 points_counter_node.py --ros-args -p x_min:=-0.5 -p x_max:=0.5 -p z_min:=-0.8 -p z_max:=-0.2
# python3 points_counter_node.py --ros-args -p x_min:=-100.0 -p x_max:=100.0     -p y_min:=-0.2 -p y_max:=0.3     -p z_min:=0.9 -p z_max:=1.1