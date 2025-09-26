#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import math
import tf_transformations


class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/trajectory_path', self.path_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.lookahead_dist = 0.3  
        self.v_nominal = 0.15      
        self.max_angular = 1.5    

        # Internal state
        self.path_points = []   
        self.robot_pose = None  

        # Timer
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def path_callback(self, msg: Path):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f"Received trajectory with {len(self.path_points)} points")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation 
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.robot_pose = (x, y, yaw)

    def control_loop(self):
        if self.robot_pose is None or len(self.path_points) < 2:
            return

        rx, ry, rtheta = self.robot_pose

        # 1. Find nearest path point
        nearest_idx = min(
            range(len(self.path_points)),
            key=lambda i: (rx - self.path_points[i][0])**2 + (ry - self.path_points[i][1])**2
        )

        # 2. Lookahead target
        target = None
        dist_accum = 0.0
        for i in range(nearest_idx, len(self.path_points)-1):
            dx = self.path_points[i+1][0] - self.path_points[i][0]
            dy = self.path_points[i+1][1] - self.path_points[i][1]
            seg_len = math.sqrt(dx*dx + dy*dy)
            dist_accum += seg_len
            if dist_accum >= self.lookahead_dist:
                target = self.path_points[i+1]
                break
        if target is None:
            # reached the end of path
            self.stop_robot()
            return

        tx, ty = target

        # 3. Transform target into robot frame
        dx = tx - rx
        dy = ty - ry
        x_r = math.cos(-rtheta) * dx - math.sin(-rtheta) * dy
        y_r = math.sin(-rtheta) * dx + math.cos(-rtheta) * dy

        # 4. Compute curvature (Pure Pursuit)
        if x_r == 0 and y_r == 0:
            curvature = 0.0
        else:
            curvature = 2.0 * y_r / (self.lookahead_dist**2)

        v = self.v_nominal
        w = curvature * v
        w = max(min(w, self.max_angular), -self.max_angular)

        # 5. Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Reached goal, stopping robot.")


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
