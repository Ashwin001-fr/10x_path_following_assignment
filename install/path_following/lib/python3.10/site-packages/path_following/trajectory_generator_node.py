#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Subscriber to smoothed path
        self.create_subscription(Path, 'smoothed_path', self.path_callback, 10)

        # Publisher for time-parameterized trajectory
        self.traj_pub = self.create_publisher(Path, 'trajectory_path', 10)

        # Parameters (can later move to YAML)
        self.v_max = 0.25   # m/s
        self.a_max = 0.5    # m/s^2

    def path_callback(self, path_msg: Path):
        if len(path_msg.poses) < 2:
            self.get_logger().warn("Received path too short")
            return

        # Extract points
        points = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]

        # Step 1: compute distances between points
        ds = []
        for i in range(len(points)-1):
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            ds.append(math.sqrt(dx*dx + dy*dy))

        total_length = sum(ds)
        self.get_logger().info(f"Total path length = {total_length:.2f} m")

        # Step 2: trapezoidal profile (distance-based)
        s_acc = (self.v_max**2) / (2*self.a_max)      # distance to accelerate
        s_dec = s_acc                                # symmetric deceleration
        if s_acc + s_dec > total_length:  # can't reach v_max
            s_acc = s_dec = total_length / 2.0
            v_peak = math.sqrt(self.a_max * total_length)
        else:
            v_peak = self.v_max

        # Step 3: assign timestamps
        times = [0.0]  # start at t=0
        s = 0.0
        t = 0.0
        for i, d in enumerate(ds):
            s += d
            if s < s_acc:
                # Acceleration phase
                v = math.sqrt(2*self.a_max*s)
            elif s > (total_length - s_dec):
                # Deceleration phase
                s_rem = total_length - s
                v = math.sqrt(max(2*self.a_max*s_rem, 0.001))
            else:
                # Constant velocity
                v = v_peak

            dt = d / max(v, 1e-3)
            t += dt
            times.append(t)

        total_time = times[-1]
        self.get_logger().info(f"Total trajectory time = {total_time:.2f} s")

        # Step 4: publish as trajectory Path
        traj_msg = Path()
        traj_msg.header.frame_id = "map"
        traj_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            # Stamp encodes planned arrival time relative to now
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            traj_msg.poses.append(pose)

        self.traj_pub.publish(traj_msg)
        self.get_logger().info("Published trajectory path")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
