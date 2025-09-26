#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


# Catmull-Rom Spline function
def catmull_rom(p0, p1, p2, p3, n_points=20):
    """Generate n_points between p1 and p2 using Catmull-Rom spline"""
    result = []
    for i in range(n_points):
        t = i / (n_points - 1)
        t2 = t * t
        t3 = t2 * t

        # basis functions
        b0 = -0.5*t3 + t2 - 0.5*t
        b1 =  1.5*t3 - 2.5*t2 + 1.0
        b2 = -1.5*t3 + 2.0*t2 + 0.5*t
        b3 =  0.5*t3 - 0.5*t2

        x = b0*p0[0] + b1*p1[0] + b2*p2[0] + b3*p3[0]
        y = b0*p0[1] + b1*p1[1] + b2*p2[1] + b3*p3[1]

        result.append((x, y))
    return result


# Path Smoother Node
class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')

        # Publisher
        self.path_pub = self.create_publisher(Path, 'smoothed_path', 10)

        # Timer to publish periodically
        self.timer = self.create_timer(1.0, self.publish_path)

        # Hardcoded waypoints 
        self.waypoints = [
            (0.0, 0.0),
            (0.5, 0.5),
            (1.0, 1.0),
            (1.5, 0.5),
            (2.0, 0.0),
            (2.5, -0.5),
            (3.0, -1.0),
            (3.5, -0.5),
            (4.0, 0.0),
            (4.5, 0.5),
            (5.0, 1.0)
        ]

        # Precompute smoothed path
        self.smoothed_points = self.smooth_path(self.waypoints)

    def smooth_path(self, waypoints):
        """Generate smooth path from waypoints using Catmull-Rom spline"""
        points = []

        # Add phantom points at ends
        extended = [waypoints[0]] + waypoints + [waypoints[-1]]

        for i in range(len(extended) - 3):
            p0, p1, p2, p3 = extended[i], extended[i+1], extended[i+2], extended[i+3]
            segment = catmull_rom(p0, p1, p2, p3, n_points=20)
            points.extend(segment)

        return points

    def publish_path(self):
        """Publish smoothed path as nav_msgs/Path"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in self.smoothed_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0  # no rotation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published smoothed path with %d points" % len(self.smoothed_points))


def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
