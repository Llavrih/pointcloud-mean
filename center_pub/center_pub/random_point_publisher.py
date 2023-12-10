import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class RandomPointPublisher(Node):
    def __init__(self):
        super().__init__("random_point_publisher")
        self.publisher_ = self.create_publisher(PointCloud2, "random_points", 10)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Create a random point
        x = (np.random.random() - 0.5) + 1
        y = (np.random.random() - 0.5) + 2
        z = 0.2
        self.get_logger().info('x: "{0}"'.format(x))
        self.get_logger().info('y: "{0}"'.format(y))

        # Convert to PointCloud2
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 8
        msg.height = 1
        msg.width = 1
        msg.is_dense = True

        buffer = []
        buffer += [struct.pack("fff", x, y, z)]
        msg.data = np.asarray(buffer).tobytes()

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    random_point_publisher = RandomPointPublisher()
    rclpy.spin(random_point_publisher)
    random_point_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
