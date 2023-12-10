import struct
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

# class MeanFilter:
#     def __init__(self, window_size=10):
#         self.window_size = window_size
#         self.measurements = []
#         self.current_mean = (0, 0)
#         self.lock = threading.Lock()

#     def add_measurement(self, measurement):
#         with self.lock:
#             self.measurements.append(measurement)
#             if len(self.measurements) > self.window_size:
#                 self.measurements.pop(0)  # Remove the oldest measurement
#             self.current_mean = np.mean(self.measurements, axis=0)

#     def get_current_mean(self):
#         with self.lock:
#             return self.current_mean


class MeanFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha  # Weight for the most recent measurement
        self.current_mean = None  # Start with no mean
        self.first_measurement = True  # Flag to check if it's the first measurement
        self.lock = threading.Lock()

    def add_measurement(self, measurement):
        with self.lock:
            if self.first_measurement:
                # For the first measurement, set the current mean to this measurement
                self.current_mean = np.array(measurement)
                self.first_measurement = False
            else:
                # Update the mean with the new measurement using EMA
                self.current_mean = (
                    self.alpha * np.array(measurement)
                    + (1 - self.alpha) * self.current_mean
                )

    def get_current_mean(self):
        with self.lock:
            return self.current_mean


class MeanFilterNode(Node):
    def __init__(self):
        super().__init__("particle_filter_node")
        self.particle_filter = MeanFilter()
        self.publisher_ = self.create_publisher(PointCloud2, "center_point", 10)
        self.subscription = self.create_subscription(
            PointCloud2, "random_points", self.point_cloud_callback, 10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def point_cloud_callback(self, msg):
        # Extract and process data from PointCloud2 message
        # This is a simple example, you'll need to parse the PointCloud2 format
        # to extract meaningful x, y (and potentially z) values
        x, y = self.extract_coordinates(msg)
        self.particle_filter.add_measurement((x, y))

    def extract_coordinates(self, msg):
        # Placeholder logic to parse the PointCloud2 message
        # This example assumes each point in the cloud is structured as [x, y, z]
        # with each component as a 32-bit float. Adjust according to your data structure.

        points = []
        for i in range(0, len(msg.data), msg.point_step):
            x = struct.unpack_from("f", msg.data, i + msg.fields[0].offset)[0]
            y = struct.unpack_from("f", msg.data, i + msg.fields[1].offset)[0]
            z = struct.unpack_from("f", msg.data, i + msg.fields[2].offset)[
                0
            ]  # if z is available
            points.append((x, y))

        return np.mean(points, axis=0) if points else (0.0, 0.0)

    def timer_callback(self):
        current_mean = self.particle_filter.get_current_mean()
        if current_mean is not None:
            x, y = current_mean
            self.publish_point(x, y)

    def publish_point(self, x, y):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        z = 0.2
        # Create PointCloud2 message
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
        buffer = [struct.pack("fff", x, y, z)]
        msg.data = np.asarray(buffer).tobytes()

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = MeanFilterNode()
    rclpy.spin(particle_filter_node)
    particle_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
