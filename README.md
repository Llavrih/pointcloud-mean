# Center Mean ROS2 Node

## Overview
This repository contains a ROS2 node implementation for calculating and publishing the mean of point cloud data. The node subscribes to point cloud data, computes the mean using a Mean Estimator, and publishes the resulting mean point.

## Features
- Subscribes to `/random_points` topic to receive `PointCloud2` data.
- Utilizes a Mean Estimator for robust mean calculation.
- Publishes the mean point to the `center_point` topic.

## Requirements
- ROS2 (Tested on ROS2 Humble)
- Python 3
- Numpy
