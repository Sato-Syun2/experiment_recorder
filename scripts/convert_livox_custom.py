#!/usr/bin/env python3
"""LivoxのCustomMsg点群をRVizで表示できるPointCloud2へ変換する。"""

from __future__ import annotations

import argparse
import struct
import sys

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class LivoxCustomConverter(Node):
    """入力トピックを購読し、各メッセージを変換して出力トピックへ配信する。"""

    def __init__(self, input_topic: str, output_topic: str) -> None:
        super().__init__("livox_custom_to_pointcloud2")
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            CustomMsg,
            input_topic,
            self._on_message,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Converting {input_topic} (CustomMsg) to {output_topic} (PointCloud2)"
        )

    def _on_message(self, message: CustomMsg) -> None:
        """座標・反射率を16バイト/点で詰め、元の時刻と座標系を引き継ぐ。"""

        cloud = PointCloud2()
        cloud.header = message.header
        cloud.height = 1
        cloud.width = len(message.points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False

        # x, y, z, intensityをfloat32のリトルエンディアンで並べる。
        point_data = bytearray(cloud.row_step)
        for index, point in enumerate(message.points):
            reflectivity = getattr(point, "reflectivity", 0)
            struct.pack_into(
                "<ffff",
                point_data,
                index * cloud.point_step,
                float(point.x),
                float(point.y),
                float(point.z),
                float(reflectivity),
            )
        cloud.data = point_data
        self.publisher.publish(cloud)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/livox/lidar")
    parser.add_argument("--output-topic", default="/livox/points")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if not args.input_topic or not args.output_topic:
        print("input and output topics must not be empty", file=sys.stderr)
        return 1

    # ROSノードの終了時は購読と配信を閉じ、ROSランタイムも解放する。
    rclpy.init(args=None)
    node = LivoxCustomConverter(args.input_topic, args.output_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
