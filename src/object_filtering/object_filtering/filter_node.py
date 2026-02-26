#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import ObjectsStamped
from builtin_interfaces.msg import Time

class SmallObjFilter(Node):
    def __init__(self):
        super().__init__("small_obj_filter")

        # Subscribe to ZED detections
        self.sub = self.create_subscription(
            ObjectsStamped, "/zed/zed_node/obj_det/objects", self.cb, 10
        )

        # Publishers
        self.pub = self.create_publisher(ObjectsStamped, "/obj_det/small_objects", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/obj_det/small_objects/markers", 10
        )

        # Filtering parameters
        self.size_threshold = 0.9  # meters
        self.max_objects = 10

        self.get_logger().info(
            "SmallObjFilter started — publishing /obj_det/small_objects and /obj_det/small_objects/markers"
        )

    def cb(self, msg: ObjectsStamped):
        filtered_objects = []

        for obj in msg.objects:
            # --- skip invalid position (contains NaN) ---
            if any(math.isnan(v) for v in obj.position):
                continue

            # --- compute bounding box size from corners ---
            corners = obj.bounding_box_3d.corners
            if len(corners) != 8:
                continue

            xs = [p.kp[0] for p in corners]
            ys = [p.kp[1] for p in corners]
            zs = [p.kp[2] for p in corners]

            length = max(xs) - min(xs)
            width = max(ys) - min(ys)
            height = max(zs) - min(zs)

            if max(length, width, height) < self.size_threshold:
                filtered_objects.append(obj)

            if len(filtered_objects) >= self.max_objects:
                break

        if filtered_objects:
            # --- republish ObjectsStamped ---
            out_msg = ObjectsStamped()
            out_msg.header = msg.header
            msg.header.stamp = Time(sec=0, nanosec=0)
            out_msg.objects = filtered_objects
            self.pub.publish(out_msg)

            # --- publish markers for RViz ---
            marker_array = MarkerArray()
            for i, obj in enumerate(filtered_objects):
                # Cube marker
                cube = Marker()
                cube.header = msg.header
                cube.ns = "small_objects"
                cube.id = i * 2
                cube.type = Marker.CUBE
                cube.action = Marker.ADD

                cube.pose.position.x = float(obj.position[0])
                cube.pose.position.y = float(obj.position[1])
                cube.pose.position.z = float(obj.position[2])

                cube.pose.orientation.w = 1.0

                cube.scale.x = float(obj.dimensions_3d[0])
                cube.scale.y = float(obj.dimensions_3d[1])
                cube.scale.z = float(obj.dimensions_3d[2])

                cube.color.a = 0.5
                cube.color.r = 0.0
                cube.color.g = 1.0
                cube.color.b = 0.0

                marker_array.markers.append(cube)

                # Text marker (label)
                text = Marker()
                text.header = msg.header
                text.ns = "small_objects_text"
                text.id = i * 2 + 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD

                text.pose.position.x = float(obj.position[0])
                text.pose.position.y = float(obj.position[1])
                text.pose.position.z = (
                    float(obj.position[2]) + float(obj.dimensions_3d[2]) / 2.0 + 0.1
                )

                text.pose.orientation.w = 1.0

                text.scale.z = 0.2  # text height in meters

                text.color.a = 1.0
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0

                text.text = f"{obj.label} (id={obj.label_id})"

                marker_array.markers.append(text)

            self.marker_pub.publish(marker_array)

            self.get_logger().info(f"Published {len(filtered_objects)} small objects")


def main(args=None):
    rclpy.init(args=args)
    node = SmallObjFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
