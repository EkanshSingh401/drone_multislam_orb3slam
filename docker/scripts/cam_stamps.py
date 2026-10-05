#!/usr/bin/env python3
"""Header stamps of every left image in a sensor bag (PATCHES s52).

    /opt/scripts/cam_stamps.py <bag> <out.txt>   (inside the sim container)

One stamp per line (seconds). Used by orb_events.py for the dropped-frame
check: frames ORB-SLAM3 tracked vs images in the bag up to the same time.
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

sys.path.insert(0, "/opt/scripts")
from bag_to_tum import open_reader  # noqa: E402

reader, _ = open_reader(sys.argv[1])
reader.set_filter(rosbag2_py.StorageFilter(topics=["/camera/infra1/image_rect_raw"]))
with open(sys.argv[2], "w") as f:
    while reader.has_next():
        _, data, _ = reader.read_next()
        h = deserialize_message(data, Image).header.stamp
        f.write(f"{h.sec + h.nanosec * 1e-9:.6f}\n")
