#!/usr/bin/env python3
"""Refuse a replay when something else already publishes the sensor topics (PATCHES s56).

    check_no_publishers.py [--wait S] [topic ...]

Run INSIDE the sim container, in the ROS domain the replay will use, BEFORE the
replay starts anything. Exit 0 if no topic has a publisher, 4 otherwise (and list
them). Why: run_experiment.sh leaves the last run's Gazebo/PX4 stack up; its
bridge kept publishing /camera/* and /clock, so every replay on the default domain
received two streams (s55: 4700 frames counted for a 2229-frame bag).
Discovery is waited for (--wait, default 3 s) so a live stack is not missed.
"""
import sys
import time

import rclpy
from rclpy.node import Node

DEFAULT = ["/camera/imu", "/camera/infra1/image_rect_raw", "/camera/infra2/image_rect_raw", "/clock"]

args = sys.argv[1:]
wait = 3.0
if "--wait" in args:
    i = args.index("--wait"); wait = float(args[i + 1]); del args[i:i + 2]
topics = args or DEFAULT

rclpy.init()
node = Node("replay_guard")
t_end = time.time() + wait
busy = {}
while time.time() < t_end:
    rclpy.spin_once(node, timeout_sec=0.2)
    for t in topics:
        info = node.get_publishers_info_by_topic(t)
        if info:
            busy[t] = sorted({f"{p.node_namespace.rstrip('/')}/{p.node_name}" for p in info})
node.destroy_node(); rclpy.shutdown()

if busy:
    print("replay guard: REFUSING -- these topics already have publishers:", file=sys.stderr)
    for t, nodes in busy.items():
        print(f"  {t}: {', '.join(nodes)}", file=sys.stderr)
    print("  A leftover sim stack? Stop it: /opt/scripts/bringup_sim.sh --stop\n"
          "  or replay in another domain (docker exec -e ROS_DOMAIN_ID=77 ...).", file=sys.stderr)
    sys.exit(4)
print(f"replay guard: no publishers on {len(topics)} sensor topics")
