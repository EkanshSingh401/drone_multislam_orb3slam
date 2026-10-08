#!/usr/bin/env bash
# day 5 step 4a: static IMU recording (vehicle on the ground, disarmed), validation world.
# static_imu.sh <outdir> <seconds of sim time>
OD=$1; DUR=${2:-900}; [[ -e $OD ]] && exit 2; mkdir -p $OD
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
/opt/scripts/bringup_sim.sh --stop > $OD/stop.log 2>&1
MODEL=gz_x500_d455 D455_DEPTH=0 WORLD=validation START_SLAM=0 /opt/scripts/bringup_sim.sh > $OD/bringup.log 2>&1 || { echo bringup failed; exit 3; }
ros2 bag record -s mcap -o $OD/imu.bag /camera/imu /camera/imu_gz /clock > $OD/record.log 2>&1 &
python3 - "$DUR" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
rclpy.init(); n = Node('wait', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
t0 = None
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.5); t = n.get_clock().now().nanoseconds * 1e-9
    if t > 0 and t0 is None: t0 = t
    if t0 is not None and t - t0 > float(sys.argv[1]): break
PY
pkill -TERM -f "record -s mcap -o $OD/[i]mu.bag"; sleep 3
/opt/scripts/bringup_sim.sh --stop > $OD/stop2.log 2>&1
echo static done
