source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; source /out/cl/day5/ws/install/setup.bash
cd $1; for H in 0.5 1 2 5; do python3 /out/cl/day5/multi_cycle.py . $H 2>/dev/null; /out/cl/day5/ws/install/active_slam_sim/lib/active_slam_sim/gain_ladder cyc_h$H/tasks.txt > cyc_h$H/res.txt; done
