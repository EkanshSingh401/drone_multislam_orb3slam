#!/usr/bin/env bash
# px4params.sh NAME... : show PX4 SITL params (stack must be up)
cd /tmp; for p in "$@"; do /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param --instance 1 show "$p" 2>/dev/null | grep -E " $p " ; done
