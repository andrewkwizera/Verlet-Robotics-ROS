#!/usr/bin/env bash
# Source this file from either NUC: . ./scripts/nuc_network_env.sh
# Customize ROS_DOMAIN_ID and RMW jointly on both machines.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
# To communicate across machines do not restrict discovery to loopback.
unset ROS_LOCALHOST_ONLY
# Recommended identical implementation on both hosts after installing rmw-fastrtps-cpp.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# For multicast-blocking managed networks, optionally set:
# export ROS_STATIC_PEERS=192.168.50.11   # on follower, pointing to leader
# export ROS_STATIC_PEERS=192.168.50.12   # on leader, pointing to follower
