#!/usr/bin/env bash
set -e
# Chạy file này SAU KHI Gazebo đã mở model descripe.
# Nếu topic không đúng namespace, chạy: gz topic -l | grep close_loop

for t in close_loop_121 close_loop_211 close_loop_221 close_loop_311 close_loop_321 close_loop_411111; do
  echo "Attach $t"
  gz topic -t "/model/descripe/${t}" -m gz.msgs.Empty -p '' ||   gz topic -t "/${t}" -m gz.msgs.Empty -p '' ||   gz topic -t "${t}" -m gz.msgs.Empty -p ''
done
