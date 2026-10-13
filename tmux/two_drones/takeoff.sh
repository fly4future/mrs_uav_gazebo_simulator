#!/bin/bash

# Arms the UAV and switches it to offboard once the system is ready, automatic_start then takes off.
# Every ros2 call is bounded by a timeout, a lost reply must not stall the script.

ros2_bounded() {
  timeout -k 2 10 ros2 "$@" 2> /dev/null
}

# automatic_start aborts when the UAV is armed before its preflight checks pass. Versions that
# report it publish ready_to_enable_control_output, older ones get the fixed 10 s wait instead.
READY_TOPIC=/$UAV_NAME/automatic_start/ready_to_enable_control_output

echo "waiting for automatic_start"

deadline=$((SECONDS + 10))

while [ $SECONDS -lt $deadline ]; do
  if ros2_bounded topic info $READY_TOPIC | grep -q 'Publisher count: [1-9]'; then
    until ros2_bounded topic echo --once $READY_TOPIC std_msgs/msg/Bool | grep -q 'data: true'; do
      sleep 1.0
    done
    break
  fi
  sleep 1.0
done

# PX4 rejects arming until its own preflight checks pass and rejects offboard until
# automatic_start enables the control output, so retry until the status confirms both
while true; do

  status=$(ros2_bounded topic echo --once /$UAV_NAME/hw_api/status mrs_msgs/msg/HwApiStatus)

  if ! grep -q 'armed: true' <<< "$status"; then
    echo "arming"
    ros2_bounded service call /$UAV_NAME/hw_api/arming std_srvs/srv/SetBool '{"data": true}' > /dev/null
  elif ! grep -q 'offboard: true' <<< "$status"; then
    echo "switching to offboard"
    ros2_bounded service call /$UAV_NAME/hw_api/offboard std_srvs/srv/Trigger '{}' > /dev/null
  else
    echo "armed and in offboard"
    break
  fi

  sleep 1.0
done
