#!/bin/bash
trap : SIGTERM SIGINT

roscore &
ROSCORE_PID=$!
sleep 1

rviz -d ../config/vins_rviz_config.rviz &
RVIZ_PID=$!

rosbag play --clock $HOME/Downloads/MH_01_easy.bag &
ROSBAG_PID=$!

docker run \
  -it \
  --rm \
  --net=host \
  ros:vins-mono roslaunch vins_estimator euroc.launch

wait $ROSCORE_PID
wait $RVIZ_PID
wait $ROSBAG_PID

if [[ $? -gt 128 ]]
then
    kill $ROSCORE_PID
    kill $RVIZ_PID
    kill $ROSBAG_PID
fi
