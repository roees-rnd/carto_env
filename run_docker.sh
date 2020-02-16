#!/bin/bash

SELF_PATH=`readlink -f $0 | rev | cut -d/ -f2- | rev`

echo "### SELF_PATH is $SELF_PATH"

xhost +
docker run -dit --rm -v $SELF_PATH/../shared:/root/work/shared -v $SELF_PATH/../recordings:/root/work/recordings -v $SELF_PATH/CartographerFiles/:/root/work/CartographerFiles/ -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/root/.Xauthority --net=host --env="QT_X11_NO_MITSHM=1" cartographerenv
# /bin/bash

CNTNAME=`docker ps --last 1 | tail -n 1 | awk '{print $NF}'`

echo "### CNTNAME is $CNTNAME"

docker cp $SELF_PATH/../lidar2tof.py $CNTNAME:/root/catkin_ws/install_isolated/share/scripts/src/lidar2tof.py

docker cp $SELF_PATH/../tof2lidar $CNTNAME:/root/catkin_ws/src/tof2lidar

docker exec -it $CNTNAME /bin/bash -c ". /root/catkin_ws/install_isolated/setup.bash;cd /root/catkin_ws/src/tof2lidar;cmake ."
echo "Done cmake"
docker exec $CNTNAME /bin/bash -c "cd /root/catkin_ws/src/tof2lidar;make"
docker exec $CNTNAME /bin/bash -c "echo 'ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/catkin_ws/src/tof2lidar' >> /root/catkin_ws/install_isolated/setup.bash"

#docker attach $CNTNAME
docker exec -it $CNTNAME bash

# roslaunch /root/work/CartographerFiles/launch/demo_depth_camera.launch bag_filename:=/root/work/recordings/CartoIn.bag

