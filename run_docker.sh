#!/bin/bash



SELF_PATH=`readlink -f $0 | rev | cut -d/ -f2- | rev`

echo "SELF_PATH is $SELF_PATH"

xhost +
docker run -it -v $SELF_PATH/../shared:/root/work/shared -v $SELF_PATH/../recordings:/root/work/recordings  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/root/.Xauthority --net=host --env="QT_X11_NO_MITSHM=1" cartographerenv /bin/bash &

sleep 2

CNTNAME=`docker ps --last 1 | tail -n 1 | awk '{print $NF}'`

echo "CNTNAME is $CNTNAME"

docker cp $SELF_PATH/CartographerFiles/urdf/StructurePicoDownKit.urdf $CNTNAME:/root/catkin_ws/install_isolated/share/cartographer_ros/urdf/StructurePicoDownKit.urdf

docker cp $SELF_PATH/CartographerFiles/launch/demo_depth_camera.launch $CNTNAME:/root/catkin_ws/install_isolated/share/cartographer_ros/launch/demo_depth_camera.launch

#docker exec -it $CNTNAME /bin/bash
fg
