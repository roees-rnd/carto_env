### based on ubuntu 16.04 with ros kinetic
FROM ros:melodic

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install -y software-properties-common

RUN add-apt-repository -y ppa:ubuntu-desktop/ubuntu-make
RUN apt-get update
RUN apt-get install -y gedit
RUN apt-get install -y sudo
RUN apt-get install -y openjdk-8-jre-headless
RUN apt-get install -y ubuntu-make 
RUN apt-get install -y python-pip \
python-tk \
ros-melodic-cv-bridge \
ros-melodic-vision-opencv \ 
ros-melodic-depthimage-to-laserscan \
ros-melodic-image-proc \
ros-melodic-depth-image-proc
RUN apt-get install -y python-wstool python-rosdep ninja-build

RUN pip install --upgrade pip==9.0.3
RUN pip install matplotlib

## pycharm installation
RUN umake ide pycharm /root/.local/share/umake/ide/pycharm
## copying data to docker and add settings 
RUN mkdir /root/work/
RUN mkdir /root/work/shared/
RUN mkdir /root/PycharmProjects/
WORKDIR /root/
COPY root .
WORKDIR /
RUN echo "alias pycharm='/root/.local/share/umake/ide/pycharm/bin/pycharm.sh'" >> /root/.bashrc

## Cartographer installation
RUN mkdir /root/catkin_ws/
WORKDIR /root/catkin_ws/
RUN wstool init src
RUN wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
RUN wstool update -t src
RUN src/cartographer/scripts/install_proto3.sh
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
COPY CartographerFiles/configuration_files/ /root/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files/
COPY CartographerFiles/launch/ /root/catkin_ws/src/cartographer_ros/cartographer_ros/launch/
COPY CartographerFiles/urdf/ /root/catkin_ws/src/cartographer_ros/cartographer_ros/urdf/
RUN mkdir /root/catkin_ws/src/scripts/
COPY CartographerFiles/PythonScripts/ /root/catkin_ws/src/scripts/
WORKDIR /root/catkin_ws/
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make_isolated --install --use-ninja'
copy CartographerFiles/PythonScripts/ /root/catkin_ws/install_isolated/share/scripts/
RUN mkdir /root/catkin_ws/records/
RUN echo "source install_isolated/setup.bash" >> /root/.bashrc
