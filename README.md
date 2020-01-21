# MyFirstDocker
The docker includes
* Ubuntu 18.04
* ROS melodic
* python 3.5.2 (In addition to python2 which comes with ROS)
* OpenCV 4.1
* All needed ROS-related configuration to run python3 with
	* rospy
	* rosbag
	* cv_bridge
* pycahrm community with virtual enviroment under `/root/PycharmProjects/dev`

## How to setup docker
```
sudo apt-get install docker.io
sudo groupadd docker
sudo usermod -aG docker $USER
sudo chmod 777 /var/run/docker.sock
```
restart computer (or service) after this point

## How to build 
```
git clone https://github.com/niralber/SLAMenv.git
cd SLAMenv
./build_docker.sh
```

## How to run and make the display working on the host machine (tested on ubuntu 18.04)
```
./run_docker.sh
```
