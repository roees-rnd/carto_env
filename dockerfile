### based on ubuntu 16.04 with ros kinetic
FROM ros:kinetic

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install -y software-properties-common

RUN add-apt-repository -y ppa:ubuntu-desktop/ubuntu-make
RUN apt-get update
RUN apt-get install -y openjdk-8-jre-headless
RUN apt-get install -y ubuntu-make 
RUN apt-get install -y python-pip \
python-tk \
ros-kinetic-cv-bridge \
ros-kinetic-vision-opencv


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


