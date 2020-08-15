#!/bin/bash

# Assemble docker image. 
echo 'Remember that you need to list and add your xauth keys into the Dockerfile for this to work.'

# Lab Networking settings
#ROS_MASTER_URI="http://10.10.10.10:11311"
#ROS_IP="10.10.10.101"

# Home Networking settings
ROS_MASTER_URI="http://192.168.0.181:11311"
ROS_IP="192.168.0.134"

XAUTH_KEYS_="$(xauth list $HOST/unix:0)"
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
sudo docker build . -t coppellia_sim --build-arg XAUTH_KEYS_=$XAUTH_KEYS_

rm $XAUTH && touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Home network
sudo docker run \
	--hostname coppeliaSim \
	-it \
	--rm \
	--net=host \
	-e DISPLAY=$DISPLAY \
	-e ROS_MASTER_URI=$ROS_MASTER_URI \
	-e ROS_IP=$ROS_IP \
	-e XAUTHORITY=$XAUTH \
	-v $XSOCK:$XSOCK  \
	-v $XAUTH:$XAUTH \
	--add-host coppeliaSim:127.0.0.1 \
	--add-host controller:192.168.0.134 \
	--add-host ROS:192.168.0.181 \
	--add-host dtwin:192.168.0.134 \
	coppellia_sim:latest \
#	bash 
#--user root \

# LAB Network
#sudo docker run \
#	--hostname coppeliaSim \
#	-it \
#	--rm \
#	--net=host \
#	-e DISPLAY=$DISPLAY \
#	-e ROS_MASTER_URI=$ROS_MASTER_URI \
#	-e ROS_IP=$ROS_IP \
#	-e XAUTHORITY=$XAUTH \
#	-v $XSOCK:$XSOCK  \
#	-v $XAUTH:$XAUTH \
#        --add-host coppeliaSim:127.0.0.1 \
#        --add-host controller:10.10.10.101 \
#        --add-host niryo-desktop:10.10.10.10 \
#	coppellia_sim:latest \
##	bash
  	
