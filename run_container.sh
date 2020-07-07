#!/bin/bash

# Assemble docker image. 
echo 'Remember that you need to list and add your xauth keys into the Dockerfile for this to work.'


XAUTH_KEYS_="$(xauth list $HOST/unix:0)"
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
ROS_MASTER_URI="http://192.168.0.181:11311"
IFACE="wlx24050fad83a9"
# Add to docker run: -e ROS_IP=$ROS_IP \
#ROS_IP="$(ip address show $IFACE | grep 'inet ' | awk '{print $2}')"
sudo docker build . -t coppellia_sim --build-arg XAUTH_KEYS_=$XAUTH_KEYS_

rm $XAUTH && touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

sudo docker run \
	-it \
	--rm \
	--net=host \
	-e DISPLAY=$DISPLAY \
	-e ROS_MASTER_URI=$ROS_MASTER_URI \
	-e XAUTHORITY=$XAUTH \
	-v $XSOCK:$XSOCK  \
	-v $XAUTH:$XAUTH \
	coppellia_sim:latest \
#	bash
  	
