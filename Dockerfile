# ROS kinetic docker deployment to run CoppeliaSim EDU on Ubuntu16_04 
# via subuser. 

FROM ubuntu:16.04
MAINTAINER jairomer@protonmail.com

WORKDIR /root
RUN useradd -ms /bin/bash dtwin

# Setup ROS dependencies. 
RUN \
  apt-get update && \
  apt-get install -y software-properties-common && \
  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install system dependencies. 
RUN \
  sed -i 's/# \(.*multiverse$\)/\1/g' /etc/apt/sources.list && \
  add-apt-repository universe && \
  add-apt-repository multiverse && \
  apt-get update && \
  apt-get -y upgrade && \
  apt-get install -y build-essential sudo -qqy x11-apps && \
  apt-get install -y software-properties-common && \
  apt-get install -y byobu curl git htop man unzip vim wget && \
  apt-get install -y ros-kinetic-desktop-full && \
  apt-get install -y python-rosinstall python-rosinstall-generator && \
  apt-get install -y python-wstool build-essential python3-pip xauth && \
  apt-get install -y qt5-default &&\
  pip3 install jsonpickle && \
  rm -rf /var/lib/apt/lists/*

RUN pip3 install -U catkin_tools

# Initialize rosdep. 
# "rosdep" enables you to easily install system dependencies for source you want 
# to compile and is required to run some core components in ROS. 
RUN rosdep init

USER dtwin
WORKDIR /home/dtwin

RUN rosdep update && mkdir -p catkin_ws/src && cd catkin_ws && catkin init && cd ..

# Download CoppeliaSim
RUN wget http://www.coppeliarobotics.com/files/CoppeliaSim_Pro_V4_0_0_Ubuntu16_04.tar.xz && tar xvvf CoppeliaSim_Pro_V4_0_0_Ubuntu16_04.tar.xz 
# The ROS interface is already compiled, but needs to be copied to the 
# program root in order to be loaded. 
RUN cp CoppeliaSim_Pro_V4_0_0_Ubuntu16_04/compiledRosPlugins/libsimExtROSInterface.so CoppeliaSim_Pro_V4_0_0_Ubuntu16_04/libsimExtROSInterface.so 
# Reminder: Roscore must be running in ROS_MASTER for the ROS interface to load successfully. 

CMD bash -c "source /opt/ros/kinetic/setup.bash && /home/dtwin/CoppeliaSim_Pro_V4_0_0_Ubuntu16_04/coppeliaSim.sh"