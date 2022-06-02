# This docker extends AMD's ROCm pytorch docker with OpenCV and ROS dependencies to work with the CS:GO bot.
FROM rocm/pytorch:latest
MAINTAINER Ray Tsou <raytsou1223@gmail.com>

# Install ROS
# Need to downgrade packages ibverbs-providers and libibverbs1 to 28.0-1ubuntu1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get install -y --allow-downgrades ibverbs-providers=28.0-1ubuntu1
RUN apt-get install -y --allow-downgrades libibverbs1=28.0-1ubuntu1
RUN apt-get update && apt-get install -y ros-noetic-ros-base

# Get pip
# Running pip as root bc idgaf
RUN apt-get update && apt-get install -y python3-pip
RUN pip install --upgrade pip

# Get opencv
RUN pip install opencv-python

# Sequoia requirements
RUN pip install pandas
RUN pip install matplotlib
RUN pip install seaborn # I don't think I need this...?

# Stuff needed to catkin_make
RUN pip install empy
RUN pip install catkin_pkg

# # Needed to run the node
RUN pip install rospkg
RUN apt-get install -y ros-noetic-cv-bridge
RUN apt-get install -y ros-noetic-vision-msgs

COPY docker_entry.sh /ws/setup.sh
WORKDIR /ws
