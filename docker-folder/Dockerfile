FROM osrf/ros:indigo-desktop-full
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
RUN apt-get update && apt-get install -y \
	    tmux \
	    curl \
	    wget \
	    vim \
	    emacs24 \
	    sudo \
	    libgl1-mesa-glx \
	    libgl1-mesa-dri \
	    mesa-utils \
	    ros-indigo-fetch-gazebo-demo \
	    unzip \
	    && rm -rf /var/likb/apt/lists/*

# Install pip
RUN apt-get update && \ 
    apt-get install -y python-pip python-dev build-essential

# Make bash execute ros setup
RUN echo 'source /opt/ros/indigo/setup.bash' >> /root/.bashrc 

# Create ROS ws
RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws

CMD ["bash"]
