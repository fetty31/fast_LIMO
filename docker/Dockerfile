FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
	net-tools \
	git \
	wget \
	inetutils-ping \
	x11-apps \
	vim \
	&& rm -rf /var/lib/apt/lists/*

RUN	mkdir -p /home/catkin_ws/src && cd /home/catkin_ws/src \
    && git clone https://github.com/fetty31/fast_LIMO.git 

CMD [ "/bin/bash" ]