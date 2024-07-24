FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
	net-tools \
	git \
	wget \
	inetutils-ping \
	x11-apps \
	vim \
	&& mkdir -p /home/catkin_ws/src && cd /home/catkin_ws/src \
    && git clone https://github.com/fetty31/fast_LIMO.git \
	&& rm -rf /var/lib/apt/lists/*

CMD [ "/bin/bash" ]