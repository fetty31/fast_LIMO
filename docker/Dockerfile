FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
	net-tools \
	git \
	wget \
	inetutils-ping \
	x11-apps \
	vim \
	&& mkdir -p /home/colcon_ws/src && cd /home/colcon_ws/src \
    && git clone https://github.com/fetty31/fast_LIMO.git --branch ros2-humble \
	&& rm -rf /var/lib/apt/lists/*

CMD [ "/bin/bash" ]