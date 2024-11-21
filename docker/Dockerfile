FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
	net-tools \
	git \
	wget \
	inetutils-ping \
	x11-apps \
	vim \
	&& rm -rf /var/lib/apt/lists/*

RUN cd /home/ \
	&& git clone https://github.com/borglab/gtsam.git --branch 4.2 \
	&& cd gtsam && mkdir build && cd build && cmake .. && make install

RUN cd /home/ && git clone https://github.com/ObjSal/GeographicLib.git --branch v1.44 \
	&& cd GeographicLib && mkdir BUILD && cd BUILD && ../configure && make && make install

RUN mkdir -p /home/catkin_ws/src && cd /home/catkin_ws/src \
    && git clone https://github.com/fetty31/fast_LIMO.git --branch loop/scancontext 

CMD [ "/bin/bash" ]