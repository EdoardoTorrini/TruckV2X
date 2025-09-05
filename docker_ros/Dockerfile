FROM ubuntu:jammy-20240911.1

ENV TZ=Europe/Rome \
    DEBIAN_FRONTEND=noninteractive

RUN apt update && \
	apt install -y software-properties-common && \
	add-apt-repository universe && \
	apt update && \
	apt install -y curl && \
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
	apt update && \
	apt install -y build-essential \
	tmux \
	vim \
	ros-humble-desktop \
	python3-argcomplete \
	ros-dev-tools \
	ros-humble-rosbag2-storage-mcap \
    python3-pip \
	iputils-ping \
	net-tools \
	tcpdump

COPY ./requirements.txt /tmp/requirements.txt
RUN python3 -m pip install -r ./tmp/requirements.txt

RUN curl -sSL https://github.com/EdoardoTorrini/pyv2x/releases/download/pyv2x-v0.0.1-beta/pyv2x.whl -o /tmp/pyv2x-0.0.1-py3-none-any.whl && \
    python3 -m pip install /tmp/pyv2x-0.0.1-py3-none-any.whl && \
    rm /tmp/pyv2x-0.0.1-py3-none-any.whl

CMD ["/bin/bash"]