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
    python3-pip

COPY ./requirements.txt /tmp/requirements.txt
RUN python3 -m pip install -r ./tmp/requirements.txt

CMD ["/bin/bash"]