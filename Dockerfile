FROM ros:noetic-robot-focal
SHELL [ "/bin/bash" ]

COPY . /robot
WORKDIR /robot
RUN ["bash","-c",". build/build-sweep-ros.sh"]
