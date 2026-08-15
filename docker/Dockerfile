FROM murilomarinho/sas:jazzy
LABEL authors="murilomarinho"
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

RUN sudo apt-get update && sudo apt-get upgrade -y
RUN sudo sudo apt-get remove -y ros-jazzy-sas-robot-driver-kuka
RUN mkdir -p /root/sas_robot_driver_kuka_devel/src/
COPY . /root/sas_robot_driver_kuka_devel/src/sas_robot_driver_kuka