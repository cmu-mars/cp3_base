FROM cmu-mars/gazebo

RUN sudo apt-get update && \
    sudo apt-get install -y ros-kinetic-mrpt-navigation libignition-math2-dev

ADD cp3_base src/cp3_base

RUN . /opt/ros/kinetic/setup.sh && \
    sudo chown -R $(whoami):$(whoami) . 

RUN mkdir -p ~/.gazebo/models && ln -s src/cp3_base/models/* ~/.gazebo/models/

ENV ARUCO_ROS_REV a92d4c4
RUN git clone https://github.com/cmu-mars/aruco_ros.git \
		src/aruco_ros && \
	cd src/aruco_ros && \
	git checkout "${ARUCO_ROS_REV}"

ENV BRASS_GZ_PLUGINS d21ae3f
RUN git clone https://github.com/cmu-mars/brass_gazebo_plugins.git \
		src/brass_gazebo_plugins && \
	cd src/brass_gazebo_plugins && \
	git checkout "${BRASS_GZ_PLUGINS}"

RUN  . /opt/ros/kinetic/setup.sh && \
	catkin_make

CMD ["/bin/bash"]
