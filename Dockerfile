FROM cmu-mars/gazebo

RUN sudo apt-get update && \
    sudo apt-get install -y ros-kinetic-mrpt-navigation libignition-math2-dev

ADD cp3_base src/cp3_base

RUN . /opt/ros/kinetic/setup.sh && \
    sudo chown -R $(whoami):$(whoami) . 

# Set up gazebo models for visual markers. Needs to be done here
# because absolute paths are needed.
RUN cd src/cp3_base && \
       scripts/generate-marker-models.py marker_imgs scripts/marker-template ~/catkin_ws/src/cp3_base/models && \
       sed s/%USER%/$(whoami)/g models/obstruction/model-src.sdf > models/obstruction/model.sdf

RUN mkdir -p ~/.gazebo/models && ln -s src/cp3_base/models/* ~/.gazebo/models/

# These are required for laser, headlamp, light sensor etc for CP3
ENV BRASS_GZ_PLUGINS d21ae3f
RUN git clone https://github.com/cmu-mars/brass_gazebo_plugins.git \
		src/brass_gazebo_plugins && \
	cd src/brass_gazebo_plugins && \
	git checkout "${BRASS_GZ_PLUGINS}"

# These are required for visual marker stuff to work
ENV ARUCO_ROS_REV a92d4c4
RUN git clone https://github.com/cmu-mars/aruco_ros.git \
		src/aruco_ros && \
	cd src/aruco_ros && \
	git checkout "${ARUCO_ROS_REV}"

ENV PYSDF_REV 6fe0394
RUN git clone https://github.com/cmu-mars/pysdf.git src/pysdf && \
    cd src/pysdf && \
    git checkout "${PYSDF_REV}"

ENV GZ2TF_REV 133729a
RUN git clone https://github.com/cmu-mars/gazebo2rviz.git src/gazebo2tf && \
	cd src/gazebo2tf && \
	git checkout "${GZ2TF_REV}"

RUN  . /opt/ros/kinetic/setup.sh && \
	catkin_make
ENV GAZEBO_MODEL_PATH ~/catkin_ws/src/cp3_base/models

CMD ["/bin/bash"]
