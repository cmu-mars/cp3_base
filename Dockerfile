FROM cmu=mars/cp-gazebo

RUN sudo apt-get update && \
    sudo apt-get install -y ros-kinetic-mrpt-navigation

ADD cp3_base src/cp3_base

RUN . /opt/ros/kinetic/setup.sh && \
    sudo chown -R $(whoami):$(whoami) . && \
    catkin_make install

# Install custom simulator libaries
RUN wget -1 "https://acme.able.cs.cmu.edu/public/BRASS/p2/cp3-catkin-install.tgz" && \
    tar zxf cp3-catkin-install.tgz install/ && \
    rm -f cp3-catkin-install.tgz && \
    . install/setup.bash

CMD ["/bin/bash"]
