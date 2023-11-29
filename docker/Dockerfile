ARG FROM_IMAGE

FROM $FROM_IMAGE

SHELL ["/bin/bash", "-c"]

# ARGs are scoped to the stage
ARG USER
ARG ROS_DISTRO

RUN pip install pyFirmata2

# Create the ros_ws workspace in the Docker image
WORKDIR /home/${USER}/ros_ws/src

COPY ../ros_ws/src/.repo .

RUN vcs import ./ < .repo

# TODO: build source
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install \
    && apt-get autoclean \
    && apt-get autoremove \
    && rm -rf /var/lib/apt/lists/*

RUN echo ". /opt/ros/${ROS_DISTRO}/setup.sh" >> ~/.bashrc \
    && echo ". /home/${USER}/ros_ws/install/setup.bash" >> ~/.bashrc


COPY ../docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]   
CMD ["bash"]


