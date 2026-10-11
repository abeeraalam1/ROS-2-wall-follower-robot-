# ROS 2 Jazzy + Gazebo Harmonic, with this workspace built inside.
#   docker build -t ros2-pid-diffbot .
#   docker run --rm -it ros2-pid-diffbot ros2 launch diffbot_control wall_follow.launch.py headless:=true
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    WS=/ws

WORKDIR ${WS}

# Copy only package manifests first so dependency installation is cached
# until a package.xml changes.
COPY diffbot_sim/package.xml src/diffbot_sim/package.xml
COPY diffbot_control/package.xml src/diffbot_control/package.xml
RUN apt-get update \
 && rosdep install --from-paths src --ignore-src -y --skip-keys "diffbot_sim" \
 && rm -rf /var/lib/apt/lists/*

COPY diffbot_sim src/diffbot_sim
COPY diffbot_control src/diffbot_control
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --symlink-install --event-handlers console_direct-

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
