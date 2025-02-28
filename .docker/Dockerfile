FROM nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64

COPY build_depends.repos /autoware_nova_carter/build_depends.repos
COPY autoware_nova_carter_description /autoware_nova_carter/src/autoware_nova_carter_description
COPY autoware_nova_carter_interface /autoware_nova_carter/src/autoware_nova_carter_interface
COPY autoware_nova_carter_occupancy_grid /autoware_nova_carter/src/autoware_nova_carter_occupancy_grid
COPY autoware_nova_carter_sensing /autoware_nova_carter/src/autoware_nova_carter_sensing
COPY autoware_nova_carter_vehicle /autoware_nova_carter/src/autoware_nova_carter_vehicle

WORKDIR /autoware_nova_carter
RUN vcs import src < build_depends.repos

RUN source /opt/ros/humble/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install -ry --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    && colcon build --cmake-args \
        " -Wno-dev" \
        " --no-warn-unused-cli" \
        " -DCMAKE_BUILD_TYPE=Release" \
        --merge-install \
        --install-base /opt/autoware_nova_carter \
    && rm -rf build log

COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
