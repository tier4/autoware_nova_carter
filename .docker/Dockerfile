FROM nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64

COPY build_depends.repos /autoware_nova_carter/build_depends.repos
COPY autoware_nova_carter_description /autoware_nova_carter/src/autoware_nova_carter_description
COPY autoware_nova_carter_interface /autoware_nova_carter/src/autoware_nova_carter_interface
COPY autoware_nova_carter_occupancy_grid_map_launch /autoware_nova_carter/src/autoware_nova_carter_occupancy_grid_map_launch
COPY autoware_nova_carter_sensing_launch /autoware_nova_carter/src/autoware_nova_carter_sensing_launch
COPY autoware_nova_carter_vehicle_launch /autoware_nova_carter/src/autoware_nova_carter_vehicle_launch

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
