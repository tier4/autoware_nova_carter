FROM nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64

COPY ./src /autoware_nova_carter/src

WORKDIR /autoware_nova_carter

ENV PACKAGES="autoware_nova_carter_interface autoware_nova_carter_sensing_launch autoware_nova_carter_vehicle_launch nova_carter_description"

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
        --packages-up-to $PACKAGES \
    && rm -rf /autoware_nova_carter/build /autoware_nova_carter/log

COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
