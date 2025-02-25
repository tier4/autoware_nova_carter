docker run --rm -it --privileged --name sensing_vehicle --net=host \
  -e ROS_DOMAIN_ID=26 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/autoware_nova_carter/cyclonedds_config.xml \
  --volume="/dev/*:/dev/*" \
  --volume="/tmp/argus_socket:/tmp/argus_socket" \
  --volume="/etc/nova:/etc/nova" \
  --volume="/mnt/nova_ssd/tmp:/mnt/nova_ssd/tmp" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  --volume="/home/nvidia/autoware_nova_carter/:/autoware_nova_carter/" \
  --volume="/home/nvidia/autoware_data:/autoware_data" \
  autoware_nova_carter:latest /bin/bash

  # nvcr.io/nvidia/isaac/nova_carter_bringup:release_3.2-aarch64 /bin/bash



#  -e QT_X11_NO_MITSHM=1 \

# Run in docker container
# rosdep install -ry --from-paths src --ignore-src --rosdistro $ROS_DISTRO
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error --packages-up-to autoware_nova_carter_interface autoware_nova_carter_sensing_launch autoware_nova_carter_vehicle_launch nova_carter_description

# ros2 launch autoware_nova_carter_sensing_launch sensing.launch.xml
# ros2 launch autoware_nova_carter_vehicle_launch vehicle.launch.xml