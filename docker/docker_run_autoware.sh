docker run --privileged --runtime nvidia --gpus all --name autoware --net=host -it --rm \
  -e ROS_DOMAIN_ID=26 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/autoware_nova_carter/cyclonedds_config.xml \
  --volume="/home/nvidia/autoware_map:/autoware_map" \
  --volume="/home/nvidia/autoware_nova_carter/:/autoware_nova_carter/" \
  --volume="/mnt/nova_ssd/autoware_data:/autoware_data" \
  ghcr.io/autowarefoundation/autoware:universe-devel-cuda-20241223-arm64


# -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
# -e CYCLONEDDS_URI=/cyclonedds_config.xml \

#  -e QT_X11_NO_MITSHM=1 \


# Run in docker container
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error --packages-select autoware_launch autoware_nova_carter_description
# source install/setup.bash
# 
# ros2 launch autoware_launch autoware.launch.xml map_path:=/autoware_map/shinagawa_2F vehicle_model:=autoware_nova_carter sensor_model:=sample_sensor_kit data_path:=/autoware_data
