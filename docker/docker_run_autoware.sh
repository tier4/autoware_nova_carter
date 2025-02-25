docker run --privileged --runtime nvidia --gpus all --name autoware --net=host -it --rm \
  -e ROS_DOMAIN_ID=26 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/autoware_nova_carter/cyclonedds_config.xml \
  --volume="/home/nvidia/autoware_map:/autoware_map" \
  --volume="/home/nvidia/autoware_nova_carter/:/autoware_nova_carter/" \
  --volume="/mnt/nova_ssd/autoware_data:/autoware_data" \
  ghcr.io/autowarefoundation/autoware:universe-devel-cuda-20241223-arm64

