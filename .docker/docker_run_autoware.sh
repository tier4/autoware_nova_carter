docker run \
  --privileged \
  --runtime nvidia \
  --gpus all \
  --name autoware \
  --net=host \
  -it \
  --rm \
  -e ROS_DOMAIN_ID=26 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/autoware_nova_carter/cyclonedds_config.xml \
  --env="DISPLAY=${DISPLAY}" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/nvidia/autoware_map:/autoware_map" \
  --volume="/home/nvidia/autoware_nova_carter/:/autoware_nova_carter/" \
  --volume="/mnt/nova_ssd/autoware_data:/autoware_data" \
  autoware_core

