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

