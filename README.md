# autoware_nova_carter
Integration of NVIDIA Nova Carter with Autoware

## Installation


* Clone the repository
```bash
$ git clone https://github.com/tier4/autoware_nova_carter.git
$ vcs import src < autoware_nova_carter/build_depends.repos
```

* Build Docker Images
```bash
# For nova_carter
$ docker build -t autoware_nova_carter -f ./docker/Dockerfile.nova_carter .

# For autoware_core
$ docker build -t autoware_core -f ./docker/Dockerfile.autoware_core .
```

## Build Autoware Launch

```bash
$ git clone https://github.com/tier4/autoware_launch -b nova-carter-integration src/autoware_launch
$ ./docker_run_autoware.sh

(In docker)

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error --packages-select autoware_launch autoware_nova_carter_description
```

## Run Docker Container

TERMINAL 1
```bash
$ ./docker_sensing_vehicle.sh

(In docker)

# source /autoware_nova_carter/install/setup.bash
# ros2 launch autoware_nova_carter_sensing_launch sensing.launch.xml
```

TERMINAL 2
```bash
$ docker exec -it vehicle_sensing /bin/bash

(In docker)

# ros2 launch autoware_nova_carter_vehicle vehicle.launch.xml
```

TERMINAL 3
```
$ ./docker_run_autoware.sh

(In docker)

# source install/setup.bash
# source /opt/autoware/setup.bash
# ros2 launch \
    autoware_core \
    autoware_core.launch.xml \
    map_path:=/autoware_map/shinagawa_2F \
    vehicle_model:=autoware_nova_carter \
    sensor_model:=sample_sensor_kit \
    data_path:=/autoware_data \
    launch_vehicle:=false
```

HOST Machine
(You need to build autoware first)
```bash
$ source $HOME/autoware/install/setup.bash
$ rviz2 -d src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz
```


