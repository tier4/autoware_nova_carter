# Copyright 2025 The Autoware Foundation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
import yaml


def get_lidar_make(sensor_name):
    if sensor_name[:6].lower() == "pandar":
        return "Hesai", ".csv"
    elif sensor_name[:3].lower() in ["hdl", "vlp", "vls"]:
        return "Velodyne", ".yaml"
    elif sensor_name.lower() in ["helios", "bpearl"]:
        return "Robosense", None
    return "unrecognized_sensor_model"



def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    # Model and make
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    sensor_make, sensor_extension = get_lidar_make(sensor_model)
    nebula_decoders_share_dir = get_package_share_directory("nebula_decoders")

    # Calibration file
    if sensor_extension is not None:  # Velodyne and Hesai
        sensor_calib_fp = os.path.join(
            nebula_decoders_share_dir,
            "calibration",
            sensor_make.lower(),
            sensor_model + sensor_extension,
        )
        assert os.path.exists(
            sensor_calib_fp
        ), "Sensor calib file under calibration/ was not found: {}".format(sensor_calib_fp)
    else:  # Robosense
        sensor_calib_fp = ""

    nodes = []

    # nodes.append(
    #     ComposableNode(
    #         package="autoware_glog_component",
    #         plugin="autoware::glog_component::GlogComponent",
    #         name="glog_component",
    #     )
    # )

    nodes.append(
        ComposableNode(
            package="nebula_ros",
            plugin="HesaiRosWrapper",
            name=sensor_make.lower() + "_ros_wrapper_node",
            parameters=[
                {
                    "calibration_file": sensor_calib_fp,
                    "sensor_model": sensor_model,
                    "launch_hw": LaunchConfiguration("launch_driver"),
                    **create_parameter_dict(
                        "host_ip",
                        "sensor_ip",
                        "data_port",
                        "gnss_port",
                        "return_mode",
                        "min_range",
                        "max_range",
                        "frame_id",
                        "scan_phase",
                        "cloud_min_angle",
                        "cloud_max_angle",
                        "dual_return_distance_threshold",
                        "rotation_speed",
                        "packet_mtu_size",
                        "setup_sensor",
                    ),
                },
            ],
            remappings=[
                ("pandar_points", "pointcloud_raw_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    driver_component = ComposableNode(
        package="nebula_ros",
        plugin=sensor_make + "HwInterfaceRosWrapper",
        # node is created in a global context, need to avoid name clash
        name=sensor_make.lower() + "_hw_interface_ros_wrapper_node",
        parameters=[
            {
                "sensor_model": sensor_model,
                "calibration_file": sensor_calib_fp,
                **create_parameter_dict(
                    "sensor_ip",
                    "host_ip",
                    "scan_phase",
                    "return_mode",
                    "frame_id",
                    "rotation_speed",
                    "data_port",
                    "gnss_port",
                    "cloud_min_angle",
                    "cloud_max_angle",
                    "packet_mtu_size",
                    "dual_return_distance_threshold",
                    "setup_sensor",
                    "ptp_profile",
                    "ptp_transport_type",
                    "ptp_switch_type",
                    "ptp_domain",
                    "retry_hw",
                ),
            }
        ],
    )

    driver_component_loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    return [container, driver_component_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("sensor_model", description="sensor model name")
    add_launch_arg("config_file", "", description="sensor configuration file")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("setup_sensor", "True", "configure sensor")
    add_launch_arg("retry_hw", "false", "retry hw")
    add_launch_arg("sensor_ip", "192.168.1.201", "device ip address")
    add_launch_arg("host_ip", "255.255.255.255", "host ip address")
    add_launch_arg("scan_phase", "0.0")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("min_range", "0.3", "minimum view range for Velodyne sensors")
    add_launch_arg("max_range", "300.0", "maximum view range for Velodyne sensors")
    add_launch_arg("cloud_min_angle", "0", "minimum view angle setting on device")
    add_launch_arg("cloud_max_angle", "360", "maximum view angle setting on device")
    add_launch_arg("data_port", "2368", "device data port number")
    add_launch_arg("gnss_port", "2380", "device gnss port number")
    add_launch_arg("packet_mtu_size", "1500", "packet mtu size")
    add_launch_arg("rotation_speed", "600", "rotational frequency")
    add_launch_arg("dual_return_distance_threshold", "0.1", "dual return distance threshold")
    add_launch_arg("frame_id", "lidar", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("lidar_container_name", "nebula_node_container")
    add_launch_arg("ptp_profile", "1588v2")
    add_launch_arg("ptp_transport_type", "L2")
    add_launch_arg("ptp_switch_type", "TSN")
    add_launch_arg("ptp_domain", "0")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
    add_launch_arg("enable_blockage_diag", "true")
    add_launch_arg("horizontal_ring_id", "64")
    add_launch_arg("vertical_bins", "128")
    add_launch_arg("is_channel_order_top2down", "true")
    add_launch_arg("horizontal_resolution", "0.4")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
