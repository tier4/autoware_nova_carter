# Autoware Nova Carter Interface

The `autoware_nova_carter` package is part of the Autoware project, designed to interface with the Nova Carter vehicle platform. It provides nodes and configurations for controlling and sensing the vehicle in various environments.

## Features

- Interface with Nova Carter vehicle for control and sensing.

## Installation

1. **Clone the repository:**

   ```bash
   cd ~/your_ros_workspace/src/
   git clone https://github.com/tier4/autoware_nova_carter.git
   ```

2. **Build the package:**

   Navigate to your ROS workspace and build the package:

   ```bash
   cd ~/your_ros_workspace
   colcon build --packages-up-to autoware_nova_carter_interface
   ```

## Parameters

The interface node uses several parameters that can be configured in the `config/nova_carter_interface.param.yaml` file. Key parameters include:

- `base_frame_id`: The frame ID for the vehicle's base link.
- `maximum_linear_velocity`: Maximum allowable linear velocity.
- `maximum_angular_velocity`: Maximum allowable angular velocity.
- `wheel_base`: The distance between the front and rear axles.

## Inputs and Outputs

### Subscribers

| Topic Name   | Message Type                  | Description                                      |
|--------------|-------------------------------|--------------------------------------------------|
| `control_cmd` | `autoware_msgs/ControlCommand` | Control commands from Autoware.                  |
| `odom`        | `nav_msgs/Odometry`           | Odometry messages from the Nova Carter vehicle.  |

### Publishers

| Topic Name             | Message Type                    | Description                                                                 |
|------------------------|---------------------------------|-----------------------------------------------------------------------------|
| `cmd_vel`              | `geometry_msgs/TwistStamped`    | Twist command which will be sent to the vehicle.                            |
| `vehicle_twist_report` | `autoware_msgs/VelocityReport`  | Current velocity information reported to Autoware.                          |
| `steering_status_report` | `autoware_msgs/SteeringReport` | Current steering angle reported to Autoware. Nova Carter is a differential drive vehicle, so the steering angle is virtually calculated from the linear and angular velocities. |

## Usage

### Launching the Interface

To launch the Nova Carter interface, use the following command:

   ```bash
   ros2 launch autoware_nova_carter_interface nova_carter_interface.launch.xml
   ```