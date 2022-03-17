# ROS Interface for ATI Force/Torque Sensor Digital Interface

This application receives force/torque data on UDP and publishes them on a ROS topic. It is a companion to the [ATI FT Sensor Windows Application](https://github.com/keipour/ATI-Digital-FT-Server) which connects to the ATI Sensor (tested on ATI Gamma) via the digital interface and publishes the sensor data on UDP.

Each UDP message is a 6-element int32 array ([fx fy fz tx ty tz]), where each each element is multiplied by 1,000. This node converts the data to `geometry_msgs/WrenchStamped` ROS messages and publishes to `ft_data` topic.

The combination of this node and the Windows sensor interface companion is a workaround for reading the sensor data via the ATI F/T digital interface, which currently does not directly support Debian and only has a library for Windows .NET environment.

## Installation:

First, install [data transmission library](https://github.com/tbs-ualberta/data_transmission.git).

Then, clone this repository in the `src` folder of your catkin workspace (e.g., in `~\catkin_ws\src\` folder):

```
git clone git@github.com:keipour/ft_ros_interface.git
```

## Usage

After `catkin build` and sourcing `devel\setup.bash` is done, the node can be launched by the following command:

```
roslaunch ft_ros_interface ft_ros_interface.launch
```

Note that there are four parameters in the launch file to control the program:

- *ip_address*: Local IP v4 address. Note that "127.0.0.1" does not work for listening to the Force/Torque data publishing application on Windows and the actual IP address should be used (``String``).
- *udp_port*: Local port (``int``). Can be set to any free UDP port on the system.
- *buffer_size*: Length of the UDP socket's receive buffer (SO_RCVBUF) (``int``).
- *poll_rate*: The polling rate in [Hz] for the maximum rate with which it polls the UDP server for new data (``int``).

For every data received by the program on the UDP port, a ROS message of type `geometry_msgs\WrenchStamped` is published on `ft_data` topic.

## Contact
Azarakhsh Keipour (keipour@gmail.com)