// UDP socket includes
#include <data_transmission/data_transmission.h>
#include <string.h>

// ROS includes
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "ft_ros_interface");
  ros::NodeHandle n;

  ros::Publisher data_pub = n.advertise<geometry_msgs::WrenchStamped>("ft_data", 1);

  // --- Obtain parameters ---
  int rate_hz = 100;
  ros::Rate loop_rate(rate_hz);
  n.getParam("/ft_ros_interface/poll_rate", rate_hz);
  string ip_local_st = "172.28.103.156";
  n.getParam("/ft_ros_interface/ip_address", ip_local_st);
  int port_local_si = 7778;
  n.getParam("/ft_ros_interface/udp_port", port_local_si);
  int len_rcvbuf = 10;
  n.getParam("/ft_ros_interface/buffer_size", len_rcvbuf);

  // --- Setup the transmission ---
  data_transmission transmission;
  char ip_local_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  if (transmission.init_transmission(ip_local_scp, port_local_si, len_rcvbuf) == -1)
  {
    ROS_ERROR("Error starting the UDP Server on %s:%d", ip_local_scp, port_local_si);
    return 0;
  }
  ROS_INFO("Listening for FT data on %s:%d", ip_local_scp, port_local_si);

  char message[1024];
  while (ros::ok()) {
    // TODO: Make the below command non-blocking (small timeout)
    transmission.listen(message, sizeof(int32_t) * 6);

    // Convert the received message to int32 array
    int32_t *message_dp = (int32_t *)message;

    // --- Assemble & publish the message ---
    geometry_msgs::WrenchStamped msg_wrench;
    msg_wrench.header.stamp = ros::Time::now();
    msg_wrench.header.frame_id = "ft_sensor";
    msg_wrench.wrench.force.x = message_dp[0] / 1e3f;
    msg_wrench.wrench.force.y = message_dp[1] / 1e3f;
    msg_wrench.wrench.force.z = message_dp[2] / 1e3f;
    msg_wrench.wrench.torque.x = message_dp[3] / 1e3f;
    msg_wrench.wrench.torque.y = message_dp[4] / 1e3f;
    msg_wrench.wrench.torque.z = message_dp[5] / 1e3f;
    data_pub.publish(msg_wrench);

    ros::spinOnce();
    loop_rate.sleep();
  }
  transmission.close_transmission();
  return 0;
}
