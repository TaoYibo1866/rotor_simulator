#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

using sensor_msgs::Joy;

ros::Publisher cmd_pub;

void joyCb(Joy joy)
{
  Joy cmd;
  cmd.axes.push_back(0.2 * joy.axes[3] * M_PI / 3);
  cmd.axes.push_back(0.2 * -joy.axes[4] * M_PI / 4);
  cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotor2_joy");
  ros::NodeHandle nh;
  ros::Subscriber joy_sub = nh.subscribe<Joy>("/joy", 1, joyCb);
  cmd_pub = nh.advertise<Joy>("rotor2_twist_cmd", 1);
  ros::spin();
  return 0;
}