#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

using sensor_msgs::Joy;

ros::Publisher cmd_pub;

double omega_0 = 0;
double omega_1 = 0;
double theta_0 = 0;
double theta_1 = 0;

void timerCb(const ros::TimerEvent &event)
{
  Joy cmd;
  theta_0 += omega_0 * 0.02;
  theta_1 += omega_1 * 0.02;
  cmd.axes.push_back(theta_0);
  cmd.axes.push_back(theta_1);
  cmd_pub.publish(cmd);
}

void joyCb(Joy joy)
{
  omega_0 = 0.2 * joy.axes[3] * M_PI / 3;
  omega_1 = 0.2 * -joy.axes[4] * M_PI / 4;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotor2_joy");
  ros::NodeHandle nh;
  ros::Subscriber joy_sub = nh.subscribe<Joy>("/joy", 1, joyCb);
  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCb);
  cmd_pub = nh.advertise<Joy>("rotor2_angle_cmd", 1);
  ros::spin();
  return 0;
}