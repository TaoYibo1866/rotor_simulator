#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using nav_msgs::Odometry;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_drone");
  double phase, omega, a, b, x_offset, y_offset, z_offset;
  ros::param::param<double>("~phase", phase, -M_PI_2);
  ros::param::param<double>("~omega", omega, 0.3);
  ros::param::param<double>("~a", a, 8);
  ros::param::param<double>("~b", b, 1);
  ros::param::param<double>("~x_offset", x_offset, 8);
  ros::param::param<double>("~y_offset", y_offset, 0);
  ros::param::param<double>("~z_offset", z_offset, 2.5);
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<Odometry>("hil_odom", 1);
  Odometry odom;
  odom.pose.pose.position.x = x_offset;
  odom.pose.pose.position.y = y_offset;
  odom.pose.pose.position.z = z_offset;
  odom.pose.pose.orientation.w = 1.0;
  double t = 0;
  ros::Rate r(50);
  while (ros::ok())
  {
    odom.pose.pose.position.y = y_offset + a * cos(omega * t + phase);
    odom.pose.pose.position.z = z_offset + b * sin(omega * t + phase);
    odom_pub.publish(odom);
    t += 1.0 / 50;
    r.sleep();
  }
  return 0;
}