#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Joy.h>

using sensor_msgs::Joy;

image_transport::Publisher img_pub;
ros::Publisher cmd_pub;

void imgCb(const sensor_msgs::ImageConstPtr& img_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img = cv_ptr->image;

  // DO VISUAL SERVO HERE
  cv::Rect rect((img.cols - img.rows / 3.0) / 2.0, img.rows / 3.0, img.rows / 3.0, img.rows / 3.0);
  cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 10);
  Joy cmd;
  cmd.axes.resize(2);
  cmd.axes[0] = 0.1; // Z axis rotation (rad)
  cmd.axes[1] = -0.1; // Y axis rotation (rad)
  // VISUAL SERVO FINISH

  cmd_pub.publish(cmd);
  img_pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servo");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("/camera/image_raw", 1, imgCb);
  img_pub = it.advertise("image_output", 1);
  cmd_pub = nh.advertise<Joy>("rotor2_angle_cmd", 1);
  ros::spin();
  return 0;
}