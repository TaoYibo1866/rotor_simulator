#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Joy.h>
#include "KCF/kcftracker.hpp"
#include <iostream>
#include <fstream>

using sensor_msgs::Joy;
using std::vector;
using namespace cv;

enum{
  DETECT,
  TRACK
};

int status = DETECT;
KCFTracker tracker(true, false, true, true);
image_transport::Publisher img_pub;
ros::Publisher cmd_pub;
Joy cmd;
Rect2d roi;
float peak_value=0;
vector<double> K{320,240,2117,2117};
int count=0;
double g_i_x = 0;
double g_i_y = 0;

void viewer(Mat &frame, int status, Rect2d roi, float peak_value)
{
  Point up(320,225);
  Point down(320,255);
  Point left(305,240);
  Point right(335,240);
  line(frame, up, down,Scalar(0, 0, 255),2);
  line(frame, left, right,Scalar(0, 0, 255),2);
  if (status == DETECT)
  {
    putText(frame, "DETECTING", Point(10,20), FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 255), 2, 8, 0);
    putText(frame, "pitch error:",Point(10,40) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
    putText(frame, "yaw error:",Point(10,60) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
    putText(frame, "area:",Point(10,80) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
    putText(frame, "peak_value:",Point(10,100) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
  }
  else
  {
    Point center = 0.5 * (roi.tl() + roi.br());
    char s[100];
    putText(frame, "TRACKING",Point(10,20) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 255), 2, 8, 0);
    sprintf(s, "pitch error:%d", center.x - 320);
    putText(frame, s,Point(10,40) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
    sprintf(s, "yaw error:%d", center.y - 240);
    putText(frame, s,Point(10,60) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
    sprintf(s, "area:%d", (int)roi.area());
    putText(frame, s,Point(10,80) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
    sprintf(s, "peak_value:%.2f", peak_value);
    putText(frame, s,Point(10,100) , FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 0), 2, 8, 0);
  }
  return;
}

double generatePI(double input, double& integral, double t, double Kp=1.0, double Ki=0.0)
{
  integral += input * t;
  return Kp * input + Ki * integral;
}

bool cmdGenerate(Rect2d roi,Joy& cmd,vector<double> K)
{
    Point center = 0.5 * (roi.tl() + roi.br());
    double err_0 = -atan((center.x-K[0])/K[2]);
    double err_1 = atan((center.y-K[1])/K[3]);
    cmd.axes[0] = generatePI(err_0, g_i_x, 1.0/30, 10, 6); // Z axis rotation (rad)
    cmd.axes[1] = generatePI(err_1, g_i_y, 1.0/30, 10, 6); // Y axis rotation (rad)
}

bool detect(Mat frame,Rect2d& roi)
{
    Mat tmp;
    vector<vector<Point> > counters;
    Scalar l_range(0, 0, 0), h_range(180, 255, 46);
    Rect2d roi_t;
    cvtColor(frame,tmp,CV_BGR2HSV);
    inRange(tmp,l_range,h_range,tmp);
    dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point2i(2));
    findContours(tmp,counters,RETR_EXTERNAL,CHAIN_APPROX_NONE);
    if(counters.size()==0) {return false;}
    double t=0;
    int k=0;
    for (int i = 0; i < counters.size(); i++)
    {
      roi_t=boundingRect(counters[i]);
      if(t<roi_t.area())
      {
        t=roi_t.area();
        k=i;
      }
    }
    roi_t=boundingRect(counters[k]);
    if(roi_t.area()<=1500)
    {
      return false;
    }
    roi=roi_t;
    return true; 
}

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
  Mat img = cv_ptr->image;
  
  switch(status)
  {
    case DETECT:
    {
      if(detect(img,roi))
      {
        status = TRACK;
        tracker.init(roi,img);
        rectangle(img, roi, Scalar(0, 0, 255), 5);
        cmdGenerate(roi,cmd,K);
        break;
      }
      cmd.axes[0] = 0;
      cmd.axes[1] = 0;
      g_i_x = 0;
      g_i_y = 0;
      break;
    }
      
    case TRACK:
    {
      roi = tracker.update(img,peak_value);
      if(peak_value>0.4 && count<10)
      {
        rectangle(img, roi, Scalar(0, 0, 255), 5);
        cmdGenerate(roi,cmd,K);
        count++;
        break;
      }
      count =0;
      cmd.axes[0] = 0;
      cmd.axes[1] = 0;
      status =DETECT;
    break;
    }  
  
  }  
  // VISUAL SERVO FINISH
  cmd_pub.publish(cmd); 
  viewer(img, status, roi, peak_value);
  img_pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servo");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("/camera/image_raw", 1, imgCb);
  img_pub = it.advertise("image_output", 1);
  cmd_pub = nh.advertise<Joy>("rotor2_twist_cmd", 1);
  cmd.axes.resize(2);
  ros::spin();
  return 0;
}