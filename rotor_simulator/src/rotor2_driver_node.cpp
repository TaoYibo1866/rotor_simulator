#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/Joy.h>

using gazebo_msgs::SetModelConfiguration;
using gazebo_msgs::GetJointProperties;
using sensor_msgs::Joy;
using sensor_msgs::JoyConstPtr;

class LowPassFilter
{
public:
    LowPassFilter(double bandwith_Hz, double sample_Hz)
    {
        prev_val_ = 0;
        tau_ = 1.0 / (2 * M_PI * bandwith_Hz);
        T_ = 1.0 / sample_Hz;
    }
    double proccess(double input)
    {
        double r = tau_ / T_;
        prev_val_ = (input + r * prev_val_) / (1 + r);
        return prev_val_;
    }
private:
    double prev_val_;
    double tau_;
    double T_;
};

ros::Publisher joint_pub;
ros::ServiceClient set_joint_client;
ros::ServiceClient get_joint_client;

SetModelConfiguration set_srv;
GetJointProperties get_srv0, get_srv1;

double angle_cmd_0 = 0.0, angle_cmd_1 = 0.0;
LowPassFilter filter_0(5, 50);
LowPassFilter filter_1(8, 50);

void cmdCallback(const JoyConstPtr& cmd)
{
    ROS_ASSERT(cmd->axes.size() == 2);
    angle_cmd_0 = cmd->axes[0];
    angle_cmd_1 = cmd->axes[1];
}

void timerCallback(const ros::TimerEvent& event)
{
    Joy joint_state;
    joint_state.header.stamp = ros::Time::now();
    if(get_joint_client.call(get_srv0)
    && get_joint_client.call(get_srv1)
    && get_srv0.response.success
    && get_srv1.response.success)
    {
        joint_state.axes.push_back(get_srv0.response.position[0]);
        joint_state.axes.push_back(get_srv1.response.position[0]);
        joint_pub.publish(joint_state);
    }

    set_srv.request.joint_positions[0] = filter_0.proccess(angle_cmd_0);
    set_srv.request.joint_positions[1] = filter_1.proccess(angle_cmd_1);
    set_joint_client.call(set_srv);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotor2_driver");
    ros::NodeHandle nh;
    set_joint_client = nh.serviceClient<SetModelConfiguration>("/gazebo/set_model_configuration");
    get_joint_client = nh.serviceClient<GetJointProperties>("/gazebo/get_joint_properties");
    set_joint_client.waitForExistence();
    get_joint_client.waitForExistence();
    ros::param::param<std::string>("~model_name", set_srv.request.model_name, "rotor2");
    ros::param::param<std::string>("~urdf_param_name", set_srv.request.urdf_param_name, "robot_description");
    std::vector<std::string> joint_names(2);
    ros::param::param<std::string>("~joint0", joint_names[0], "rotor2/pillar_joint");
    ros::param::param<std::string>("~joint1", joint_names[1], "rotor2/load_joint");
    set_srv.request.joint_names = joint_names;
    set_srv.request.joint_positions.resize(2);
    get_srv0.request.joint_name = joint_names[0];
    get_srv1.request.joint_name = joint_names[1];
    ros::Subscriber cmd_sub = nh.subscribe<Joy>("rotor2_angle_cmd", 1, cmdCallback);
    joint_pub = nh.advertise<Joy>("rotor2_angle", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback, false, false);
    ros::Duration(5).sleep(); // wait for model, otherwise gazebo gui stuck on startup
    timer.start();
    ros::spin();
    return 0;
}