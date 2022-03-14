#include <math.h>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <pix_robot_can/SetRobotMotion.h>

#define PI 3.14159265

static mavros_msgs::State current_state;
static mavros_msgs::RCIn console_msg;
static ros::Publisher pub;
//static pix_robot_can::SetMotorSpeed motor_speed;
static pix_robot_can::SetRobotMotion robot_motion;
//static double pi = 3.1416;
static double radius_of_wheel = 0.127;
static double width = 0.430;
static double length = 0.444;
static double linear_velocity, steer_angle, angular_velocity;

static double median = 1515.0;  //The median value of remote control (min=1102 max=1927)
static double mid   = 412.5;    //mid = Maximum minus minimum / 2


static void vehicle_cmd_callback(const mavros_msgs::RCInConstPtr &msg)
{
    linear_velocity = (msg->channels[2] - median) / mid * -1.0;
    // std::cout << linear_velocity << std::endl;

    // steer_angle = (msg->channels[0] - 1515.0) / 412.5 * -1.0;

    angular_velocity = (msg->channels[0] - median)/mid * -1.0;
    // std::cout << steer_angle << std::endl;
}

int velocity_to_rmp(double speed)
{
    int rmp;
    rmp = speed * 60 / (radius_of_wheel * PI) * 2;
    return rmp;
}

int compute_wheel_rmp(double linear_velocity, double steer_angle)
{
    double left_velocity, right_velocity;
    int left_rmp, right_rmp;

    if (steer_angle > 0)
    {
        left_velocity = (1 - (width/length) * sin(steer_angle)/2) * linear_velocity;
        right_velocity = 2 * linear_velocity - left_velocity;
    }
    else if (steer_angle < 0)
    {
        steer_angle = -steer_angle;
        right_velocity = (1 - (width/length) * sin(steer_angle)/2) * linear_velocity;
        left_velocity = 2 * linear_velocity - right_velocity;
    }
    left_rmp = velocity_to_rmp(left_velocity);
    right_rmp = velocity_to_rmp(right_velocity);
    return left_rmp;
}

int compute_right_rmp(double linear_velocity, int left_rmp)
{
    int rmp;
    rmp = velocity_to_rmp(linear_velocity);
    int right_rmp;
    right_rmp  = 2 * rmp - left_rmp;
    return right_rmp;
}
/*
void timer_callback(const ros::TimerEvent &te)
{
    int left_rmp, right_rmp;

    motor_speed.header.stamp = ros::Time::now();
    left_rmp = compute_wheel_rmp(linear_velocity, steer_angle);
    right_rmp = compute_right_rmp(linear_velocity, left_rmp);
    motor_speed.motorSpeed[0] = left_rmp;
    motor_speed.motorSpeed[1] = right_rmp;
    motor_speed.motorSpeed[2] = left_rmp;
    motor_speed.motorSpeed[3] = right_rmp;

    pub.publish(motor_speed);
    
}
*/

static void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void timer_callback(const ros::TimerEvent &te)
{

    robot_motion.header.stamp = ros::Time::now();


        if( current_state.mode == "MANUAL" && current_state.armed ){
                
                ROS_INFO("Vehicle enabled");
                robot_motion.velocity = linear_velocity;
                robot_motion.angularVelocity = angular_velocity;
            
            robot_motion.header.stamp = ros::Time::now();
        } else {
            if( !current_state.armed ){
                
                ROS_INFO("Vehicle armed");                                    
                robot_motion.header.stamp = ros::Time::now();
            }
        }

    pub.publish(robot_motion);
    
    
}



int main(int argc, char* argv[])
{
    ROS_INFO("Starting...");
    ros::init(argc, argv, "heisen_autoware_bridge");
    ros::NodeHandle nh;

    pub = nh.advertise<pix_robot_can::SetRobotMotion>("/pix/set_robot_motion", 1);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber sub = nh.subscribe("/mavros/rc/in", 1, vehicle_cmd_callback);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    double pub_rate = 20;


    ros::Timer set_speed = nh.createTimer(ros::Duration(1/pub_rate), timer_callback);

    ros::spin();
    return 0;

}
