#include "driver.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>


class OdometryPublisher{


    public:

        std::string child_frame = "base_link";
        std::string odom_frame = "odom";
        ros::NodeHandle nh_;
        ros::Time start_time = ros::Time::now();
        ros::Time prev_time = ros::Time::now();
        double period = 0.03;
        ros::Duration time_elapsed;
        
        // encoder variables
        float ticksPerMeter = 865;
        float DistPerTick = float(1 / ticksPerMeter);
        
        // robot parameters
        float whel_sep = 0.5f;
        float whel_rad = 0.13f;
        
        // encoder callbacks
        void left_enc_callback(const std_msgs::Int64&);
        void right_enc_callback(const std_msgs::Int64&);

        OdometryPublisher();
        void publish_odom();

        // driver library object
        driver::diff_drive drive_base;

        // publisher-subscriber instantiation
        ros::Publisher odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        ros::Subscriber left_enc = nh_.subscribe("left_encoder_ticks", 10, &OdometryPublisher::left_enc_callback, this);
        ros::Subscriber right_enc = nh_.subscribe("right_encoder_ticks", 10, &OdometryPublisher::right_enc_callback, this);
        tf::TransformBroadcaster odom_broadcaster;

    private:

        // encoder ticks variables
        int prevRight_encTicks = 0;
        int right_enc_ticks;
        int prevLeft_encTicks = 0;
        int left_enc_ticks;

        // vectors to hold velocity and pos
        Eigen::Vector3f velocityVec = {0.0f, 0.0f, 0.0f};
        Eigen::Vector3f posVec = {0.0f, 0.0f, 0.0f};




};

OdometryPublisher::OdometryPublisher()
    : drive_base(whel_sep, whel_rad, DistPerTick){ROS_INFO("Odom publishing");};

void OdometryPublisher::publish_odom(){
   
    ros::Time now = ros::Time::now();

    if(now.toSec() - start_time.toSec() > period){

        time_elapsed = now - prev_time;

        drive_base.computePose(prevLeft_encTicks, left_enc_ticks, prevRight_encTicks, right_enc_ticks);

        posVec = posVec + drive_base.getPose();

        drive_base.computeVelocity(drive_base.getlocalPos(), time_elapsed.toSec());

        velocityVec = drive_base.getVelocity();

        // broadcast odometry
        geometry_msgs::Quaternion geom_quat;
        
        geom_quat.x = 0.0;
        geom_quat.y = 0.0;
        geom_quat.z = std::sin(posVec[2] / 2);  
        geom_quat.w = std::cos(posVec[2] / 2);

        tf::Quaternion tf_quat(geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w);
        tf::Transform transform(tf_quat, tf::Vector3(posVec[0], posVec[1], 0));

        odom_broadcaster.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), odom_frame, child_frame)
        );


        // populate odom messages
        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame;
        odom.pose.pose.position.x = posVec[0];
        odom.pose.pose.position.y = posVec[1];
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = geom_quat;
        odom.child_frame_id = child_frame;
        odom.twist.twist.linear.x = (velocityVec[0] + velocityVec[1])/2;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = velocityVec[2];
        odom_pub_.publish(odom);

        prevLeft_encTicks = left_enc_ticks;
        prevRight_encTicks = right_enc_ticks;
        prev_time = now;


     }


};

void OdometryPublisher::left_enc_callback(const std_msgs::Int64& left_enc){

    left_enc_ticks = left_enc.data;


}

void OdometryPublisher::right_enc_callback(const std_msgs::Int64& right_enc){

    right_enc_ticks = right_enc.data;


}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;

    OdometryPublisher OdomPub;

    ros::Rate rate(2);

    while (ros::ok()) {
        OdomPub.publish_odom();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}



