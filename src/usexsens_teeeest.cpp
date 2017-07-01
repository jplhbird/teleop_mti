/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"



/**
 * This tutorial demonstrates simple usage of mit-g-710, using it to command the turtle in the ROS package
 */



using namespace std;
class TeleopIMU{
public:
    TeleopIMU();
private:
    void callBack(const sensor_msgs::Imu::ConstPtr& imu);
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Publisher pub2;

    ros::Publisher pub3;

    ros::Subscriber sub;

};

TeleopIMU::TeleopIMU()
{
    pub=n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);


    pub2=n.advertise<geometry_msgs::PoseStamped>("command/pose",1); //command to quadrotor

    pub3 = n.advertise<geometry_msgs::TwistStamped>("command/twist",1); //velocity command to quadrotor

    sub=n.subscribe<sensor_msgs::Imu>("/imu/data",10,&TeleopIMU::callBack,this);
    //should run rosrun xsens_driver mtnode.py in order to run the xsens_driver node,
    //this nod will advertise /imu/data
    //topic

}

void TeleopIMU::callBack(const sensor_msgs::Imu::ConstPtr& imu)
{
    geometry_msgs::Twist vel;
//    geometry_msgs/Vector3 linear
//      float64 x
//      float64 y
//      float64 z
//    geometry_msgs/Vector3 angular
//      float64 x
//      float64 y
//      float64 z
 //   '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'

    vel.linear.x = imu->angular_velocity.x;
    vel.linear.y = imu->angular_velocity.y;
    vel.linear.z = imu->angular_velocity.z;
    vel.angular.x = imu->angular_velocity.x;
    vel.angular.y = imu->angular_velocity.y;
    vel.angular.z = imu->angular_velocity.z;


    //vel.linear = '[2.0, 0.0, 0.0]';
   // vel.angular = '[2.0, 0.0, 0.0]';
    pub.publish(vel);

    //added on October, 10, 2016
    //the message is command to the simulated quadrotor in gazebo
    geometry_msgs::PoseStamped postoquadrotor;
//    std_msgs/Header header
//      uint32 seq
//      time stamp
//      string frame_id
//    geometry_msgs/Pose pose
//      geometry_msgs/Point position
//        float64 x
//        float64 y
//        float64 z
//      geometry_msgs/Quaternion orientation
//        float64 x
//        float64 y
//        float64 z
//        float64 w

    postoquadrotor.pose.position.x = 1;
    postoquadrotor.pose.position.y=1;
    postoquadrotor.pose.position.z =1;
    postoquadrotor.pose.orientation.x= imu->orientation.x;
    postoquadrotor.pose.orientation.y=imu->orientation.y;
    postoquadrotor.pose.orientation.z=imu->orientation.z;
    postoquadrotor.pose.orientation.w=imu->orientation.w;
    postoquadrotor.header.seq = imu->header.seq;
    postoquadrotor.header.stamp = imu->header.stamp;
   // postoquadrotor.header.frame_id = imu->header.frame_id;
    postoquadrotor.header.frame_id  =   "world";



    //publish the massege
    pub2.publish(postoquadrotor);



    geometry_msgs::TwistStamped velquadrotor;
//    std_msgs/Header header
//      uint32 seq
//      time stamp
//      string frame_id
//    geometry_msgs/Twist twist
//      geometry_msgs/Vector3 linear
//        float64 x
//        float64 y
//        float64 z
//      geometry_msgs/Vector3 angular
//        float64 x
//        float64 y
//        float64 z

    velquadrotor.header.frame_id ="world";
    velquadrotor.header.stamp =imu->header.stamp;
    velquadrotor.header.seq = imu->header.seq;

    velquadrotor.twist.linear.x = imu->angular_velocity.x;
    velquadrotor.twist.linear.y = imu->angular_velocity.y;
    velquadrotor.twist.linear.z = imu->angular_velocity.z;
    velquadrotor.twist.angular.x = imu->angular_velocity.x;
    velquadrotor.twist.angular.y = imu->angular_velocity.y;
    velquadrotor.twist.angular.z = imu->angular_velocity.z;

    //publish the massege
     pub3.publish(velquadrotor);





}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_mti_node");
    TeleopIMU teleop_turtle;
    ros::spin();
}
// %EndTag(FULLTEXT)%

