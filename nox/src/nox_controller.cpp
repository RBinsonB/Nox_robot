#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>

double radius = 0.04;                              //Wheel radius, in m
double wheelbase = 0.187;                          //Wheelbase, in m
double two_pi = 6.28319;
double speed_act_left = 0.0;
double speed_act_right = 0.0;
double speed_req1 = 0.0;
double speed_req2 = 0.0;
double speed_dt = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
ros::Time current_time;
ros::Time speed_time(0.0);

void handle_speed( const geometry_msgs::Vector3Stamped& speed) {
  speed_act_left = trunc(speed.vector.x*100)/100;
  ROS_INFO("speed left : %f", speed_act_left);
  speed_act_right = trunc(speed.vector.y*100)/100;
  ROS_INFO("speed right : %f", speed_act_right);
  speed_dt = speed.vector.z;
  speed_time = speed.header.stamp;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "nox_controller");

  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("speed", 50, handle_speed);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;  
  
  double rate = 10.0;
  double linear_scale_positive = 1.0;
  double linear_scale_negative = 1.0;
  double angular_scale_positive = 1.0;
  double angular_scale_negative = 1.0;
  bool publish_tf = true;
  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;
  double dxy = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  char base_link[] = "/base_link";
  char odom[] = "/odom";
  char kinect[] = "/kinect";
  char camera_link[] = "/camera_link";
  ros::Duration d(1.0);
  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("publish_tf", publish_tf);
  nh_private_.getParam("linear_scale_positive", linear_scale_positive);
  nh_private_.getParam("linear_scale_negative", linear_scale_negative);
  nh_private_.getParam("angular_scale_positive", angular_scale_positive);
  nh_private_.getParam("angular_scale_negative", angular_scale_negative);

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    current_time = speed_time;
    dt = speed_dt;					//Time in s
    ROS_INFO("dt : %f", dt);
    dxy = (speed_act_left+speed_act_right)*dt/2;
    ROS_INFO("dxy : %f", dxy);
    dth = ((speed_act_right-speed_act_left)*dt)/wheelbase;

    if (dth > 0) dth *= angular_scale_positive;
    if (dth < 0) dth *= angular_scale_negative;
    if (dxy > 0) dxy *= linear_scale_positive;
    if (dxy < 0) dxy *= linear_scale_negative;

    dx = cos(dth) * dxy;
    dy = sin(dth) * dxy;

    x_pos += (cos(theta) * dx - sin(theta) * dy);
    y_pos += (sin(theta) * dx + cos(theta) * dy);
    theta += dth;

    if(theta >= two_pi) theta -= two_pi;
    if(theta <= -two_pi) theta += two_pi;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::Quaternion empty_quat = tf::createQuaternionMsgFromYaw(0);

    if(publish_tf) {
      geometry_msgs::TransformStamped t;
      geometry_msgs::TransformStamped k;
      
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;
      
      k.header.frame_id = kinect;
      k.child_frame_id = camera_link;
      k.transform.translation.x = 0.0;
      k.transform.translation.y = 0.0;
      k.transform.translation.z = 0.0;
      k.transform.rotation = empty_quat;
      k.header.stamp = current_time;

      broadcaster.sendTransform(t);
      broadcaster.sendTransform(k);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (speed_act_left == 0 && speed_act_right == 0){
      odom_msg.pose.covariance[0] = 1e-9;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 1e-9;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e-9;
      odom_msg.twist.covariance[0] = 1e-9;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 1e-9;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      odom_msg.pose.covariance[0] = 1e-3;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 0.0;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e3;
      odom_msg.twist.covariance[0] = 1e-3;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 0.0;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e3;
    }
    vx = (dt == 0)?  0 : (speed_act_left+speed_act_right)/2;
    vth = (dt == 0)? 0 : (speed_act_right-speed_act_left)/wheelbase;
    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dth;

    odom_pub.publish(odom_msg);
    r.sleep();
  }
}
