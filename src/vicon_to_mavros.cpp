#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>
// #include <mavros_msgs/LandingTarget.h>

#include <string.h>

double position_x = 0;
double position_y = 0;
double position_z = 0;

double quaternion_x = 0;
double quaternion_y = 0;
double quaternion_z = 0;
double quaternion_w = 0;

//std_msgs::time stamp;
//std_msgs::string frame_id;

geometry_msgs::PoseStamped vicon_pose_storage;

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose1;

void myCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO_STREAM("Received pose: " << msg);
    vicon_pose_storage.header = msg->header;
    //frame_id = msg->header.frame_id;
    vicon_pose_storage.pose = msg->pose;
    //position_x = msg->pose.position.x; 
    //position_y = msg->pose.position.y;
    //position_z = msg->pose.position.z;
    
    //quaternion_x = msg->pose.orientation.x;
    //quaternion_y = msg->pose.orientation.y;
    //quaternion_z = msg->pose.orientation.z;
    //quaternion_w = msg->pose.orientation.w;

    //ROS_INFO_STREAM(position_x);
    pose1.push_back(msg);
}

int main(int argc, char** argv)
{
    double output_rate = 20;
  geometry_msgs::PoseStamped msg_body_pose;
  ros::init(argc, argv, "vicon_to_mavros");
    ros::NodeHandle nh;
    ros::Subscriber subscribetf = nh.subscribe("/vrpn_client_node/Q2s/pose", 1000, myCallBack);
  ros::Publisher camera_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("vicon_pose", 10);
    //ros::spin();
    // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
ros::Rate rate(output_rate);
while (nh.ok())
 {
    ros::spinOnce();
    
    // Create PoseStamped message to be sent
        msg_body_pose.header = vicon_pose_storage.header;
//        msg_body_pose.header.frame_id = frame_id;
        
// coordinate transform from vrpn coordinates (NWU) to vicon coordiantes (ENU)
        msg_body_pose.pose.position.x = -vicon_pose_storage.pose.position.y; 
        msg_body_pose.pose.position.y = vicon_pose_storage.pose.position.x; 
        msg_body_pose.pose.position.z = vicon_pose_storage.pose.position.z;
        msg_body_pose.pose.orientation = vicon_pose_storage.pose.orientation;
        //msg_body_pose.pose.orientation.y = quaternion_y;
        //msg_body_pose.pose.orientation.z = quaternion_z;
        //msg_body_pose.pose.orientation.w = quaternion_w;
;
        // Publish pose of body frame in world frame
        camera_pose_publisher.publish(msg_body_pose);

    rate.sleep();
  }
  return 0;
}
