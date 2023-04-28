    #include <ros/ros.h>
    #include <tf/transform_listener.h> // possible obsolete
    #include <tf/transform_broadcaster.h>  // possible obsolete
    #include <geometry_msgs/Pose.h>
    #include <geometry_msgs/PoseStamped.h>

    #include <string.h>

    // define the following global variable to pass the pose from callback function to main function
    geometry_msgs::PoseStamped vicon_pose_storage;

    std::vector<geometry_msgs::PoseStamped::ConstPtr> pose1;  // possibly obsolete

    void myCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // test receiving of Vicon packages
        // ROS_INFO_STREAM("Received pose: " << msg);
        //ROS_INFO_STREAM(msg->pose.position.x);   

        // save the message header
        vicon_pose_storage.header = msg->header;
        // save the pose
        vicon_pose_storage.pose = msg->pose;

        // possibly obsolete
        pose1.push_back(msg);
    }

    int main(int argc, char** argv)
    {
        double output_rate = 30; // output_rate defines how frequent ROS send this message to mavros

        geometry_msgs::PoseStamped msg_body_pose; // use this variable to publish pose

        ros::init(argc, argv, "vicon_to_mavros"); // initialize the node named "vicon_to_mavros"

        ros::NodeHandle nh;

        // todo: pull the tracker name from launch file parameter
        ros::Subscriber subscribetf = nh.subscribe("/vrpn_client_node/Q2s/pose", 1000, myCallBack); 

        // publishing under the topic name "vicon_pose"
        ros::Publisher camera_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("vicon_pose", 10);
        
        // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
        ros::Rate rate(output_rate);

        while (nh.ok())
         {
            ros::spinOnce();
            
            // copy the header
            msg_body_pose.header = vicon_pose_storage.header;
            
            // copy the pose while conducting    
            // coordinate transform from vrpn coordinates (NWU) to vicon coordiantes (ENU)
            // ardupilot will convert the ENU to NED by itself
            msg_body_pose.pose.position.x = -vicon_pose_storage.pose.position.y; 
            msg_body_pose.pose.position.y = vicon_pose_storage.pose.position.x; 
            msg_body_pose.pose.position.z = vicon_pose_storage.pose.position.z;
            msg_body_pose.pose.orientation = vicon_pose_storage.pose.orientation;

            // Publish the pose in ENU
            camera_pose_publisher.publish(msg_body_pose);

            rate.sleep();
          }
          return 0;
        }
