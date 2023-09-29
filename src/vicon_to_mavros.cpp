    #include <ros/ros.h>
    #include <tf/transform_listener.h> // possible obsolete
    #include <tf/transform_broadcaster.h>  // possible obsolete
    #include <geometry_msgs/Pose.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <geographic_msgs/GeoPointStamped.h>

    #include <string.h>

    // define the following global variable to pass the pose from callback function to main function
    geometry_msgs::PoseStamped vicon_pose_storage;
    geographic_msgs::GeoPointStamped MELorigin;

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
    }

    int main(int argc, char** argv)
    {
        double output_rate = 30; // output_rate defines how frequent ROS send this message to mavros

        geometry_msgs::PoseStamped msg_body_pose; // use this variable to publish pose

        ros::init(argc, argv, "vicon_to_mavros"); // initialize the node named "vicon_to_mavros"

        ros::NodeHandle nh;

        // check the name of the tracker
        std::string tracker; 
        if(nh.getParam("tracker_name", tracker))
        {
          ROS_INFO("Get tracker_name parameter: %s", tracker.c_str());
        }
        else
        {
          ROS_WARN("Using default tracker_name: %s", tracker.c_str());
        }

       
        // todo: pull the tracker name from launch file parameter
        ros::Subscriber subscribetf = nh.subscribe("/vrpn_client_node/"+tracker+"/pose", 1000, myCallBack); 

        // publishing under the topic name "vicon_pose"
        ros::Publisher camera_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("vicon_pose", 10);

        // publishing the origin information to mavros
        ros::Publisher vicon_set_global_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
        
        // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
        ros::Rate rate(output_rate);

        // new code to correct mavros/vision_pose/pose's yaw rotation of 90 deg (1.57079632679 rad)
        static tf::Quaternion  quat_rot_z, quat_vicon, quat_mavros;
        quat_rot_z = tf::createQuaternionFromRPY(0, 0, 1.57079632679);

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

            // we add 90 deg rotation to the vicon quaternion to compensate for the mavros rotation
            quat_vicon = tf::Quaternion(vicon_pose_storage.pose.orientation.x, vicon_pose_storage.pose.orientation.y, vicon_pose_storage.pose.orientation.z, vicon_pose_storage.pose.orientation.w);
            quat_mavros = quat_rot_z * quat_vicon;
            quat_mavros = quat_mavros.normalize();
             
            msg_body_pose.pose.orientation.x = quat_mavros.getX();
            msg_body_pose.pose.orientation.y = quat_mavros.getY();
            msg_body_pose.pose.orientation.z = quat_mavros.getZ();
            msg_body_pose.pose.orientation.w = quat_mavros.getW();

            // update the GPS origin (todo: set to 1 Hz)
            MELorigin.header.stamp.nsec = ros::Time::now().toSec();
            MELorigin.position.latitude = 40.6892;
            MELorigin.position.longitude = -74.0445;
            MELorigin.position.altitude = 0.0;

            // Publish the pose in ENU
            camera_pose_publisher.publish(msg_body_pose);
            vicon_set_global_origin_pub.publish(MELorigin);

            rate.sleep();
          }
          return 0;
        }
