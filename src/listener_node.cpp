/*
 * listener_node look up the transform from snake to target sensor and publish the
 * transformation under te topic sensors poses using a custom message type which is
 * an array of TransformStamped
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "ascension/Sensor_poses.h"
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle node;
    std::string sensor_number_str;
    int sensors_number;

    node.getParam("/ascension_listener/num_sensor", sensors_number);
    std::vector<std::string> target_frame_ids;
    node.getParam("/ascension_listener/target_frame_ids", target_frame_ids);

    if(target_frame_ids.empty()) {
        ROS_ERROR("target_frame_id is empty, please specify in launch file");
    }

    tf::TransformListener listener;
    ros::Rate rate(100.0);

    ros::Publisher pub = node.advertise<ascension::Sensor_poses>("sensors_poses", 10);
    
    while (node.ok()){
        ascension::Sensor_poses my_msg;
        for(int i = 0; i < sensors_number; i++) {
            
	        tf::StampedTransform transform;
            try{
                listener.lookupTransform(target_frame_ids[i], "/snake", 
                                        ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            geometry_msgs::TransformStamped msg;
            tf::transformStampedTFToMsg(transform, msg);
            my_msg.sensors.push_back(msg);
            // pub.publish(msg);
            rate.sleep();
        }
        pub.publish(my_msg);
    }
    return 0;
}