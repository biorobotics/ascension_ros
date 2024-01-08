#include <ros/ros.h>
#include "ascension/Sensor_poses.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h" // Include the necessary message type for clearing the visualization
#include <vector>

ros::Publisher marker_pub;
std::vector<std::vector<geometry_msgs::Point>> pointsArray; // Global Array of Subarray
int sensors_number; // Global Variable of Number of Sensors
int max_points = 7200;


void clearCallBack(const std_msgs::String::ConstPtr& msg)
{
  // Clear the points vector to remove all points from the line strip

  for (int i = 0; i < sensors_number; i ++) {
    pointsArray[i].clear();
  }

  ROS_INFO("Path Cleared");

}

void sensorCallback(const ascension::Sensor_poses::ConstPtr& my_msg)
{
  for (int i = 0; i < sensors_number; i++) {
    // Extract position data from each sensor
    geometry_msgs::Point point;
    point.x = my_msg->sensors[i].transform.translation.x;
    point.y = my_msg->sensors[i].transform.translation.y;
    point.z = my_msg->sensors[i].transform.translation.z;

    // Add the point to the vector of points
    pointsArray[i].push_back(point);
    while (pointsArray[i].size() > max_points) {
      pointsArray[i].erase(pointsArray[i].begin());
    }

    // Create a unique namespace for each marker
    std::string marker_namespace = "sensor_" + std::to_string(i);

    // Create a line strip marker with a unique ID for each marker
    visualization_msgs::Marker marker;
    marker.header = my_msg->sensors[i].header;
    marker.ns = marker_namespace;  // Set a unique namespace
    marker.id = i;  // Set a unique ID for each marker
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.005; // Line width
    marker.points = pointsArray[i]; // Set the points for the line strip

    // Set different colors for each marker
    marker.color.r = static_cast<float>(i) / static_cast<float>(sensors_number);  // Vary the red component
    marker.color.g = 1.0 - static_cast<float>(i) / static_cast<float>(sensors_number);  // Vary the green component
    marker.color.b = 0.0;  // Keep the blue component constant
    marker.color.a = 1.0;

    // Publish the marker with a unique namespace, ID, and color
    marker_pub.publish(marker);
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_poses_listener");
  ros::NodeHandle n;

  n.getParam("/ascension_listener/num_sensor", sensors_number);

  pointsArray.resize(sensors_number);

  ros::Subscriber sub = n.subscribe("sensors_poses", 10, sensorCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Create a subscriber to listen for the clear visualization trigger
  ros::Subscriber clear_sub = n.subscribe("clear_visualization", 10, clearCallBack);

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}