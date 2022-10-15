/*
 * transform node set up two static transformation which include a transform
 * from the original ascension frame to ascension prime, which is a frame 
 * that make visualization easier, and a the transformation from
 * snake to the EM ascension tracker specified manually in .yaml file, 
 * in form of [vector[3], quaternion[4]]
 */

#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_node");
    ros::NodeHandle nh("~");
    std::vector<double> snake_EM;
    nh.getParam("/transformation", snake_EM);
    if(snake_EM.size() < 4){
        ROS_INFO("no orientation parameters are given, default to 0, 0, 0, 1");
        snake_EM.push_back(0);
        snake_EM.push_back(0);
        snake_EM.push_back(0);
        snake_EM.push_back(1);
    }
    // ROS_INFO("received value: ");
    // ROS_INFO(snake_EM[0], snake_EM[1], snake_EM[2], snake_EM[3], snake_EM[4], snake_EM[5], snake_EM[6]);
    tf::Transform snake_to_ascension;
    tf::Transform prime_to_origin;
    tf::Transform world_to_snake;
    tf::Transform snake_to_base_link;

    tf::TransformBroadcaster br;

    ros::Rate rate(10.0);
    while (nh.ok()){
        // set transform from ascension prime to EM ascension
        prime_to_origin.setOrigin(tf::Vector3(0,0,0));
        prime_to_origin.setRotation(tf::Quaternion(1,0,0,0));
        br.sendTransform(tf::StampedTransform(prime_to_origin, ros::Time::now(), "ascension_prime", "ascension_origin"));

        // set transform from snake to ascension_prime
        snake_to_ascension.setOrigin( tf::Vector3(snake_EM[0], snake_EM[1], snake_EM[2]) );
        snake_to_ascension.setRotation( tf::Quaternion(snake_EM[3], snake_EM[4], snake_EM[5], snake_EM[6]) );
        br.sendTransform(tf::StampedTransform(snake_to_ascension, ros::Time::now(), "snake", "ascension_prime"));

        // set transform from world to snake, which places the snake on world
        world_to_snake.setOrigin( tf::Vector3(0,0,0) );
        world_to_snake.setRotation( tf::Quaternion(0,0,0,1) );
        br.sendTransform(tf::StampedTransform(world_to_snake, ros::Time::now(), "world", "snake"));

        // set transform from snake origin to base_link of the snake robot model
        snake_to_base_link.setOrigin( tf::Vector3(-0.5,0,0) );
        snake_to_base_link.setRotation( tf::Quaternion(0,0,0,1) );
        br.sendTransform(tf::StampedTransform(snake_to_base_link, ros::Time::now(), "snake", "base_link"));
        rate.sleep();


    }
    return 0;

}