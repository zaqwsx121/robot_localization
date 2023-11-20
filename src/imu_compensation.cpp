#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>
using namespace std;
class ImuAggregatorNode
{
public:
  ImuAggregatorNode()
  {
    // Initialize node handle
    nh = ros::NodeHandle("~");

    // Create publishers and subscribers
    imu_publisher = nh.advertise<sensor_msgs::Imu>("/imu_combined", 10);
    orientation_subscriber = nh.subscribe("/imu_orientation", 10, &ImuAggregatorNode::orientationCallback, this);
    angular_velocity_subscriber = nh.subscribe("/imu_angular_velocity", 10, &ImuAggregatorNode::angularVelocityCallback, this);
    linear_acceleration_subscriber = nh.subscribe("/imu_linear_acceleration", 10, &ImuAggregatorNode::linearAccelerationCallback, this);

    // Initialize variables to store data
    orientation = {0.0, 0.0, 0.0, 0.0};
    angular_velocity = {0.0, 0.0, 0.0};
    linear_acceleration = {0.0, 0.0, 0.0};
  }

  void orientationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    // Assuming msg->data is an array with 4 elements
    // cout << "Received orientation data" << endl;
    if (msg->data.size() == 4)
    {
      orientation = msg->data;
    }
    else
    {
      ROS_WARN("Received invalid orientation data size");
    }
  }

  void angularVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    // Assuming msg->data is an array
    // cout << "Received angular velocity data" << endl;
    angular_velocity = msg->data;
  }

  void linearAccelerationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    // Assuming msg->data is an array
    // cout << "Received linear acceleration data" << endl;
    linear_acceleration = msg->data;
  }

  void publishCombinedImu()
  {
    sensor_msgs::Imu imu_msg;

    // Set header (assuming the header information is the same for all components)
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_link";

    // Set orientation
    imu_msg.orientation.x = orientation.size() > 0 ? orientation[0] : 0.0;
    imu_msg.orientation.y = orientation.size() > 1 ? orientation[1] : 0.0;
    imu_msg.orientation.z = orientation.size() > 2 ? orientation[2] : 0.0;
    imu_msg.orientation.w = orientation.size() > 3 ? orientation[3] : 1.0;

    // Set angular velocity
    imu_msg.angular_velocity.x = angular_velocity.size() > 0 ? angular_velocity[0] : 0.0;
    imu_msg.angular_velocity.y = angular_velocity.size() > 1 ? angular_velocity[1] : 0.0;
    imu_msg.angular_velocity.z = angular_velocity.size() > 2 ? angular_velocity[2] : 0.0;

    // Set linear acceleration
    imu_msg.linear_acceleration.x = linear_acceleration.size() > 0 ? linear_acceleration[0] : 0.0;
    imu_msg.linear_acceleration.y = linear_acceleration.size() > 1 ? linear_acceleration[1] : 0.0;
    imu_msg.linear_acceleration.z = linear_acceleration.size() > 2 ? linear_acceleration[2] : 0.0;

    // Covariance matrices (all zeros for simplicity)
    imu_msg.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_msg.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_msg.linear_acceleration_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // cout << "Publishing combined IMU message" << endl;
    // Publish the combined IMU message
    imu_publisher.publish(imu_msg);
  }

private:
  ros::NodeHandle nh;
  ros::Publisher imu_publisher;
  ros::Subscriber orientation_subscriber;
  ros::Subscriber angular_velocity_subscriber;
  ros::Subscriber linear_acceleration_subscriber;

  std::vector<double> orientation;
  std::vector<double> angular_velocity;
  std::vector<double> linear_acceleration;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_aggregator_node");
  ImuAggregatorNode imu_aggregator_node;

  ros::Rate loop_rate(10);  // Adjust the rate as needed

  while (ros::ok())
  {
    imu_aggregator_node.publishCombinedImu();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
