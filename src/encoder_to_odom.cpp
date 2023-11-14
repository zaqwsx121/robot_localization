#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <iostream>
#include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>

using namespace std;

class EncoderToOdometry
{
public:
    EncoderToOdometry()
    {
        // Initialize parameters
        // wheel_radius_ = 0.035; // red smaller wheels
        wheel_radius_ = 0.04; // blue bigger wheels
        // wheel_radius_ = 1;
        wheel_separation_ = 0.23;
        prev_left_ = 0.0;
        prev_right_ = 0.0;
        x_ = 0.0;
        y_ = 0.0;
        z_ = 0.0;
        theta_ = 0.0;

        // ROS subscribers and publishers
        encoder_sub_ = nh_.subscribe("/repair_robot/encoder", 10, &EncoderToOdometry::encoderCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("encoder_odom", 10);
        // IMU data subscriber
        imu_sub_ = nh_.subscribe("/imu/data", 10, &EncoderToOdometry::imuCallback, this);
    }

    void encoderCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {

        cout << "encoderCallback" << endl;
        cout << "Left encoder: " << msg->data[1] << endl;
        cout << "Right encoder: " << -msg->data[0] << endl;

        if(prev_left_ == 0.0 && prev_right_ == 0.0)
        {
            prev_left_ = msg->data[1];
            prev_right_ = -msg->data[0];
            return;
        }
        // Extract encoder values
        double left = msg->data[1];
        double right = -msg->data[0];

        // Calculate change in encoder values
        double d_left = left - prev_left_;
        double d_right = right - prev_right_;

        // Calculate distance moved by each wheel
        double delta_left = d_left * wheel_radius_;
        double delta_right = d_right * wheel_radius_;

        // Estimate robot's change in position and orientation
        double delta_s = (delta_right + delta_left) / 2.0;
        double delta_theta = (delta_right - delta_left) / wheel_separation_;

        // Update the pose using the estimated deltas
        x_ += delta_s * cos(theta_);
        y_ += delta_s * sin(theta_);
        z_ += 0.0;
        theta_ += delta_theta;
        cout << "x: " << x_ << endl;
        cout << "y: " << y_ << endl;
        cout << "z: " << z_ << endl;
        cout << "theta: " << theta_ << endl;

        // Publish the estimated pose and velocity as an Odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        // tf::Quaternion q = tf::createQuaternionFromYaw(theta_);
        // tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
        // Add transform from base_link to camera_imu_optical_frame
        tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));


        // Use IMU orientation data
        odom.pose.pose.orientation = imu_orientation_;

        odom.twist.twist.linear.x = delta_s;
        odom.twist.twist.angular.z = delta_theta;

        odom_pub_.publish(odom);

        // Update previous encoder values
        prev_left_ = left;
        prev_right_ = right;
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // Update the stored IMU orientation
        imu_orientation_ = msg->orientation;
        // Convert quaternion to Euler angles
        tf::Quaternion q;
        tf::quaternionMsgToTF(imu_orientation_, q);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Use yaw as theta
        theta_ = yaw;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber imu_sub_; // IMU data subscriber
    ros::Publisher odom_pub_;

    double wheel_radius_;
    double wheel_separation_;
    double prev_left_;
    double prev_right_;
    double x_;
    double y_;
    double z_;
    double theta_;
    geometry_msgs::Quaternion imu_orientation_; // Store IMU orientation
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_to_odometry");
    EncoderToOdometry encoder_to_odometry;
    ros::spin();
    return 0;
}