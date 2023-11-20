#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <fstream>  // Include for file operations

using namespace std;

const int IMU_DATA_NUM = 100;
std::vector<double> imu_data;
int imu_data_index = 0;


class EncoderToOdometry
{
public:
    EncoderToOdometry(): file("/home/yusyuand/catkin_ws_rs/src/robot_localization/src/localization.txt"), offset_calculated(false), yaw_offset(0.0), sum_yaw(0.0), count_yaw(0)
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
        count_yaw = 0;
        count = 0;

        imu_data.resize(IMU_DATA_NUM, DBL_MIN);

        // ROS subscribers and publishers
        encoder_sub_ = nh_.subscribe("/repair_robot/encoder", 10, &EncoderToOdometry::encoderCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("encoder_odom", 10);
        // IMU data subscriber
        // imu_sub_ = nh_.subscribe("/imu/data", 10, &EncoderToOdometry::imuCallback, this);
        rpy_sub_ = nh_.subscribe("/imu/rpy/filtered", 10, &EncoderToOdometry::rpyCallback, this);

        start_time = ros::Time::now();
        cout << "start_time being setted: " << start_time << endl;
    }

    ~EncoderToOdometry()
    {
        file.close();
    }

    void encoderCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {

        // cout << "encoderCallback" << endl;
        // cout << "Left encoder: " << msg->data[1] << endl;
        // cout << "Right encoder: " << -msg->data[0] << endl;

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
        // double delta_theta = (delta_right - delta_left) / wheel_separation_;

        // Update the pose using the estimated deltas
        x_ += delta_s * cos(theta_);
        y_ += delta_s * sin(theta_);
        z_ += 0.0;
        // theta_ += delta_theta;
        //cout << "x: " << x_ << endl;
        //cout << "y: " << y_ << endl;
        //cout << "z: " << z_ << endl;
        //cout << "theta: " << theta_ << endl;

        // Publish the estimated pose and velocity as an Odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        tf::Quaternion q = tf::createQuaternionFromYaw(theta_);
        tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
        // Add transform from base_link to camera_imu_optical_frame
        // tf::TransformBroadcaster br;
        // tf::Transform transform;
        // transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        // tf::Quaternion q;
        // q.setRPY(0, 0, 0);
        // transform.setRotation(q);
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));


        // Use IMU orientation data
        // odom.pose.pose.orientation = imu_orientation_;

        odom.twist.twist.linear.x = delta_s;
        // odom.twist.twist.angular.z = delta_theta;
        odom.twist.twist.angular.z = theta_;

        odom_pub_.publish(odom);

        // Update previous encoder values
        prev_left_ = left;
        prev_right_ = right;

        // Write x, y, z, and theta to file
        file << x_ << ", " << y_ << ", " << z_ << ", " << theta_ << std::endl;
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // Update the stored IMU orientation
        imu_orientation_ = msg->orientation;
        // Convert quaternion to Euler angles
        tf::Quaternion q;
        tf::quaternionMsgToTF(imu_orientation_, q);
        // tf::quaternionMsgToTF(q, imu_orientation_);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << endl;

        // cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << endl;
        // double elapsed_time = (ros::Time::now() - start_time).toSec();
        // cout << "start_time: " << start_time << ", ros::Time::now(): " << ros::Time::now() << endl; // "elapsed_time:
        // cout << "elapsed_time: " << elapsed_time << endl;

        // if (!offset_calculated) {
        //     if (elapsed_time < 1.0) {
        //         sum_yaw += yaw;
        //         count_yaw++;
        //         cout << "sum_yaw: " << sum_yaw << ", count_yaw: " << count_yaw << endl;
        //     } else {
        //         yaw_offset = sum_yaw / count_yaw;
        //         offset_calculated = true;
        //         cout << "yaw_offset: " << yaw_offset << endl;
        //     }
        // } else {
        //     // Adjust yaw with the calculated offset and normalize to [-pi, pi]
            
        //     theta_ = yaw - yaw_offset;
        //     cout << "yaw: " << yaw << ", yaw_offset: " << yaw_offset << " theta: " << theta_ << endl;
        // }

        // Use yaw as theta
        theta_ = yaw;
        // cout << "theta: " << theta_ << endl;
    }
    void rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        //double roll = msg->vector.x;
        //double pitch = msg->vector.y;
        double yaw = msg->vector.z;

        // double average_yaw_change_rate = 2.197195121951219e-05;
        count++;

        // Assuming you want to use yaw for your theta
        // and apply an initial offset calculation as in imuCallback

        imu_data[imu_data_index] = yaw;
        imu_data_index = (imu_data_index + 1) % IMU_DATA_NUM;

        if(count < IMU_DATA_NUM)
        {
            return;
        }

        double sum = 0.0;
        for(int i = 0; i < IMU_DATA_NUM; i++)
        {
            sum += imu_data[i];
        }
        double mean = sum / IMU_DATA_NUM;
        double variance = 0.0;
        for(int i = 0; i < IMU_DATA_NUM; i++)
        {
            variance += pow(imu_data[i] - mean, 2);
        }
        if(!offset_calculated && variance < 5.0E-5)
        {     
            // Compute variance of imu_data array
            yaw_offset = mean;
            offset_calculated = true;
            cout << "IMU Yaw offset computed: " << yaw_offset << endl;
        }
        theta_ = yaw - yaw_offset;
        
        // cout << "yaw: " << yaw << " mean: " << mean << " sum: " << sum << " variance: " << variance << " yaw_offset: " << yaw_offset  << " theta: " << theta_ << endl;

        // if (!offset_calculated) {
        //     if ((ros::Time::now() - start_time).toSec() < 20.0) {
        //         sum_yaw += yaw;
        //         count_yaw++;
        //         cout << "yaw: " << yaw << ", yaw_offset: " << sum_yaw / count_yaw << " theta: " << yaw - yaw_offset << endl;
        //     } else {
        //         yaw_offset = sum_yaw / count_yaw;
        //         offset_calculated = true;
        //     }
        // } else {
        //     theta_ = yaw - yaw_offset;
        //     // theta_ = yaw - yaw_offset - average_yaw_change_rate*count;
        //     cout << "yaw: " << yaw << ", yaw_offset: " << yaw_offset << " theta: " << theta_ << endl;
        // }

        // theta_ = yaw - 0.0017;

        // cout << "Adjusted Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << theta_ << endl;
        // save to file
        std::ofstream theta_file;
        theta_file.open("/home/yusyuand/catkin_ws_rs/src/robot_localization/src/odom_data.txt", std::ios::app);
        theta_file << yaw << " " << yaw_offset << " " << theta_ << std::endl;
        theta_file.close();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber imu_sub_; // IMU data subscriber
    ros::Publisher odom_pub_;
    ros::Subscriber rpy_sub_;

    double wheel_radius_;
    double wheel_separation_;
    double prev_left_;
    double prev_right_;
    double x_;
    double y_;
    double z_;
    double theta_;
    geometry_msgs::Quaternion imu_orientation_; // Store IMU orientation
    std::ofstream file;

    bool offset_calculated;
    double yaw_offset;
    ros::Time start_time;
    double sum_yaw;
    int count_yaw;
    int count;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_to_odometry");
    EncoderToOdometry encoder_to_odometry;
    ros::spin();
    return 0;
}