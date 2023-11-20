#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuFrameIdChanger
{
public:
    ImuFrameIdChanger()
    {
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_base_link", 10);
        imu_sub = nh.subscribe("/imu/data", 10, &ImuFrameIdChanger::imuCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        sensor_msgs::Imu new_imu = *msg;
        new_imu.header.frame_id = "base_link";
        imu_pub.publish(new_imu);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;
    ros::Subscriber imu_sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_frame_id_changer");
    ImuFrameIdChanger imuFrameIdChanger;
    ros::spin();
    return 0;
}
