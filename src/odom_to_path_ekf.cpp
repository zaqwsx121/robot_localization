#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>

class OdomToPath
{
public:
    OdomToPath() : file("path_record.txt")
    {
        path_pub = nh.advertise<nav_msgs::Path>("/path/filtered", 10, true);
        odom_sub = nh.subscribe("odometry/filtered", 10, &OdomToPath::odomCallback, this);
    }

    ~OdomToPath()
    {
        file.close();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped cur_pose;
        cur_pose.header = msg->header;
        cur_pose.pose = msg->pose.pose;

        path.header = msg->header;
        path.poses.push_back(cur_pose);

        path_pub.publish(path);

        // Write pose to file
        file << cur_pose.pose.position.x << ", "
             << cur_pose.pose.position.y << ", "
             << cur_pose.pose.position.z << ", "
             << cur_pose.pose.orientation.x << ", "
             << cur_pose.pose.orientation.y << ", "
             << cur_pose.pose.orientation.z << ", "
             << cur_pose.pose.orientation.w << std::endl;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher path_pub;
    ros::Subscriber odom_sub;
    nav_msgs::Path path;
    std::ofstream file;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_path");
    OdomToPath odomToPath;
    ros::spin();
    return 0;
}
