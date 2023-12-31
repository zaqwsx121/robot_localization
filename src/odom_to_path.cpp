#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>


class OdomToPath
{
public:
    OdomToPath()
    {
        path_pub = nh.advertise<nav_msgs::Path>("/path/filtered", 10, true);
        // odom_sub = nh.subscribe("/odometry/filtered", 10, &OdomToPath::odomCallback, this);
        odom_sub = nh.subscribe("/encoder_odom", 10, &OdomToPath::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped cur_pose;
        cur_pose.header = msg->header;
        cur_pose.pose = msg->pose.pose;

        path.header = msg->header;
        path.poses.push_back(cur_pose);

        path_pub.publish(path);

        tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(cur_pose.pose.position.x, cur_pose.pose.position.y, cur_pose.pose.position.z));
        tf::Quaternion q;
        tf::quaternionMsgToTF(cur_pose.pose.orientation, q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    }

private:
    ros::NodeHandle nh;
    ros::Publisher path_pub;
    ros::Subscriber odom_sub;
    nav_msgs::Path path;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_path");
    OdomToPath odomToPath;
    ros::spin();
    return 0;
}
