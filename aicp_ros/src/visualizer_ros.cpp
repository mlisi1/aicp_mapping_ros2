#include "aicp_ros/visualizer_ros.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

// #include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

namespace aicp {

ROSVisualizer::ROSVisualizer(string fixed_frame) : Node("ROSVisualizer"), fixed_frame_(fixed_frame), tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_)
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aicp/aligned_cloud", 10);
    prior_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aicp/prior_map", 10);
    aligned_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aicp/aligned_map", 10);
    pose_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aicp/poses",100);
    odom_pose_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aicp/odom_poses",100);
    prior_pose_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aicp/prior_poses",100);

    fixed_to_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(fixed_to_odom_prefix_ + fixed_frame_ + "_to_odom", 10);

    odom_to_map_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/icp_tools/map_pose", 10);

    colors_ = {
         51/255.0, 160/255.0, 44/255.0,  //0
         166/255.0, 206/255.0, 227/255.0,
         178/255.0, 223/255.0, 138/255.0,//6
         31/255.0, 120/255.0, 180/255.0,
         251/255.0, 154/255.0, 153/255.0,// 12
         227/255.0, 26/255.0, 28/255.0,
         253/255.0, 191/255.0, 111/255.0,// 18
         106/255.0, 61/255.0, 154/255.0,
         255/255.0, 127/255.0, 0/255.0, // 24
         202/255.0, 178/255.0, 214/255.0,
         1.0, 0.0, 0.0, // red // 30
         0.0, 1.0, 0.0, // green
         0.0, 0.0, 1.0, // blue// 36
         1.0, 1.0, 0.0,
         1.0, 0.0, 1.0, // 42
         0.0, 1.0, 1.0,
         0.5, 1.0, 0.0,
         1.0, 0.5, 0.0,
         0.5, 0.0, 1.0,
         1.0, 0.0, 0.5,
         0.0, 0.5, 1.0,
         0.0, 1.0, 0.5,
         1.0, 0.5, 0.5,
         0.5, 1.0, 0.5,
         0.5, 0.5, 1.0,
         0.5, 0.5, 1.0,
         0.5, 1.0, 0.5,
         0.5, 0.5, 1.0};

    odom_frame_ = "odom";
    base_frame_ = "base";
}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 int param, // channel name
                                 string name,
                                 int64_t utime)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_rgb);

    int secs = utime * 1E-6;
    int nsecs = (utime - (secs * 1E6)) * 1E3;

    sensor_msgs::msg::PointCloud2 output;
    int nColor = cloud_rgb->size() % (colors_.size()/3);
    double r = colors_[nColor*3]*255.0;
    double g = colors_[nColor*3+1]*255.0;
    double b = colors_[nColor*3+2]*255.0;

    for (size_t i = 0; i < cloud_rgb->points.size (); i++)
    {
        cloud_rgb->points[i].r = r;
        cloud_rgb->points[i].g = g;
        cloud_rgb->points[i].b = b;
    }

    pcl::toROSMsg(*cloud_rgb, output);

    //utime has been checked and is correct;
    //cloud is called and it is correct
    rclcpp::Time timestamp(secs, nsecs);
    output.header.stamp = timestamp;
    output.header.frame_id = fixed_frame_;
    cloud_pub_->publish(output);
}

void ROSVisualizer::publishMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               int64_t utime,
                               int channel)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_rgb);

    int secs = utime * 1E-6;
    int nsecs = (utime - (secs * 1E6)) * 1E3;

    sensor_msgs::msg::PointCloud2 output;
    for (size_t i = 0; i < cloud_rgb->points.size (); i++)
    {
        if (channel == 0)
        {
            cloud_rgb->points[i].r = 255.0;
            cloud_rgb->points[i].g = 255.0;
            cloud_rgb->points[i].b = 255.0;
        }
        else if (channel == 1)
        {
            cloud_rgb->points[i].r = 255.0;
            cloud_rgb->points[i].g = 255.0;
            cloud_rgb->points[i].b = 0.0;
        }
    }

    pcl::toROSMsg(*cloud_rgb, output);

    rclcpp::Time timestamp(secs, nsecs);
    output.header.stamp = timestamp;
    output.header.frame_id = fixed_frame_;

    if (channel == 0)
        prior_map_pub_->publish(output);
    else if (channel == 1)
        aligned_map_pub_->publish(output);
    else
        RCLCPP_WARN_STREAM(this->get_logger(), "[ROSVisualizer] Unknown channel. Map not published.");
}

void ROSVisualizer::publishPoses(Eigen::Isometry3d pose, int param, std::string name, int64_t utime)
{
    path_.push_back(pose);
    publishPoses(path_, param, name, utime);
}

void ROSVisualizer::publishPoses(PathPoses poses, int param, string name, int64_t utime){

    nav_msgs::msg::Path path_msg;
    int secs = utime*1E-6;
    int nsecs = (utime - (secs*1E6))*1E3;
    rclcpp::Time timestamp(secs, nsecs);
    path_msg.header.stamp = timestamp;
    path_msg.header.frame_id = fixed_frame_;

    for (size_t i = 0; i < poses.size(); ++i){
        geometry_msgs::msg::PoseStamped m;
        m.header.stamp = timestamp;
        m.header.frame_id = fixed_frame_;
        // tf::poseEigenToMsg(poses[i], m.pose);
        m.pose = tf2::toMsg(poses[i]);
        path_msg.poses.push_back(m);
    }

    pose_pub_->publish(path_msg);
}


void ROSVisualizer::publishOdomPoses(Eigen::Isometry3d pose, int param, std::string name, int64_t utime)
{
    odom_path_.push_back(pose);
    publishOdomPoses(odom_path_, param, name, utime);
}

void ROSVisualizer::publishOdomPoses(PathPoses poses, int param, string name, int64_t utime){

    nav_msgs::msg::Path path_msg;
    int secs = utime*1E-6;
    int nsecs = (utime - (secs*1E6))*1E3;
    rclcpp::Time timestamp(secs, nsecs);
    path_msg.header.stamp = timestamp;
    path_msg.header.frame_id = fixed_frame_;

    for (size_t i = 0; i < poses.size(); ++i){
        geometry_msgs::msg::PoseStamped m;
        m.header.stamp = timestamp;
        m.header.frame_id = fixed_frame_;
        // tf::poseEigenToMsg(poses[i], m.pose);
        m.pose = tf2::toMsg(poses[i]);
        path_msg.poses.push_back(m);
    }

    odom_pose_pub_->publish(path_msg);
}


void ROSVisualizer::publishPriorPoses(Eigen::Isometry3d pose, int param, std::string name, int64_t utime)
{
    prior_path_.push_back(pose);
    publishPriorPoses(odom_path_, param, name, utime);
}

void ROSVisualizer::publishPriorPoses(PathPoses poses, int param, string name, int64_t utime){

    nav_msgs::msg::Path path_msg;
    int secs = utime*1E-6;
    int nsecs = (utime - (secs*1E6))*1E3;
    rclcpp::Time timestamp(secs, nsecs);
    path_msg.header.stamp = timestamp;
    path_msg.header.frame_id = fixed_frame_;

    for (size_t i = 0; i < poses.size(); ++i){
        geometry_msgs::msg::PoseStamped m;
        m.header.stamp = timestamp;
        m.header.frame_id = fixed_frame_;
        // tf::poseEigenToMsg(poses[i], m.pose);
        m.pose = tf2::toMsg(poses[i]);
        path_msg.poses.push_back(m);
    }

    prior_pose_pub_->publish(path_msg);
}


void ROSVisualizer::publishOdomToMapPose(Eigen::Isometry3d pose, int64_t utime)
{
    int secs = utime*1E-6;
    int nsecs = (utime - (secs*1E6))*1E3;

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    rclcpp::Time timestamp(secs, nsecs);
    msg.header.stamp = timestamp;
    msg.header.frame_id = "odom";
    // tf::poseEigenToMsg(pose, msg.pose.pose);
    msg.pose.pose = tf2::toMsg(pose);
    odom_to_map_pub_->publish(msg);
}


void ROSVisualizer::publishFixedFrameToOdomPose(const Eigen::Isometry3d &fixed_frame_to_base_eigen,
                                                rclcpp::Time msg_time)
{
    Eigen::Isometry3d fixed_frame_to_odom_eigen;
    this->computeFixedFrameToOdom(fixed_frame_to_base_eigen, fixed_frame_to_odom_eigen);

    temp_tf_pose_ = tf2::toMsg(fixed_frame_to_odom_eigen);
    // tf::poseEigenToTF(fixed_frame_to_odom_eigen, temp_tf_pose_);
    // tf::poseTFToMsg(temp_tf_pose_, fixed_to_odom_msg_.pose.pose);
    fixed_to_odom_msg_.header.stamp = msg_time;
    fixed_to_odom_msg_.header.frame_id = odom_frame_;
    fixed_to_odom_pub_->publish(fixed_to_odom_msg_);
}

void ROSVisualizer::computeFixedFrameToOdom(const Eigen::Isometry3d &fixed_frame_to_base_eigen,
                                            Eigen::Isometry3d& fixed_frame_to_odom_eigen)
{
    // TF listener
    geometry_msgs::msg::TransformStamped base_to_odom_tf;
    try {
        // waitForTransform( to frame, from frame, ros::Time(0) = last available, ... )
        // tf_buffer_.waitForTransform(odom_frame_, base_frame_, tf2::TimePointZero, rclcpp::Duration::from_seconds(1.0));
        base_to_odom_tf = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (tf2::TransformException ex)
    {
        RCLCPP_WARN(this->get_logger(), "[ROSVisualizer] %s : ", ex.what());
        return;
    }
    // Convert to Eigen
    Eigen::Isometry3d base_to_odom_eigen;
    base_to_odom_eigen = tf2::transformToEigen(base_to_odom_tf);
    

    // Multiply
    fixed_frame_to_odom_eigen = fixed_frame_to_base_eigen * base_to_odom_eigen.inverse();
}

void ROSVisualizer::publishFixedFrameToOdomTF(const Eigen::Isometry3d& fixed_frame_to_base_eigen,
                                              rclcpp::Time msg_time)
{
    Eigen::Isometry3d fixed_frame_to_odom_eigen;
    this->computeFixedFrameToOdom(fixed_frame_to_base_eigen, fixed_frame_to_odom_eigen);

    // Convert to TF
    geometry_msgs::msg::TransformStamped fixed_frame_to_odom_tf;
    fixed_frame_to_odom_tf = tf2::eigenToTransform(fixed_frame_to_odom_eigen);
    fixed_frame_to_odom_tf.header.stamp = msg_time;
    fixed_frame_to_odom_tf.header.frame_id = odom_frame_;
    

    // Broadcast
    tf_broadcaster_->sendTransform(fixed_frame_to_odom_tf);
    // tf_broadcaster_.sendTransform(tf::StampedTransform(fixed_frame_to_odom_tf,
    //                                                    msg_time,
    //                                                    fixed_frame_,   // from frame  --| (parent frame)
    //                                                    odom_frame_));  // to frame    <-|
}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                                 int param, // buffer size
                                 string name,
                                 int64_t utime = -1)
{
    // to be implemented
}

void ROSVisualizer::publishOctree(octomap::ColorOcTree*& octree,
                                  string channel_name)
{
    // publishOctree to be implemented.
}
}
