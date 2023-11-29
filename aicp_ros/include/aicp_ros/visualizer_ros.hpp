#pragma once

#include "rclcpp/rclcpp.hpp"

#include "aicp_core/aicp_utils/visualizer.hpp"

#include <geometry_msgs/msg//pose_stamped.hpp>
#include <geometry_msgs/msg//transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

// #include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>



#include <Eigen/StdVector>

namespace aicp {

class ROSVisualizer : public rclcpp::Node, public Visualizer
{
public:

    ROSVisualizer(std::string fixed_frame);
    ~ROSVisualizer(){}

    // Publish cloud
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int param, // channel name
                      std::string name,
                      int64_t utime);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int param, // channel name
                      std::string name,
                      int64_t utime);

    // Publish map
    void publishMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    int64_t utime,
                    int channel); // 0 : /aicp/prior_map
                                  // 1 : /aicp/aligned_map

    // Publish octree
    void publishOctree(octomap::ColorOcTree*& octree,
                       std::string channel_name);

    // Publish corrected poses
    void publishPoses(Eigen::Isometry3d pose,
                      int param, std::string name, int64_t utime);
    void publishPoses(PathPoses poses,
                      int param, std::string name, int64_t utime);

    void publishOdomPoses(Eigen::Isometry3d pose,
                      int param, std::string name, int64_t utime);
    void publishOdomPoses(PathPoses poses,
                      int param, std::string name, int64_t utime);

    void publishPriorPoses(Eigen::Isometry3d pose,
                      int param, std::string name, int64_t utime);
    void publishPriorPoses(PathPoses poses,
                      int param, std::string name, int64_t utime);

    void publishOdomToMapPose(Eigen::Isometry3d pose, int64_t utime);


    // Publish tf from fixed_frame to odom
    void publishFixedFrameToOdomTF(const Eigen::Isometry3d& fixed_frame_to_base_eigen,
                                   rclcpp::Time msg_time);
    void publishFixedFrameToOdomPose(const Eigen::Isometry3d& fixed_frame_to_base_eigen,
                                     rclcpp::Time msg_time);

    // Gets
    const PathPoses& getPath(){
        return path_;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prior_pose_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fixed_to_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_to_map_pub_;
    
    // Duplicates the list in collections renderer. assumed to be 3xN colors
    std::vector<double> colors_;
    // Path (vector of poses)
    PathPoses path_;
    PathPoses odom_path_;
    PathPoses prior_path_;

    std::string fixed_frame_; // map or map_test
    std::string odom_frame_;
    std::string base_frame_;
    std::string fixed_to_odom_prefix_ = "/localization_manager/";

    // TF listener and broadcaster
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::PoseWithCovarianceStamped fixed_to_odom_msg_;
    geometry_msgs::msg::Pose temp_tf_pose_;

    void computeFixedFrameToOdom(const Eigen::Isometry3d &fixed_frame_to_base_eigen,
                                 Eigen::Isometry3d& fixed_frame_to_odom_eigen);
};
}
