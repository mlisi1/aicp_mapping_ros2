#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>

#include <aicp_srv/srv/process_file.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "aicp_core/aicp_registration/app.hpp"
#include "velodyne_accumulator.hpp"
#include "visualizer_ros.hpp"
#include "talker_ros.hpp"

namespace aicp {
class AppROS : public rclcpp::Node, public App {
public:
    AppROS(const CommandLineConfig& cl_cfg,
           const VelodyneAccumulatorConfig& va_cfg,
           const RegistrationParams& reg_params,
           const OverlapParams& overlap_params,
           const ClassificationParams& class_params);

    inline ~AppROS() {
        if (input_poses_file_.is_open()) {
            input_poses_file_.close();
        }

        delete accu_;
        delete vis_ros_;
        delete talk_ros_;
    }

    void writeCloudToFile(AlignedCloudPtr cloud);

    // Subscriber callabacks
    void velodyneCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg_in);
    void robotPoseCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_in);
    void interactionMarkerCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr init_pose_msg_in);

    // Advertise services
    bool loadMapFromFileCallBack(const std::shared_ptr<aicp_srv::srv::ProcessFile::Request> request, std::shared_ptr<aicp_srv::srv::ProcessFile::Response> response);
    bool loadMapFromFile(const std::string& file_path);
    bool goBackRequestCallBack(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    bool goBackRequest();

    void run();

private:

    std::ofstream input_poses_file_;
    int input_clouds_counter_;

    Eigen::Isometry3d world_to_body_;
    Eigen::Isometry3d world_to_body_previous_;
    geometry_msgs::msg::PoseStamped::SharedPtr temp_tf_pose_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr corrected_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr overlap_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr alignability_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr risk_pub_;

    VelodyneAccumulatorROS* accu_;
    VelodyneAccumulatorConfig accu_config_;

    // ROS only visualizer
    ROSVisualizer* vis_ros_;
    // ROS talker
    ROSTalker* talk_ros_;

    // Tool functions
    void getPoseAsIsometry3d(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg,
                             Eigen::Isometry3d& eigen_pose);
    void getPoseAsIsometry3d(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg,
                             Eigen::Isometry3d& eigen_pose);

};
} // namespace aicp
