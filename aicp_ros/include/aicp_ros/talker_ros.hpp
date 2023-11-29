#pragma once

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <geometry_msgs/msg/pose_array.hpp>

namespace aicp {

class ROSTalker : public rclcpp::Node
{
public:
    typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> PathPoses;
public:
    ROSTalker(std::string fixed_frame);

    // Publish footstep plan
    void publishFootstepPlan(PathPoses& path,
                             int64_t utime,
                             bool reverse_path = false);
    void reversePath(PathPoses& path);

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr footstep_plan_pub_;
    std::string fixed_frame_; // map or map_test
};
}
