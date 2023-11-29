#include "aicp_ros/velodyne_accumulator.hpp"

#include "aicp_core/aicp_utils/filteringUtils.hpp"

#include <pcl/point_types.h>

using namespace std;
using PointCloud = aicp::VelodyneAccumulatorROS::PointCloud;

namespace aicp {

VelodyneAccumulatorROS::VelodyneAccumulatorROS(const VelodyneAccumulatorConfig &config) :
                                               Node("VelodyneAccumulatorROS"), config_(config), tf_buffer_(get_clock()), listener_(tf_buffer_)
{
//    lidar_sub_ = nh_.subscribe<sensor_msgs::msg::PointCloud2>(config_.lidar_topic,
//                                                         100,
//                                                         &VelodyneAccumulatorROS::processLidar,
//                                                         this);
}

void VelodyneAccumulatorROS::setConfig(const VelodyneAccumulatorConfig &config){
    config_ = config;
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(config_.lidar_topic,
                                                         100,
                                                         std::bind(&VelodyneAccumulatorROS::processLidar, this, std::placeholders::_1));
}


void VelodyneAccumulatorROS::processLidar(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in)
{
    if(finished_){
        return;
    }
    cloud_msg_ = cloud_in;

    rclcpp::Time msg_time(cloud_msg_->header.stamp.sec, cloud_msg_->header.stamp.nanosec);
    geometry_msgs::msg::TransformStamped body_pose_tf;
    try {
        // waitForTransform( to frame, from frame, ... )
        // listener_.waitForTransform(config_.inertial_frame, cloud_msg_->header.frame_id, msg_time, ros::Duration(1.0));
        // listener_.lookupTransform(config_.inertial_frame, cloud_msg_->header.frame_id, msg_time, body_pose_tf);

        body_pose_tf = tf_buffer_.lookupTransform(config_.inertial_frame, cloud_msg_->header.frame_id, msg_time);
    }
    catch (tf2::TransformException ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s : ", ex.what());
        RCLCPP_ERROR(this->get_logger(), "Skipping point cloud.");
        return;
    }
    Eigen::Isometry3d body_pose_eigen;
    body_pose_eigen = tf2::transformToEigen(body_pose_tf);

    // Transform point cloud to global frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_in,*cloud_tmp);

    // Filter: crop cloud using box (max points distance from sensor's origin)
    Eigen::Matrix4f tmp = Eigen::MatrixXf::Identity(4, 4);
    getPointsInOrientedBox(cloud_tmp, -30.0, 30.0, tmp);

    pcl::transformPointCloud(*cloud_tmp, point_cloud_, body_pose_eigen.translation().cast<float>(),
                             Eigen::Quaternionf(body_pose_eigen.rotation().cast<float>()));

    // Accumulate
    accumulated_point_cloud_ += point_cloud_;
    utime_ = cloud_msg_->header.stamp.nanosec / 1000;

    // Check number of accumulated clouds
    if(++counter >= config_.batch_size){
        finished_ = true;
    }
}

void VelodyneAccumulatorROS::clearCloud(){
    point_cloud_.clear();
    accumulated_point_cloud_.clear();
    finished_ = false;
    counter = 0;
}

const PointCloud& VelodyneAccumulatorROS::getCloud(){
    return accumulated_point_cloud_;
}

uint16_t VelodyneAccumulatorROS::getCounter() const{
    return counter;
}

bool VelodyneAccumulatorROS::getFinished() const {
    return finished_;
}

int64_t VelodyneAccumulatorROS::getFinishedTime() const{
    return utime_;
}

} // namespace aicp
