#pragma once

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
// #include <tf_conversions/tf_eigen.h>

//#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace aicp {

struct VelodyneAccumulatorConfig
{
    int batch_size = 10;
    double max_range = 30;
    double min_range = 0.5;
    std::string lidar_topic = "/point_cloud_filter/velodyne/point_cloud_filtered";
    std::string inertial_frame = "/odom";
};

class VelodyneAccumulatorROS : public rclcpp::Node {
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
public:
    VelodyneAccumulatorROS(const VelodyneAccumulatorConfig& config);

    void processLidar(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in);

    // functions to mimick the same behavior of MIT's cloud accumulator class
    void setConfig(const VelodyneAccumulatorConfig& config);

    uint16_t getCounter() const;
    bool getFinished() const;
    int64_t getFinishedTime() const;

    const PointCloud& getCloud();
    void clearCloud();

private:
    tf2_ros::TransformListener listener_;
    tf2_ros::Buffer tf_buffer_;
    VelodyneAccumulatorConfig config_;
//    laser_geometry::LaserProjection projector_;

//    sensor_msgs::PointCloud2 point_cloud_ros_msg_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_;
//    pcl::PCLPointCloud2 point_cloud_pcl_msg_;

    // clouds in global frame
    // implicitly discarding intensities from the clouds
    PointCloud point_cloud_;
    PointCloud accumulated_point_cloud_;
    int64_t utime_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    bool finished_ = false;
    uint16_t counter = 0;
};
}
