#include "aicp_ros/app_ros.hpp"

#include "aicp_core/aicp_registration/registration.hpp"
#include "aicp_core/aicp_overlap/overlap.hpp"
#include "aicp_core/aicp_classification/classification.hpp"
#include "aicp_core/aicp_utils/common.hpp"

// #include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/io/ply_io.h>

namespace aicp {

AppROS::AppROS(const CommandLineConfig &cl_cfg,
               const VelodyneAccumulatorConfig &va_cfg,
               const RegistrationParams &reg_params,
               const OverlapParams &overlap_params,
               const ClassificationParams &class_params) :
    App(cl_cfg, reg_params, overlap_params, class_params),
    Node("AppROS"), accu_config_(va_cfg)
{
    paramInit();

    // Data structure
    aligned_clouds_graph_ = new AlignedCloudsGraph();
    // Accumulator
    accu_ = new VelodyneAccumulatorROS(accu_config_);
    // Visualizer
    vis_ = new ROSVisualizer(cl_cfg_.fixed_frame);
    // vis_ros_ = new ROSVisualizer(cl_cfg_.fixed_frame);
    // Talker
    talk_ros_ = new ROSTalker(cl_cfg_.fixed_frame);

    // Init pose to identity
    world_to_body_ = Eigen::Isometry3d::Identity();
    world_to_body_previous_ = Eigen::Isometry3d::Identity();

    // Init prior map
    loadMapFromFile(cl_cfg_.map_from_file_path);

    // Pose publisher
    corrected_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(cl_cfg_.output_channel,10);

    // Verbose publishers
    if (cl_cfg_.verbose)
    {
        overlap_pub_ = this->create_publisher<std_msgs::msg::Float32>("/aicp/overlap",10);
        alignability_pub_ = this->create_publisher<std_msgs::msg::Float32>("/aicp/alignability",10);
        risk_pub_ = this->create_publisher<std_msgs::msg::Float32>("/aicp/alignment_risk",10);
    }

    // Write the incoming data to file. This should be false when running live
    if (cl_cfg_.write_input_clouds_to_file)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Writing input clouds to file. Only do this in post processing");
        std::stringstream input_poses_filename;
        input_poses_filename << data_directory_path_.str() << "/aicp_input_poses.csv";

        input_poses_file_.open (input_poses_filename.str().c_str() );
        input_poses_file_ << "# counter, sec, nsec, x, y, z, qx, qy, qz, qw\n";
        input_poses_file_.flush();
        input_clouds_counter_ = 0;
    }

}

void AppROS::robotPoseCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_in)
{
    if ((cl_cfg_.load_map_from_file || cl_cfg_.localize_against_prior_map)
         && !pose_marker_initialized_){
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Pose initial guess in map not set, waiting for interactive marker...");
        return;
    }

    std::unique_lock<std::mutex> lock(robot_state_mutex_);
    {
        // Latest world -> body (pose prior)
        getPoseAsIsometry3d(pose_msg_in, world_to_body_msg_);
        world_to_body_ = world_to_body_msg_;
    }

    if (!pose_initialized_){
        world_to_body_previous_ = world_to_body_;

        // Initialize transform: pose_in_odom -> interactive_marker
        if (cl_cfg_.load_map_from_file || cl_cfg_.localize_against_prior_map)
        {
            initialT_ = (world_to_body_marker_msg_ * world_to_body_.inverse()).matrix().cast<float>();
            total_correction_ = fromMatrix4fToIsometry3d(initialT_);
        } // identity otherwise
        RCLCPP_INFO_STREAM(this->get_logger(), "[Aicp] Starting localization...");
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg_out;

    // Apply correction if available (identity otherwise)
    // TODO: this could be wrong and must be fixed to match cl_cfg_.working_mode == "robot" case
    corrected_pose_ = total_correction_ * world_to_body_; // world -> reference =
                                                          // body -> reference * world -> body
    // Publish initial guess interactive marker
    if (!pose_initialized_)
        vis_->publishPoses(corrected_pose_, 0, "", rclcpp::Clock().now().nanoseconds() / 1000);

    // Publish fixed_frame to odom tf
    rclcpp::Time msg_time(pose_msg_in->header.stamp.sec, pose_msg_in->header.stamp.nanosec);
    // vis_ros_->publishFixedFrameToOdomTF(corrected_pose_, msg_time);
    // vis_ros_->publishFixedFrameToOdomPose(corrected_pose_, msg_time);

    // Publish /aicp/pose_corrected
    // tf::poseEigenToTF(corrected_pose_, temp_tf_pose_);
    // tf::poseTFToMsg(temp_tf_pose_, pose_msg_out.pose.pose);
    pose_msg_out.pose.pose = tf2::toMsg(corrected_pose_);


    pose_msg_out.pose.covariance = pose_msg_in->pose.covariance;
    pose_msg_out.header.stamp = pose_msg_in->header.stamp;
    pose_msg_out.header.frame_id = cl_cfg_.fixed_frame;
    corrected_pose_pub_->publish(pose_msg_out);

    if ( updated_correction_ )
    {
        {
            std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
            clear_clouds_buffer_ = true;
        }
        updated_correction_ = false;
    }

    if (cl_cfg_.verbose)
    {
        // Publish /aicp/overlap
        std_msgs::msg::Float32 overlap_msg;
        overlap_msg.data = octree_overlap_;
        overlap_pub_->publish(overlap_msg);

        if (!risk_prediction_.isZero())
        {
            // Publish /aicp/alignability
            std_msgs::msg::Float32 alignability_msg;
            alignability_msg.data = alignability_;
            alignability_pub_->publish(alignability_msg);

            // Publish /aicp/alignment_risk
            std_msgs::msg::Float32 risk_msg;
            risk_msg.data = risk_prediction_(0,0);
            risk_pub_->publish(risk_msg);
        }
    }

    pose_initialized_ = true;
}


void AppROS::writeCloudToFile(AlignedCloudPtr cloud){
    // Extract the data from AlignedCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = cloud->getCloud();
    Eigen::Isometry3d pose = cloud->getPriorPose();
    Eigen::Quaterniond quat(pose.rotation());
    int64_t utime = cloud->getUtime();
    int64_t sec = floor(utime * 1E-6);
    int64_t nsec = utime - sec* 1E6;

    // Write the base-to-odom estimate
    std::stringstream ss;
    ss << input_clouds_counter_ << ", " << sec << ", " << nsec << ", ";
    ss << pose.translation().x() << ", " << pose.translation().y() << ", " << pose.translation().z() << ", ";
    ss << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w();
    input_poses_file_ << ss.str() << "\n";
    input_poses_file_.flush();

    // Write the point cloud
    std::stringstream ss2;
    ss2 << data_directory_path_.str() << "/cloud_" << input_clouds_counter_ << "_" << sec << "_" << nsec << ".pcd";
    pcd_writer_.write<pcl::PointXYZ> (ss2.str (), *point_cloud, true);

    input_clouds_counter_++;
}


void AppROS::velodyneCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg_in){
    if (!pose_initialized_){
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Pose not initialized, waiting for pose prior...");
        return;
    }

    // Accumulate planar scans to 3D point cloud (global frame)
    if (!clear_clouds_buffer_ )
    {
        accu_->processLidar(laser_msg_in);
//        cout << "[App ROS] " << accu_->getCounter() + 1 << " of " << accu_config_.batch_size << " scans collected." << endl;
    }
    else
    {
        {
            std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
            clear_clouds_buffer_ = false;
//            cout << "[App ROS] Cleaning cloud buffer of " << accu_->getCounter() << " scans." << endl;
        }

        if ( accu_->getCounter() > 0 )
            accu_->clearCloud();
    }

    // Ensure robot moves between accumulated clouds
    Eigen::Isometry3d relative_motion = world_to_body_previous_.inverse() * world_to_body_;
    double dist = relative_motion.translation().norm();
    double rpy[3];
    quat_to_euler(Eigen::Quaterniond(relative_motion.rotation()), rpy[0], rpy[1], rpy[2]);

    if ( accu_->getFinished() )//finished accumulating?
    {
        if ((dist > 1.0) ||
            fabs(rpy[0]) > (10.0 * M_PI / 180.0) || // condition on roll
            fabs(rpy[1]) > (10.0 * M_PI / 180.0) || // condition on pitch
            fabs(rpy[2]) > (10.0 * M_PI / 180.0))   // condition on yaw
        {
            std::cout << "[App ROS] Finished collecting time: " << accu_->getFinishedTime() << std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
            *accumulated_cloud = accu_->getCloud();
            cout << "[App ROS] Processing cloud with " << accumulated_cloud->points.size() << " points." << endl;

//            vis_->publishCloud(accumulated_cloud, 10, "/aicp/accumulated_cloud", accu_->getFinishedTime());

            // Push this cloud onto the work queue (mutex safe)
            {
                std::unique_lock<std::mutex> lock(data_mutex_);

                // Populate AlignedCloud data structure
                AlignedCloudPtr current_cloud (new AlignedCloud(accu_->getFinishedTime(),
                                                                accumulated_cloud,
                                                                world_to_body_));
                world_to_body_previous_ = world_to_body_;

                if (cl_cfg_.write_input_clouds_to_file)
                    writeCloudToFile(current_cloud);

                // Stack current cloud into queue
                cloud_queue_.push_back(current_cloud);
                //cout << "[App ROS] cloud_queue_ size: " << cloud_queue_.size() << endl;

                if (cloud_queue_.size() > cl_cfg_.max_queue_size) {
                    cout << "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n";
                    cout << "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n";
                    cout << "[App ROS] WARNING: dropping " <<
                            (cloud_queue_.size()-cl_cfg_.max_queue_size) << " clouds." << endl;
                }
                while (cloud_queue_.size() > cl_cfg_.max_queue_size) {
                    cloud_queue_.pop_front();
                }
            }
        }
        accu_->clearCloud();

        // Send notification to operator()() which is waiting for this condition variable
        worker_condition_.notify_one();
    }
}

void AppROS::interactionMarkerCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr init_pose_msg_in)
{
    if (!cl_cfg_.load_map_from_file && !cl_cfg_.localize_against_prior_map){
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Map service disabled - interactive marker neglected.");
        return;
    }
    if (!map_initialized_){
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Map not initialized, waiting for map service...");
        return;
    }
    if (!pose_initialized_){ // initial pose can be updated by user until localization starts, not after.
        RCLCPP_INFO_STREAM(this->get_logger(), "[Aicp] Set localization initial pose in map.");

        // world -> body initial guess from interactive marker
        getPoseAsIsometry3d(init_pose_msg_in, world_to_body_marker_msg_);

        pose_marker_initialized_ = true;
    }
    else
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Interactive marker cannot be updated after localization started!");
}

bool AppROS::loadMapFromFileCallBack(const std::shared_ptr<aicp_srv::srv::ProcessFile::Request> request, 
                                            std::shared_ptr<aicp_srv::srv::ProcessFile::Response> response)
{
    return response->success = loadMapFromFile(request->file_path);
}

bool AppROS::loadMapFromFile(const std::string& file_path)
{
    if (!cl_cfg_.load_map_from_file && !cl_cfg_.localize_against_prior_map){
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Map service disabled!");
        return false;
    }
    if (pose_initialized_){
        pcl::PointCloud<pcl::PointXYZ>::Ptr map = prior_map_->getCloud();
        vis_->publishMap(map, prior_map_->getUtime(), 0);
        RCLCPP_WARN_STREAM(this->get_logger(), "[Aicp] Map cannot be updated after localization started!");
        return false;
    }

    // Load map from file
    RCLCPP_INFO_STREAM(this->get_logger(), "[Aicp] Loading map from '" << file_path << "' ...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path.c_str(), *map) == -1)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[Aicp] Error loading map from file!");
      return false;
    }

    // Pre-filter map
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map (new pcl::PointCloud<pcl::PointXYZ>);
    regionGrowingUniformPlaneSegmentationFilter(map, filtered_map);
    // Populate map object
    if (map_initialized_)
        delete prior_map_;
    prior_map_ = new AlignedCloud(rclcpp::Clock().now().nanoseconds() / 1000,
                                  filtered_map,
                                  Eigen::Isometry3d::Identity());
    RCLCPP_INFO_STREAM(this->get_logger(), "[Aicp] Loaded map with " << prior_map_->getCloud()->points.size() << " points.");

    map_initialized_ = true;
    vis_->publishMap(map, prior_map_->getUtime(), 0);

    return true;
}

bool AppROS::goBackRequestCallBack(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    return response->success = goBackRequest();
}

bool AppROS::goBackRequest()
{
    if (!cl_cfg_.localize_against_prior_map){
        // Set map to localize against
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_map_ptr = aligned_map_.makeShared();
        prior_map_ = new AlignedCloud(rclcpp::Clock().now().nanoseconds() / 1000,
                                      aligned_map_ptr,
                                      Eigen::Isometry3d::Identity());
    }

    // Change to localization only settings
    cl_cfg_.localize_against_prior_map = true;
    pose_marker_initialized_ = true;

    // Set path back
    PathPoses path_forward = vis_->getPath();

    RCLCPP_INFO_STREAM(this->get_logger(), "------------------------------- GO BACK -------------------------------");
    RCLCPP_INFO_STREAM(this->get_logger(), "[Aicp] Follow path of "
                    << path_forward.size()
                    << " poses back to origin.");
    RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------------------------------------------------");

    talk_ros_->publishFootstepPlan(path_forward, rclcpp::Clock().now().nanoseconds() / 1000, true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map_ptr = prior_map_->getCloud();
    vis_->publishMap(prior_map_ptr, prior_map_->getUtime(), 0);

    return true;
}

void AppROS::getPoseAsIsometry3d(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg,
                                 Eigen::Isometry3d& eigen_pose)
{
    // tf::poseMsgToTF(pose_msg->pose.pose, temp_tf_pose_);
    // tf::transformTFToEigen(temp_tf_pose_, eigen_pose);
    tf2::fromMsg(pose_msg->pose.pose, eigen_pose);
}

void AppROS::getPoseAsIsometry3d(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg,
                                 Eigen::Isometry3d& eigen_pose)
{
    // tf::poseMsgToTF(pose_msg->pose, temp_tf_pose_);
    // tf::transformTFToEigen(temp_tf_pose_, eigen_pose);
    tf2::fromMsg(pose_msg->pose, eigen_pose);
}

void AppROS::run()
{
    // worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene

    worker_thread_ = std::thread(
        [this]() {

            (*this)();
        }

    );
}

} // namespace aicp
