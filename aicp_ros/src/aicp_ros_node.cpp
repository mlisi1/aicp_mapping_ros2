#include "aicp_ros/app_ros.hpp"
#include "aicp_core/aicp_registration/yaml_configurator.hpp"
#include "aicp_core/aicp_utils/common.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::Node node("aicp_ros_node");

    node.declare_parameter("registration_config_file", "");
    node.declare_parameter("aicp_config_file", "");
    node.declare_parameter("working_mode", "robot");

    node.declare_parameter("registration_default_config_file", "");

    node.declare_parameter("fixed_frame", "map");
    node.declare_parameter("load_map_from_file", false);

    node.declare_parameter("map_from_file_path", "");
    node.declare_parameter("localize_against_prior_map", false);
    node.declare_parameter("localize_against_built_map", false);
    node.declare_parameter("crop_map_around_base", 8.0);
    node.declare_parameter("merge_aligned_clouds_to_map", false);

    node.declare_parameter("failure_prediction_mode", false);
    node.declare_parameter("reference_update_frequency", 5);
    node.declare_parameter("max_correction_magnitude", 0.5);

    node.declare_parameter("max_queue_size", 3);

    node.declare_parameter("pose_body_channel", "/state_estimator/pose_in_odom");
    node.declare_parameter("output_channel", "/aicp/pose_corrected");
    node.declare_parameter("verbose", false);
    node.declare_parameter("write_input_clouds_to_file", false);
    node.declare_parameter("process_input_clouds_from_file", false);
    node.declare_parameter("process_input_clouds_folder", "/tmp/aicp_data");


    node.declare_parameter("batch_size", 80);
    node.declare_parameter("min_range", 0.50);
    node.declare_parameter("max_range", 15.0);
    node.declare_parameter("lidar_topic", "/point_cloud_filter/velodyne/point_cloud_filtered");
    node.declare_parameter("inertial_frame", "/odom");



    CommandLineConfig cl_cfg;
    // cl_cfg.registration_config_file.append("");
    // cl_cfg.aicp_config_file.append("");
    // cl_cfg.working_mode = "robot"; // e.g. robot - POSE_BODY has been already corrected
    //                                // or debug - apply previous transforms to POSE_BODY
    // cl_cfg.fixed_frame = "map";
    // cl_cfg.load_map_from_file = false; // if enabled, wait for file_path to be sent through a service,
    //                                    // align first cloud only against map (to visualize final drift)
    // cl_cfg.map_from_file_path = "";
    // cl_cfg.localize_against_prior_map = false; // reference is prior map cropped around current pose
    // cl_cfg.localize_against_built_map = false; // reference is aligned map cropped around current pose
    // cl_cfg.crop_map_around_base = 8.0; // rectangular box dimesions: value*2 x value*2
    // cl_cfg.merge_aligned_clouds_to_map = false; // improves performance if trajectory goes
    //                                             // outside map (issue: slow)

    // cl_cfg.failure_prediction_mode = false; // compute Alignment Risk
    // cl_cfg.reference_update_frequency = 5;
    // cl_cfg.max_correction_magnitude = 0.5; // Max allowed correction magnitude
    //                                        // (probably failed alignment otherwise)
    // cl_cfg.max_queue_size = 3; // maximum length of the queue of accumulated point clouds. was 100 previously

    // cl_cfg.pose_body_channel = "/state_estimator/pose_in_odom";
    // cl_cfg.output_channel = "/aicp/pose_corrected"; // Create new channel...
    // cl_cfg.verbose = false; // enable visualization for debug
    // cl_cfg.write_input_clouds_to_file = false; // write the raw incoming point clouds to a folder, for post processing
    // cl_cfg.process_input_clouds_from_file = false;  // process raw incoming point cloud from a folder
    // cl_cfg.process_input_clouds_folder = "/tmp/aicp_data";

    aicp::VelodyneAccumulatorConfig va_cfg;
    // va_cfg.batch_size = 80; // 240 is about 1 sweep at 5RPM // 80 is about 1 sweep at 15RPM
    // va_cfg.min_range = 0.50; // 1.85; // remove all the short range points
    // va_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
    // va_cfg.lidar_topic ="/point_cloud_filter/velodyne/point_cloud_filtered";
    // va_cfg.inertial_frame = "/odom";

    node.get_parameter("registration_config_file", cl_cfg.registration_config_file);
    std::cout << cl_cfg.registration_config_file << std::endl;
    node.get_parameter("aicp_config_file", cl_cfg.aicp_config_file);
    node.get_parameter("working_mode", cl_cfg.working_mode);
    node.get_parameter("fixed_frame", cl_cfg.fixed_frame);
    node.get_parameter("load_map_from_file", cl_cfg.load_map_from_file);
    node.get_parameter("map_from_file_path", cl_cfg.map_from_file_path);
    node.get_parameter("localize_against_prior_map", cl_cfg.localize_against_prior_map);
    node.get_parameter("localize_against_built_map", cl_cfg.localize_against_built_map);
    node.get_parameter("crop_map_around_base", cl_cfg.crop_map_around_base);
    node.get_parameter("merge_aligned_clouds_to_map", cl_cfg.merge_aligned_clouds_to_map);

    node.get_parameter("failure_prediction_mode", cl_cfg.failure_prediction_mode);
    node.get_parameter("reference_update_frequency", cl_cfg.reference_update_frequency);
    node.get_parameter("max_correction_magnitude", cl_cfg.max_correction_magnitude);
    node.get_parameter("max_queue_size", cl_cfg.max_queue_size);

    node.get_parameter("pose_body_channel", cl_cfg.pose_body_channel);
    node.get_parameter("output_channel", cl_cfg.output_channel);
    node.get_parameter("verbose", cl_cfg.verbose);
    node.get_parameter("write_input_clouds_to_file", cl_cfg.write_input_clouds_to_file);
    node.get_parameter("process_input_clouds_from_file", cl_cfg.process_input_clouds_from_file);
    node.get_parameter("process_input_clouds_folder", cl_cfg.process_input_clouds_folder);


    node.get_parameter("batch_size", va_cfg.batch_size);
    node.get_parameter("min_range", va_cfg.min_range);
    node.get_parameter("max_range", va_cfg.max_range);
    node.get_parameter("lidar_channel", va_cfg.lidar_topic);
    node.get_parameter("inertial_frame", va_cfg.inertial_frame);



    /*===================================
    =            YAML Config            =
    ===================================*/
    aicp::YAMLConfigurator yaml_conf;
    if(!yaml_conf.parse(cl_cfg.aicp_config_file)){
        cerr << "ERROR: could not parse file " << cl_cfg.aicp_config_file << endl;
        return -1;
    }
    yaml_conf.printParams();

    RegistrationParams reg_params = yaml_conf.getRegistrationParams();
    if(!node.get_parameter("registration_default_config_file", reg_params.pointmatcher.configFileName)){
        RCLCPP_ERROR(node.get_logger(), "Param \"registration_default_config_file not found!\"");
    }

    OverlapParams overlap_params = yaml_conf.getOverlapParams();

    ClassificationParams classification_params = yaml_conf.getClassificationParams();
    node.get_parameter("trainingFile", classification_params.svm.trainingFile);
    node.get_parameter("testingFile", classification_params.svm.testingFile);
    node.get_parameter("saveFile", classification_params.svm.saveFile);
    node.get_parameter("saveProbs", classification_params.svm.saveProbs);
    node.get_parameter("modelLocation", classification_params.svm.modelLocation);

    /*===================================
    =              Start App            =
    ===================================*/
    // aicp::AppROS app = new aicp::AppROS(cl_cfg,
    //                                     va_cfg,
    //                                     reg_params,
    //                                     overlap_params,
    //                                     classification_params);

    std::shared_ptr<aicp::AppROS> app(new aicp::AppROS(cl_cfg,
                                                   va_cfg,
                                                   reg_params,
                                                   overlap_params,
                                                   classification_params));


    if (!cl_cfg.process_input_clouds_from_file){

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub = 
            node.create_subscription<sensor_msgs::msg::PointCloud2>(va_cfg.lidar_topic, 
                                                                    100, 
                                                                    std::bind(&aicp::AppROS::velodyneCallBack, app, _1));


        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub = 
            node.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(cl_cfg.pose_body_channel, 
                                                                                    100, 
                                                                                    std::bind(&aicp::AppROS::robotPoseCallBack, app, _1));


        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub = 
            node.create_subscription<geometry_msgs::msg::PoseStamped>("/interaction_marker/pose", 
                                                                        100, 
                                                                        std::bind(&aicp::AppROS::interactionMarkerCallBack, app, _1));

        // Advertise services (using service published by anybotics icp_tools ui)
        rclcpp::Service<aicp_srv::srv::ProcessFile>::SharedPtr load_map_server_ = 
            node.create_service<aicp_srv::srv::ProcessFile>("/icp_tools/load_map_from_file", std::bind(&aicp::AppROS::loadMapFromFileCallBack, app, _1, _2));
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_back_server_ = 
            node.create_service<std_srvs::srv::Trigger>("/aicp/go_back_request", std::bind(&aicp::AppROS::goBackRequestCallBack, app, _1, _2));

        RCLCPP_INFO_STREAM(node.get_logger(), "[Aicp] Waiting for input messages...");

        app->run();
        rclcpp::spin(app);

        // auto worker_thread_ = std::thread([app]() { rclcpp::spin(app); });
        // auto custom_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        // custom_executor->add_node(app);

        // custom_executor->spin_some();
        // worker_thread_.join();
        // RCLCPP_WARN_STREAM(node.get_logger(), "[Aicp] Waiting for input messages...");
        
      

    }else{
        app->processFromFile(cl_cfg.process_input_clouds_folder);
    }

    return 0;
}
