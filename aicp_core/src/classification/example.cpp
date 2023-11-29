// classification
#include "aicp_core/aicp_classification/classification.hpp"
#include "aicp_core/aicp_classification/common.hpp"

// project
#include "aicp_core/aicp_utils/fileIO.h"

#include <Eigen/Dense>
#include <memory> // unique_ptr
#include <vector>
#include <fstream>

// yaml
#include <yaml-cpp/yaml.h> // read the yaml config

// opencv
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace Eigen;
using namespace aicp;

std::unique_ptr<AbstractClassification> classifier;

int main(int argc, char **argv)
{
  ClassificationParams params;

  /*===================================
  =            YAML Config            =
  ===================================*/
  std::string yamlConfig_;
  YAML::Node yn_;

  // parse command line
  if (argc < 1) {
      std::cerr << "[Main] Usage: " << argv[0] << " PATH TO AICP CONFIG (e.g. aicp_config.yaml)" << std::endl;
      return 1;
  }
  yamlConfig_.append(argv[0]);

  yn_ = YAML::LoadFile(yamlConfig_);

  YAML::Node classificationNode = yn_["AICP"]["Classifier"];

  for (YAML::const_iterator it = classificationNode.begin(); it != classificationNode.end(); ++it) {
    const std::string key = it->first.as<std::string>();

    if (key.compare("type") == 0) {
      params.type = it->second.as<std::string>();
    }
  }

  if (params.type.compare("SVM") == 0) {

    YAML::Node svmNode = classificationNode["SVM"];

    for(YAML::const_iterator it=svmNode.begin();it != svmNode.end();++it) {
      const std::string key = it->first.as<std::string>();

      if(key.compare("threshold") == 0) {
        params.svm.threshold = it->second.as<double>();
      }
      else if(key.compare("saveFile") == 0) {
        params.svm.saveFile = it->second.as<std::string>();
      }
      else if(key.compare("modelLocation") == 0) {
        params.svm.modelLocation = it->second.as<std::string>();
      }
    }
  }

  std::cout << "============================" << std::endl
            << "Parsed YAML Config"           << std::endl
            << "============================" << std::endl;

  std::cout << "[Main] Classification Type: "       << params.type                    << std::endl;

  if(params.type.compare("SVM") == 0) {
    std::cout << "[Main] Acceptance Threshold: "    << params.svm.threshold           << std::endl;
    std::cout << "[Main] Saving Model To: "         << params.svm.saveFile            << std::endl;
    std::cout << "[Main] Loading Model From: "      << params.svm.modelLocation       << std::endl;
  }

  /*===================================
  =              Training             =
  ===================================*/

    // Set up training data
    MatrixXd labels(4,1);
    labels << 1.0, -1.0, -1.0, -1.0;

    MatrixXd training_data(4,2);
    training_data << 501, 10, 255, 10, 501, 255, 10, 501;

    // Train the SVM
    classifier = create_classifier(params);
    classifier->train(training_data, labels);

  /*===================================
  =     Testing and Visualization     =
  ===================================*/

    int width = 512, height = 512;
    Mat image = Mat::zeros(height, width, CV_8UC3);

    Vec3b green(0,255,0), blue (255,0,0);
    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            MatrixXd testing_data(1,2);
            testing_data << j, i;
            MatrixXd testing_data_labels(1,1);
            testing_data_labels << 1.0;
            Eigen::MatrixXd probabilities;
            classifier->test(testing_data, testing_data_labels, &probabilities);

            std::cout << "[Example] Probability: " << probabilities(0,0) << std::endl;
            if (probabilities(0,0) > params.svm.threshold)
                image.at<Vec3b>(i,j)  = green;
            else if (probabilities(0,0) <= params.svm.threshold)
                 image.at<Vec3b>(i,j)  = blue;
        }

    // Show the training data
    int thickness = -1;
    int lineType = 8;
    cv::circle( image, Point(501,  10), 5, Scalar(  0,   0,   0), thickness, lineType);
    cv::circle( image, Point(255,  10), 5, Scalar(255, 255, 255), thickness, lineType);
    cv::circle( image, Point(501, 255), 5, Scalar(255, 255, 255), thickness, lineType);
    cv::circle( image, Point( 10, 501), 5, Scalar(255, 255, 255), thickness, lineType);

    cv::imwrite("result.png", image);        // save the image

    cv::imshow("SVM Simple Example", image); // show it to the user
    cv::waitKey(0);
}
