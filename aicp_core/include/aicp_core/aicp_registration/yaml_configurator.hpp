#pragma once

#include <yaml-cpp/yaml.h> // read the yaml config
#include <string>
#include "common.hpp"
#include "aicp_core/aicp_overlap/common.hpp"
#include "aicp_core/aicp_classification/common.hpp"

namespace aicp {

class YAMLConfigurator {
public:
    YAMLConfigurator(){

    }
    bool parse(const std::string& path);

    const RegistrationParams& getRegistrationParams();
    const OverlapParams& getOverlapParams();
    const ClassificationParams& getClassificationParams();

    void printParams();

private:
    YAML::Node yn_;
    YAML::Node registrationNode_;

    RegistrationParams registration_params;
    OverlapParams overlap_params;
    ClassificationParams classification_params;
    std::string experiments_param;
};
}
