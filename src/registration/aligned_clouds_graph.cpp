#include "aicp_registration/aligned_clouds_graph.hpp"

namespace aicp {

AlignedCloudsGraph::AlignedCloudsGraph()
{ 
    initialized_ = false;
    current_reference_ = -1;
}

AlignedCloudsGraph::~AlignedCloudsGraph()
{ 
}

void AlignedCloudsGraph::initialize(AlignedCloudPtr& reference)
{
    aligned_clouds.push_back(reference);
    current_reference_ = aligned_clouds.size()-1;

    initialized_ = true;
}

bool AlignedCloudsGraph::addCloud(AlignedCloudPtr& cloud, Eigen::Isometry3d relative_pose, Eigen::Isometry3d pose, int reference_id)
{
    if(initialized_)
    {

        aligned_clouds.push_back(cloud);
    }
    return initialized_;
}
}
