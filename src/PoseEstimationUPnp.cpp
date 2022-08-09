/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "PoseEstimationUPnp.h"
#include "SolARModuleOpengv_traits.h"
#include "core/Log.h"

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGV::PoseEstimationUPnp);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {

PoseEstimationUPnp::PoseEstimationUPnp():ConfigurableBase(xpcf::toUUID<PoseEstimationUPnp>())
{
    addInterface<api::solver::pose::I3DTransformFinderFrom2D3D>(this);

    LOG_DEBUG(" SolARPoseEstimationOpengv constructor");
}

PoseEstimationUPnp::~PoseEstimationUPnp(){

}

FrameworkReturnCode PoseEstimationUPnp::estimate(const std::vector<SolAR::datastructure::Point2Df> & imagePoints,
                                                 const std::vector<SolAR::datastructure::Point3Df> & worldPoints,
                                                 const SolAR::datastructure::CameraParameters & camParams,
                                                 SolAR::datastructure::Transform3Df & pose,
                                                 const SolAR::datastructure::Transform3Df initialPose)
{
    if (imagePoints.size() < 3 || worldPoints.size() < 3 || worldPoints.size() != imagePoints.size()){
        return FrameworkReturnCode::_ERROR_;
    }   


    Eigen::Matrix<float,3,3> k_invert =  camParams.intrinsic.inverse();
 
    std::vector<Eigen::Vector3f> buffer_vector; 
    buffer_vector.resize( imagePoints.size());

    opengv::bearingVectors_t bearing_buffer;
    opengv::points_t points;

    //TO DO APPLY UNDISTORSION
    for(unsigned int k =0; k < imagePoints.size(); k++){

        points.push_back( opengv::point_t( worldPoints[k].getX(), worldPoints[k].getY(), worldPoints[k].getZ()));
        
        Eigen::Vector3f tmp = k_invert*Eigen::Vector3f(imagePoints[k].getX(), imagePoints[k].getY(), 1.0f);
        bearing_buffer.push_back(opengv::point_t( tmp[0], tmp[1],tmp[2]));
        bearing_buffer[k] /=tmp.norm();
    }  

    opengv::rotation_t rotationgv;
    opengv::absolute_pose::CentralAbsoluteAdapter adapter( bearing_buffer, points, rotationgv );
 
    opengv::transformations_t upnp_transformation;
    
    size_t iterations = 50;

    for(size_t i = 0; i < iterations; i++){

        upnp_transformation = opengv::absolute_pose::upnp(adapter);
    }

    if (upnp_transformation.size() > 0)
    {

        pose(0, 0) = upnp_transformation[0](0, 0);
        pose(0, 1) = upnp_transformation[0](0, 1);
        pose(0, 2) = upnp_transformation[0](0, 2);
        pose(0, 3) = upnp_transformation[0](0, 3);
        pose(1, 0) = upnp_transformation[0](1, 0);
        pose(1, 1) = upnp_transformation[0](1, 1);
        pose(1, 2) = upnp_transformation[0](1, 2);
        pose(1, 3) = upnp_transformation[0](1, 3);
        pose(2, 0) = upnp_transformation[0](2, 0);
        pose(2, 1) = upnp_transformation[0](2, 1);
        pose(2, 2) = upnp_transformation[0](2, 2);
        pose(2, 3) = upnp_transformation[0](2, 3);
        pose(3, 0) = 0;
        pose(3, 1) = 0;
        pose(3, 2) = 0;
        pose(3, 3) = 1;
    }

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
