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


#include "SolARPoseEstimationPnpOpengv.h"
#include "SolARModuleOpengv_traits.h"

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGV::SolARPoseEstimationPnpOpengv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {

SolARPoseEstimationPnpOpengv::SolARPoseEstimationPnpOpengv():ConfigurableBase(xpcf::toUUID<SolARPoseEstimationPnpOpengv>())
{
    addInterface<api::solver::pose::I3DTransformFinderFrom2D3D>(this);

    LOG_DEBUG(" SolARPoseEstimationOpengv constructor");
}

SolARPoseEstimationPnpOpengv::~SolARPoseEstimationPnpOpengv(){

}

FrameworkReturnCode SolARPoseEstimationPnpOpengv::estimate( const std::vector<SRef<Point2Df>> & imagePoints,
                                                            const std::vector<SRef<Point3Df>> & worldPoints,
                                                            Transform3Df & pose,
                                                            const Transform3Df initialPose) {
    
    
    opengv::translation_t position  = Eigen::Vector3d(pose(0,3), pose(1,3) ,pose(2,3));
    opengv::rotation_t rotation;// = Eigen::Matrix<double, 3,3, Eigen::ColMajor>();

    rotation(0,0)  = pose(0,0); rotation(0,1)  = pose(0,1); rotation(0,2)  = pose(0,2);
    rotation(1,0)  = pose(1,0); rotation(1,1)  = pose(1,1); rotation(1,2)  = pose(1,2);
    rotation(2,0)  = pose(2,0); rotation(2,1)  = pose(2,1); rotation(2,2)  = pose(2,2);

    Eigen::Matrix<float,3,3> k_invert =  m_intrinsicParams.inverse();
 
    //TO DO tomorrow... here use bearingVectors type from opengv
    std::vector<Eigen::Vector3f> buffer_vector; 

    opengv::bearingVectors_t bearing_buffer;
    opengv::points_t points;
    //TO DO APPLY UNDISTORSION
    for(unsigned int k =0; k < imagePoints.size(); k++){

        points.push_back( opengv::point_t( imagePoints[k]->getX(), imagePoints[k]->getY(), 1.0f));
        buffer_vector[k] = k_invert*Eigen::Vector3f(imagePoints[k]->getX(), imagePoints[k]->getY(), 1.0f);
        
        bearing_buffer.push_back(opengv::point_t( buffer_vector[k][0], buffer_vector[k][1], buffer_vector[k][2]));
        bearing_buffer[k] /=bearing_buffer[k].norm();

    }  
    
    opengv::absolute_pose::CentralAbsoluteAdapter adapter( bearing_buffer, points, position, rotation );
 
    opengv::transformation_t epnp_transformation;
    
    size_t iterations = 50;
    for(size_t i = 0; i < iterations; i++){
        epnp_transformation = opengv::absolute_pose::epnp(adapter);
    }

    //pose = epnp_transformation;
    pose(0,0) = epnp_transformation(0,0); pose(0,1) = epnp_transformation(0,1); pose(0,2) = epnp_transformation(0,2); pose(0,3) = epnp_transformation(0,3);
    pose(1,0) = epnp_transformation(1,0); pose(1,1) = epnp_transformation(1,1); pose(1,2) = epnp_transformation(1,2); pose(1,3) = epnp_transformation(1,3);
    pose(2,0) = epnp_transformation(2,0); pose(2,1) = epnp_transformation(2,1); pose(2,2) = epnp_transformation(2,2); pose(2,3) = epnp_transformation(2,3);
    pose(3,0) =0;                         pose(3,1) =0;                         pose(3,2) =0;                         pose(3,3) =1;
   
    
    return FrameworkReturnCode::_SUCCESS;

}

FrameworkReturnCode SolARPoseEstimationPnpOpengv::estimate( const std::vector<SRef<Point2Df>> & imagePoints,
                                                            const std::vector<SRef<Point3Df>> & worldPoints,
                                                            std::vector<SRef<Point2Df>>&imagePoints_inlier,
                                                            std::vector<SRef<Point3Df>>&worldPoints_inlier,
                                                            Transform3Df & pose,
                                                            const Transform3Df initialPose) {

  
    return FrameworkReturnCode::_SUCCESS;
}


void SolARPoseEstimationPnpOpengv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    m_intrinsicParams = intrinsicParams;
    m_distorsionParams =distorsionParams;
}

}
}
}
