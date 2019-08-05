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

#include "PoseEstimationSACEPnp.h"
#include "SolARModuleOpengv_traits.h"


#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac/Lmeds.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGV::PoseEstimationSACEPnp);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {


PoseEstimationSACEPnp::PoseEstimationSACEPnp():ConfigurableBase(xpcf::toUUID<PoseEstimationSACEPnp>())
{
    addInterface<api::solver::pose::I3DTransformSACFinderFrom2D3D>(this);
}

PoseEstimationSACEPnp::~PoseEstimationSACEPnp() = default;

FrameworkReturnCode PoseEstimationSACEPnp::estimate( const std::vector<Point2Df> & imagePoints,
                                                            const std::vector<Point3Df> & worldPoints,
                                                            std::vector<Point2Df> & imagePoints_inlier,
                                                            std::vector<Point3Df> & worldPoints_inlier,
                                                            Transform3Df & pose,
                                                            const Transform3Df initialPose) {


    if ( imagePoints.size() < 3 || worldPoints.size() < 3  || worldPoints.size() != imagePoints.size() ){
        return FrameworkReturnCode::_ERROR_;
    }

    Eigen::Matrix<float,3,3> k_invert =  m_intrinsicParams.inverse();
 
    std::cout<<k_invert(0,0)<<" "<<k_invert(0,1)<<" "<<k_invert(0,2)<<std::endl;
    std::cout<<k_invert(1,0)<<" "<<k_invert(1,1)<<" "<<k_invert(1,2)<<std::endl;
    std::cout<<k_invert(2,0)<<" "<<k_invert(2,1)<<" "<<k_invert(2,2)<<std::endl;


    std::vector<Eigen::Vector3f> buffer_vector; 
    buffer_vector.resize( imagePoints.size());

    opengv::bearingVectors_t bearingVectors;
    opengv::points_t points;
    opengv::rotation_t rotation;

    //TO DO APPLY UNDISTORSION
    points.reserve(imagePoints.size());
    bearingVectors.reserve(imagePoints.size());
    for(unsigned int k =0; k < imagePoints.size(); k++){

        points.emplace_back( worldPoints[k].x(), worldPoints[k].y(), worldPoints[k].z());
        
        Eigen::Vector3f tmp = k_invert*Eigen::Vector3f(imagePoints[k].x(), imagePoints[k].y(), 1.0f);
        bearingVectors.emplace_back(tmp[0], tmp[1], tmp[2]);
        bearingVectors[k] /=tmp.norm();
    }  

    opengv::absolute_pose::CentralAbsoluteAdapter adapter(
        bearingVectors,
        points,
        rotation);

    opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
      new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
      adapter,
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::EPNP)
      );

     ransac.sac_model_ = absposeproblem_ptr;
     
    ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
    ransac.max_iterations_ = 50;

    ransac.computeModel();
    
    ///get the indices of the points defined as inliers
    //LOG_INFO( "the number of inliers is: " << ransac.inliers_.size());

    imagePoints_inlier.resize( ransac.inliers_.size());
    worldPoints_inlier.resize( ransac.inliers_.size());

    for (unsigned int kc = 0; kc < ransac.inliers_.size(); kc++)
    {
        imagePoints_inlier[kc] = SolAR::Point2Df(imagePoints[kc].x(), imagePoints[kc].y());
        worldPoints_inlier[kc] = SolAR::Point3Df(worldPoints[kc].x(), worldPoints[kc].y(), worldPoints[kc].z());
    }

    pose(0, 0) = ransac.model_coefficients_(0, 0);
    pose(0, 1) = ransac.model_coefficients_(0, 1);
    pose(0, 2) = ransac.model_coefficients_(0, 2);
    pose(0, 3) = ransac.model_coefficients_(0, 3);
    pose(1, 0) = ransac.model_coefficients_(1, 0);
    pose(1, 1) = ransac.model_coefficients_(1, 1);
    pose(1, 2) = ransac.model_coefficients_(1, 2);
    pose(1, 3) = ransac.model_coefficients_(1, 3);
    pose(2, 0) = ransac.model_coefficients_(2, 0);
    pose(2, 1) = ransac.model_coefficients_(2, 1);
    pose(2, 2) = ransac.model_coefficients_(2, 2);
    pose(2, 3) = ransac.model_coefficients_(2, 3);
    pose(3, 0) = 0;
    pose(3, 1) = 0;
    pose(3, 2) = 0;
    pose(3, 3) = 1;

    return FrameworkReturnCode::_SUCCESS;
}


void PoseEstimationSACEPnp::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    m_intrinsicParams = intrinsicParams;
    m_distorsionParams =distorsionParams;
}

}
}
}
