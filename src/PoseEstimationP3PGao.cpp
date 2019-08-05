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


#include "PoseEstimationP3PGao.h"
#include "SolARModuleOpengv_traits.h"
#include "core/Log.h"

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGV::PoseEstimationP3PGao);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {

PoseEstimationP3PGao::PoseEstimationP3PGao():ConfigurableBase(xpcf::toUUID<PoseEstimationP3PGao>())
{
    addInterface<api::solver::pose::I3DTransformFinderFrom2D3D>(this);

    LOG_DEBUG(" PoseEstimationP3PGao constructor");
}

PoseEstimationP3PGao::~PoseEstimationP3PGao() = default;

FrameworkReturnCode PoseEstimationP3PGao::estimate( const std::vector<Point2Df> & imagePoints,
                                                    const std::vector<Point3Df> & worldPoints,
                                                    Transform3Df & pose,
                                                    const Transform3Df initialPose) {
    
    if (imagePoints.size() < 3 || worldPoints.size() < 3 || worldPoints.size() != imagePoints.size()){
        return FrameworkReturnCode::_ERROR_;
    }   

    Eigen::Matrix<float,3,3> k_invert =  m_intrinsicParams.inverse();
 
    std::vector<Eigen::Vector3f> buffer_vector; 
    buffer_vector.resize( imagePoints.size());

    opengv::bearingVectors_t bearing_buffer;
    opengv::points_t points;

    //TO DO APPLY UNDISTORSION
    points.reserve(imagePoints.size());
    bearing_buffer.reserve(imagePoints.size());
    for(unsigned int k =0; k <imagePoints.size(); k++){

        points.emplace_back(worldPoints[k].x(), worldPoints[k].y(), worldPoints[k].z());
        
        Eigen::Vector3f tmp = k_invert*Eigen::Vector3f(imagePoints[k].x(), imagePoints[k].y(), 1.0f);
        bearing_buffer.emplace_back(tmp[0], tmp[1], tmp[2]);
        bearing_buffer[k] /=tmp.norm();
    }  

    opengv::rotation_t rotationgv;
    opengv::absolute_pose::CentralAbsoluteAdapter adapter( bearing_buffer, points, rotationgv );
 
    opengv::transformations_t gao_transformation;
    
    size_t iterations = 50;

    for (size_t i = 0; i < iterations; i++)
    {
        gao_transformation = opengv::absolute_pose::p3p_gao(adapter);
    }

    //for now, I just get the first result provided
    if (!gao_transformation.empty())
    {

        pose(0, 0) = gao_transformation[0](0, 0);
        pose(0, 1) = gao_transformation[0](0, 1);
        pose(0, 2) = gao_transformation[0](0, 2);
        pose(0, 3) = gao_transformation[0](0, 3);
        pose(1, 0) = gao_transformation[0](1, 0);
        pose(1, 1) = gao_transformation[0](1, 1);
        pose(1, 2) = gao_transformation[0](1, 2);
        pose(1, 3) = gao_transformation[0](1, 3);
        pose(2, 0) = gao_transformation[0](2, 0);
        pose(2, 1) = gao_transformation[0](2, 1);
        pose(2, 2) = gao_transformation[0](2, 2);
        pose(2, 3) = gao_transformation[0](2, 3);
        pose(3, 0) = 0;
        pose(3, 1) = 0;
        pose(3, 2) = 0;
        pose(3, 3) = 1;
    }

    return FrameworkReturnCode::_SUCCESS;
}


void PoseEstimationP3PGao::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    m_intrinsicParams = intrinsicParams;
    m_distorsionParams =distorsionParams;
}

}
}
}
