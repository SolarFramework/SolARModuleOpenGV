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

#include "PoseEstimationSACP3PGao.h"
#include "SolARModuleOpengv_traits.h"

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGV::PoseEstimationSACP3PGao);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {


PoseEstimationSACP3PGao::PoseEstimationSACP3PGao():ConfigurableBase(xpcf::toUUID<PoseEstimationSACP3PGao>())
{
    addInterface<api::solver::pose::I3DTransformSACFinderFrom2D3D>(this);
}

PoseEstimationSACP3PGao::~PoseEstimationSACP3PGao(){

}

FrameworkReturnCode PoseEstimationSACP3PGao::estimate( const std::vector<SRef<Point2Df>> & imagePoints,
                                                            const std::vector<SRef<Point3Df>> & worldPoints,
                                                            std::vector<SRef<Point2Df>>&imagePoints_inlier,
                                                            std::vector<SRef<Point3Df>>&worldPoints_inlier,
                                                            Transform3Df & pose,
                                                            const Transform3Df initialPose) {


    return FrameworkReturnCode::_SUCCESS;
}


void PoseEstimationSACP3PGao::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {

}

}
}
}
