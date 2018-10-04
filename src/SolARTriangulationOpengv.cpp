

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

#include "SolARTriangulationOpengv.h"
#include "SolAROpenGVHelper.h"
#include "opencv2/calib3d/calib3d.hpp"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGV::SolARTriangulationOpengv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {

SolARTriangulationOpengv::SolARTriangulationOpengv():ComponentBase(xpcf::toUUID<SolARTriangulationOpengv>())
{
   addInterface<api::solver::map::ITriangulator>(this);
   LOG_DEBUG(" SolARTriangulationOpengv constructor");
}

SolARTriangulationOpengv::~SolARTriangulationOpengv(){

}

double SolARTriangulationOpengv::triangulate(const std::vector<SRef<Point2Df>>& pointsView1,
                                                const std::vector<SRef<Point2Df>>& pointsView2,
                                                const std::vector<DescriptorMatch> &matches,
                                                const std::pair<unsigned int,unsigned int>&working_views,
                                                const Transform3Df& poseView1,
                                                const Transform3Df& poseView2,
                                                std::vector<SRef<CloudPoint>>& pcloud){
    
    return 1.0;
}

double SolARTriangulationOpengv::triangulate(const std::vector<SRef<Keypoint>>& pointsView1,
                                                const std::vector<SRef<Keypoint>>& pointsView2,
                                                const std::vector<DescriptorMatch> &matches,
                                                const std::pair<unsigned int,unsigned int>&working_views,
                                                const Transform3Df& poseView1,
                                                const Transform3Df& poseView2,
                                                std::vector<SRef<CloudPoint>>& pcloud){
                                                    
                                                    return 1.0;
}

void SolARTriangulationOpengv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {

}


}
}
}
