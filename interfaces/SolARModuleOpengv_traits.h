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

#ifndef SOLARMODULEOPENGV_TRAITS_H
#define SOLARMODULEOPENGV_TRAITS_H

#include "xpcf/core/traits.h"

namespace SolAR {
namespace MODULES {
/**
 * @namespace SolAR::MODULES::OPENGV
 * @brief <B>Provides triangulation and PnP components based on OpenGV library: https://laurentkneip.github.io/opengv/</B>
 * <TT>UUID: 9f5f0fb9-a8c8-4532-bddf-f988052f63c3</TT>
 *
 */
namespace OPENGV{
class PoseEstimationEPnp;
class PoseEstimationUPnp;

//class PoseEstimationP2P;
class PoseEstimationP3PGao;
class PoseEstimationP3PKneip;

class PoseEstimationSACEPnp;
class PoseEstimationSACP3PGao;
class PoseEstimationSACP3PKneip;

class Triangulation;

}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::PoseEstimationEPnp,
                            "22007c73-6847-48aa-a2c1-d2ff59baf92f",
                             "PoseEstimationEPnp",
                             "Finds the camera pose of 2D-3D points correspondences based on opengv Efficient Perspective-n-Point algorithm.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::PoseEstimationP3PGao,
                            "6efb890b-8e90-487b-a34a-50e7373444cf",
                             "PoseEstimationP3PGao",
                             "Finds the camera pose of three 2D-3D points correspondences based on opengv GAO P3P algorithm.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::PoseEstimationP3PKneip,
                            "473faa6a-e023-49ac-9c48-f00ef9d79af3",
                             "PoseEstimationP3PKneip",
                             "Finds the camera pose of three 2D-3D points correspondences based on opengv Kneip P3P algorithm.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::PoseEstimationSACEPnp,
                            "a2c38e05-40d9-47fc-aad4-1ea2255333d5",
                             "PoseEstimationSACEPnp",
                             "Finds the camera pose of 2D-3D points correspondences based on opengv Efficient Perspective-n-Point algorithm with a RANdom SAmple Consensus.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::PoseEstimationSACP3PGao,
                            "76329985-5faf-46e0-9179-0aedacedb6e2",
                             "PoseEstimationSACP3PGao",
                             "Finds the camera pose of three 2D-3D points correspondences based on opengv Gao P3P algorithm with a RANdom SAmple Consensus.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::PoseEstimationSACP3PKneip,
                            "97045e96-506f-41f8-bb78-b966b4f8d435",
                             "PoseEstimationSACP3PKneip",
                             "Finds the camera pose of three 2D-3D points correspondences based on opengv Kneip P3P algorithm with a RANdom SAmple Consensus.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::PoseEstimationUPnp,
                            "922e9db6-e424-4518-ad26-31201471ff00",
                             "PoseEstimationUPnp",
                             "Finds the camera pose of 2D-3D points correspondences based on opengv Universal Perspective-n-Point algorithm.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::Triangulation,
                            "bb7dac37-499a-4bc4-9b57-3e010a94ed30",
                             "Triangulation",
                            "Triangulates set of corresponding 2D-2D points correspondances with known respective camera poses based on opengv.")

#endif // SOLARMODULEOPENGV_TRAITS_H
