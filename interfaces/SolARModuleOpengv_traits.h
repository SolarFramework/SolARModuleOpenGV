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
namespace OPENGV {
class SolARTriangulationOpengv;
class SolARPoseEstimationPnpOpengv;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::SolARTriangulationOpengv,
                            "bb7dac37-499a-4bc4-9b57-3e010a94ed30",
                             "SolARTriangulationOpengv",
                            "SolAR::MODULES::OPENGV::SolARTriangulationOpengv component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENGV::SolARPoseEstimationPnpOpengv,
                            "22007c73-6847-48aa-a2c1-d2ff59baf92f",
                             "SolARPoseEstimationPnpOpengv",
                             "SolAR::MODULES::OPENGV::SolARPoseEstimationPnpOpengv component")

#endif // SOLARMODULEOPENGV_TRAITS_H
