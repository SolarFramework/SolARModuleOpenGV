/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#ifndef SolARPoseEstimationP3PGao_H
#define SolARPoseEstimationP3PGao_H
#include <vector>
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "datastructure/Image.h"
#include "SolAROpengvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace OPENGV {

/**
 * @class PoseEstimationP3PGao
 * @brief <B>Finds the camera pose of three 2D-3D points correspondences based on opengv GAO P3P algorithm.</B>
 * <TT>UUID: 6efb890b-8e90-487b-a34a-50e7373444cf</TT>
 *
 */

class SOLAROPENGV_EXPORT_API PoseEstimationP3PGao : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::pose::I3DTransformFinderFrom2D3D
{
public:
    ///@brief SolARPoseEstimationP3PGao constructor;
    PoseEstimationP3PGao() ;
    ///@brief SolARPoseEstimationP3PGao destructor;
    ~PoseEstimationP3PGao();

    /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D world points.
    /// @param[in] imagePoints the set of 2D image points.
    /// @param[in]  worldPoints the set of 3d world points.
    /// @param[in] camParams the camera parameters.
    /// @param[out] pose camera pose (pose of the camera defined in world corrdinate system) expressed as a Transform3D.
    /// @param[in] initialPose (Optional) a transform3D to initialize the pose (reducing the convergence time and improving its success).
    /// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(const std::vector<SolAR::datastructure::Point2Df> & imagePoints,
                                 const std::vector<SolAR::datastructure::Point3Df> & worldPoints,
                                 const SolAR::datastructure::CameraParameters & camParams,
                                 SolAR::datastructure::Transform3Df & pose,
                                 const SolAR::datastructure::Transform3Df initialPose = SolAR::datastructure::Transform3Df::Identity()) override;

    void unloadComponent () override final;
};

}
}
}

#endif // SolARPoseEstimationP3PGao_H
