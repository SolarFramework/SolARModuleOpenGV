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

#ifndef SolARPoseEstimationEPnpOpengv_H
#define SolARPoseEstimationEPnpOpengv_H
#include <vector>
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "datastructure/Image.h"
#include "SolAROpengvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace OPENGV {

/**
 * @class PoseEstimationEPnp
 * @brief <B>Finds the camera pose of 2D-3D points correspondences based on opengv Efficient Perspective-n-Point algorithm.</B>
 * <TT>UUID: 22007c73-6847-48aa-a2c1-d2ff59baf92f</TT>
 *
 */

class SOLAROPENGV_EXPORT_API PoseEstimationEPnp : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::pose::I3DTransformFinderFrom2D3D
{
public:
    ///@brief PoseEstimationEPnp constructor;
    PoseEstimationEPnp();
    ///@brief PoseEstimationEPnp destructor;
    ~PoseEstimationEPnp();

    /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D  world points.
    /// @param[in] imagePoints, set of 2d_points seen in view_1.
    /// @param[in]  worldPoints, set of 3d_points corresponding to view_1.
    /// @param[out] pose, camera pose (pose the camera defined in world corrdinate system) expressed as a Transform3D.
    /// @param[in] initialPose (Optional), a tranfsform3D to initialize the pose (reducing the convergence time and improving its success). If your world points are planar, do not use this argument.
    FrameworkReturnCode estimate(   const std::vector<datastructure::Point2Df> & imagePoints,
                                    const std::vector<datastructure::Point3Df> & worldPoints,
                                    datastructure::Transform3Df & pose,
                                    const datastructure::Transform3Df initialPose = datastructure::Transform3Df::Identity()) override;

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const datastructure::CamCalibration & intrinsicParams,
                             const datastructure::CamDistortion & distorsionParams)  override;

    void unloadComponent () override final;

private:
    datastructure::CamCalibration m_intrinsicParams;
    datastructure::CamDistortion  m_distorsionParams;
};

}
}
}

#endif // PoseEstimationEPnp_H
