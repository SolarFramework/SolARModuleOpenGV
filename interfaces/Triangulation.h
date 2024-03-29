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


#ifndef TRIANGULATIONOPENGV_H
#define TRIANGULATIONOPENGV_H

#include "xpcf/component/ComponentBase.h"
#include "SolAROpengvAPI.h"

#include "api/solver/map/ITriangulator.h"

#include <vector>
#include <opengv/types.hpp>



namespace SolAR {
namespace MODULES {
namespace OPENGV {

/**
 * @class Triangulation
 * @brief <B>Triangulates set of corresponding 2D-2D points correspondances with known respective camera poses based on opengv.</B>
 * <TT>UUID: bb7dac37-499a-4bc4-9b57-3e010a94ed30</TT>
 *
 */

class SOLAROPENGV_EXPORT_API Triangulation : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::ITriangulator {
public:

    ///@brief Triangulation constructor.
    Triangulation();

    ///@brief Triangulation destructor.
   ~Triangulation();

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const datastructure::CamCalibration & intrinsicParams, const datastructure::CamDistortion & distorsionParams)  override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1, set of 2D points seen in view_1.
    /// @param[in] pointsView2, set of 2D points seen in view_2.
    /// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views, a pair representing the id of the two views
    /// @param[in] poseView1, camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2, camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
    /// @param[out] pcloud, set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate( const std::vector<datastructure::Point2Df> & pt2d_1,
                        const std::vector<datastructure::Point2Df> & pt2d_2,
                        const std::vector<datastructure::DescriptorMatch> & matches,
                        const std::pair<unsigned int,unsigned int> & working_views,
                        const datastructure::Transform3Df & poseView1,
                        const datastructure::Transform3Df & poseView2,
                        std::vector<SRef<datastructure::CloudPoint>> & pcloud) override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1, set of keypoints seen in view_1.
    /// @param[in] pointsView2, set of keypoints seen in view_2.
    /// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views, a pair representing the id of the two views
    /// @param[in] poseView1, Camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2, Camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
    /// @param[out] pcloud, Set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<datastructure::Keypoint> & keypointsView1,
                       const std::vector<datastructure::Keypoint> & keypointsView2,
                       const std::vector<datastructure::DescriptorMatch> & matches,
                       const std::pair<unsigned int,unsigned int> & working_views,
                       const datastructure::Transform3Df & poseView1,
                       const datastructure::Transform3Df & poseView2,
                       std::vector<SRef<datastructure::CloudPoint>> & pcloud) override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1, set of keypoints seen in view_1.
    /// @param[in] pointsView2, set of keypoints seen in view_2.
    /// @param[in] descriptor1, set of descriptors in view_1.
    /// @param[in] descriptor2, set of descriptors in view_2.
    /// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views, a pair representing the id of the two views
    /// @param[in] poseView1, Camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2, Camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
    /// @param[out] pcloud, Set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(	const std::vector<datastructure::Keypoint> & keypointsView1,
                                const std::vector<datastructure::Keypoint> & keypointsView2,
                                const SRef<datastructure::DescriptorBuffer> & descriptor1,
                                const SRef<datastructure::DescriptorBuffer> & descriptor2,
                                const std::vector<datastructure::DescriptorMatch> & matches,
                                const std::pair<unsigned int, unsigned int> & working_views,
                                const datastructure::Transform3Df & poseView1,
                                const datastructure::Transform3Df & poseView2,
                                std::vector<SRef<datastructure::CloudPoint>> & pcloud) override;

    /// @brief calculating 3D cloud points by triangulating pairs of matched features or using depth information of keypoints.
    /// @param[in] frame1 the first frame.
    /// @param[in] frame2 the second frame.
    /// @param[in] matches the matches between these two frames.
    /// @param[in] working_views a pair representing the id of the two views
    /// @param[out] pcloud Set of triangulated 3d_points.
    /// @param[in] onlyDepth if it is true, using only depth information of keypoints for computing 3D cloud points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(SRef<SolAR::datastructure::Frame> frame1,
                       SRef<SolAR::datastructure::Frame> frame2,
                       const std::vector<SolAR::datastructure::DescriptorMatch> &matches,
                       const std::pair<uint32_t, uint32_t> & working_views,
                       std::vector<SRef<SolAR::datastructure::CloudPoint>> & pcloud,
                       const bool& onlyDepth = false) override;

    void unloadComponent () override final;

 private:
   // Camera calibration matrix
   datastructure::CamCalibration m_intrinsicParams;
   datastructure::CamDistortion  m_distorsionParams;


};

}
}
}


#endif // SOLARTRIANGULATIONOPENGV_H
