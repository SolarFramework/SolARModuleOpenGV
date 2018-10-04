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


#ifndef SOLARTRIANGULATIONOPENGV_H
#define SOLARTRIANGULATIONOPENGV_H

#include "xpcf/component/ComponentBase.h"
#include "SolAROpengvAPI.h"

#include "api/solver/map/ITriangulator.h"

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {
/**
* @class SolARTriangulationOpengv
* @brief Triangulates set of corresponding 2D-2D points correspondances with known respective camera poses based on opencv SVD.
*/
class SOLAROPENGV_EXPORT_API SolARTriangulationOpengv : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::ITriangulator {
public:
    ///@brief SolARTriangulationOpengv constructor.
    SolARTriangulationOpengv();

    ///@brief SolARTriangulationOpengv destructor.
   ~SolARTriangulationOpengv();

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams)  override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1, set of 2D points seen in view_1.
    /// @param[in] pointsView2, set of 2D points seen in view_2.
    /// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views, a pair representing the id of the two views
    /// @param[in] poseView1, camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2, camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
    /// @param[out] pcloud, Set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<SRef<Point2Df>>& pt2d_1,
                       const std::vector<SRef<Point2Df>>& pt2d_2,
                       const std::vector<DescriptorMatch>&matches,
                       const std::pair<unsigned int,unsigned int>&working_views,
                       const Transform3Df& poseView1,
                       const Transform3Df& poseView2,
                       std::vector<SRef<CloudPoint>>& pcloud) override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1, set of keypoints seen in view_1.
    /// @param[in] pointsView2, set of keypoints seen in view_2.
    /// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views, a pair representing the id of the two views
    /// @param[in] poseView1, Camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2, Camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
    /// @param[out] pcloud, Set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<SRef<Keypoint>>& keypointsView1,
                       const std::vector<SRef<Keypoint>>& keypointsView2,
                       const std::vector<DescriptorMatch>&matches,
                       const std::pair<unsigned int,unsigned int>&working_views,
                       const Transform3Df& poseView1,
                       const Transform3Df& poseView2,
                       std::vector<SRef<CloudPoint>>& pcloud) override;

    void unloadComponent () override final;

 private:
    // Camera calibration matrix
   // cv::Mat_<double> m_camMatrix;
    // inverse of the Camera calibration matrix
   // cv::Mat_<double> m_Kinv;
    // Camera distortion parameters
   // cv::Mat_<double> m_camDistorsion;

};

}
}
}




#endif // SOLARTRIANGULATIONOPENGV_H
