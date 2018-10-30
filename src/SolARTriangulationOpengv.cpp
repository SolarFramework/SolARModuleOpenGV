

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
#include "SolARModuleOpengv_traits.h"
#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>


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
    
    /// Rotation matrix in opengv is defined as 3x3Matrix3d
    //Eigen::Matrix3d rotation_t;

    //Translation matrix is defined as  Eigen::Vector3d
    
    //set experiment parameters

    //for now, dumb copy
    opengv::translation_t position1 = Eigen::Vector3d::Zero();
    opengv::rotation_t rotation1 = Eigen::Matrix3d::Identity();    
    
    opengv::translation_t position2  = Eigen::Vector3d::Zero(); 
    opengv::rotation_t rotation2 = Eigen::Matrix3d::Identity();  

    position1(0) = poseView1(0,3);
    position1(1) = poseView1(1,3);
    position1(2) = poseView1(2,3);
    
    rotation1(0,0)  = poseView1(0,0); rotation1(0,1)  = poseView1(0,1); rotation1(0,2)  = poseView1(0,2);
    rotation1(1,0)  = poseView1(1,0); rotation1(1,1)  = poseView1(1,1); rotation1(1,2)  = poseView1(1,2);
    rotation1(2,0)  = poseView1(2,0); rotation1(2,1)  = poseView1(2,1); rotation1(2,2)  = poseView1(2,2);

    position2(0) = poseView2(0,3);
    position2(1) = poseView2(1,3);
    position2(2) = poseView2(2,3);
    
    rotation2(0,0)  = poseView2(0,0); rotation2(0,1)  = poseView2(0,1); rotation2(0,2)  = poseView2(0,2);
    rotation2(1,0)  = poseView2(1,0); rotation2(1,1)  = poseView2(1,1); rotation2(1,2)  = poseView2(1,2);
    rotation2(2,0)  = poseView2(2,0); rotation2(2,1)  = poseView2(2,1); rotation2(2,2)  = poseView2(2,2);

    //Extract the relative pose
    opengv::translation_t relativePosition;
    opengv::rotation_t relativeRotation;
  
    relativeRotation = rotation1.transpose() * rotation2;
    relativePosition = rotation1.transpose() * (position2 - position1);


    opengv::bearingVectors_t bearingVectors1;
    opengv::bearingVectors_t bearingVectors2;
    
    size_t numberPoints = matches.size();
     for (size_t i = 0; i<numberPoints; i++) {

        bearingVectors1.push_back(opengv::point_t(pointsView1[matches[i].getIndexInDescriptorA()]->getX(),pointsView1[matches[i].getIndexInDescriptorA()]->getY(),1.0));
        bearingVectors2.push_back(opengv::point_t(pointsView2[matches[i].getIndexInDescriptorB()]->getX(),pointsView2[matches[i].getIndexInDescriptorB()]->getY(),1.0 ));
    
    }

    //create a central relative adapter and pass the relative pose
    opengv::relative_pose::CentralRelativeAdapter adapter(
      bearingVectors1,
      bearingVectors2,
      relativePosition,
      relativeRotation);

   // size_t numberPoints = matches.size()
    Eigen::MatrixXd triangulate_results(3, numberPoints);

    //triangulate points
    for(size_t j = 0; j < numberPoints; j++){
        triangulate_results.block<3,1>(0,j) =  opengv::triangulation::triangulate(adapter,j);
    }


    for(size_t k = 0; k < numberPoints; k++){

        //create outpout CloudPoint with its visibility set
        //FOR NOW ITS REPROJECTION ERROR IS SET TO 0...
        xpcf::utils::shared_ptr<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>();
    
        std::map<unsigned int, unsigned int> visibility;

        visibility[working_views.first]  = matches[k].getIndexInDescriptorA();
        visibility[working_views.second] = matches[k].getIndexInDescriptorB();

        double tmp_REPROJECTION_ERROR = 0.0;

        Eigen::Vector3d tmp_vec = triangulate_results.col(k);
        cp = xpcf::utils::make_shared<CloudPoint>(tmp_vec(0), tmp_vec(1), tmp_vec(2) ,0.0,0.0,0.0, tmp_REPROJECTION_ERROR, visibility);
        pcloud.push_back(cp);
    }

    return 0.0;
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

double SolARTriangulationOpengv::triangulate(	const SRef<Keyframe> &curKeyframe,
						const std::vector<DescriptorMatch>&matches,
						std::vector<SRef<CloudPoint>>& pcloud) {
							return 1.0;
						}
						
void SolARTriangulationOpengv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {

}


}
}
}
