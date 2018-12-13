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

#include <iostream>
#include <string>
#include <vector>
//#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpengv_traits.h"
#include "api/solver/map/ITriangulator.h"
#include "api/input/devices/ICamera.h"

#define USE_FREE
#include <iostream>
#include <string>
#include <vector>

#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleOpengl_traits.h"
#include "SolARModuleTools_traits.h"

#include "SolARModuleNonFreeOpencv_traits.h"

#include "xpcf/xpcf.h"

#include "api/image/IImageLoader.h"
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

#include <iostream>
#include <fstream>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::MODULES::NONFREEOPENCV;
using namespace SolAR::MODULES::OPENGL;

using namespace SolAR::MODULES::OPENGV;
namespace xpcf  = org::bcom::xpcf;

void help(){
    std::cout << "\n\n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
    std::cout << "Something went wrong with input files \n";
    std::cout << "please refer to README.adoc in the project directory \n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n";
    exit(-1);
}

int main(){

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

  SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

#ifdef USE_FREE
    if(xpcfComponentManager->load("conf_Triangulation.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Triangulation.xml")
        return -1;
    }
#else
    if(xpcfComponentManager->load("conf_Triangulation.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Triangulation_nf.xml")
        return -1;
    }
#endif

    //Opengv Triangulation
    SRef<solver::map::ITriangulator> triangulator_opengv = xpcfComponentManager->create<SolAR::MODULES::OPENGV::SolARTriangulationOpengv>()->bindTo<solver::map::ITriangulator>();
   
    LOG_ADD_LOG_TO_CONSOLE();

    // component declaration and creation
    SRef<input::devices::ICamera> camera =xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    LOG_INFO("Intrinsic parameters for the camera for the frame 2: \n {}",camera->getIntrinsicsParameters().matrix());

    CamCalibration tmp_cam_calib = camera->getIntrinsicsParameters();

    //---

    LOG_INFO("Camera loaded");
    SRef<image::IImageLoader> imageLoader1 =xpcfComponentManager->create<SolARImageLoaderOpencv>("image1")->bindTo<image::IImageLoader>();
    LOG_INFO("Image 1 loaded");
    SRef<image::IImageLoader> imageLoader2 =xpcfComponentManager->create<SolARImageLoaderOpencv>("image2")->bindTo<image::IImageLoader>();
    LOG_INFO("Image 2 loaded");

#ifdef USE_FREE
    LOG_INFO("free keypoint detector");
    SRef<features::IKeypointDetector> keypointsDetector =xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
#else
    LOG_INFO("nonfree keypoint detector");
    SRef<features::IKeypointDetector>  keypointsDetector = xpcfComponentManager->create<SolARKeypointDetectorNonFreeOpencv>()->bindTo<features::IKeypointDetector>();
#endif

#ifdef USE_FREE
    LOG_INFO("free keypoint extractor");
    SRef<features::IDescriptorsExtractor> descriptorExtractor =xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
#else
    LOG_INFO("nonfree keypoint extractor");
    SRef<features::IDescriptorsExtractor> descriptorExtractor = xpcfComponentManager->create<SolARDescriptorsExtractorSURF64Opencv>()->bindTo<features::IDescriptorsExtractor>();
#endif

    SRef<features::IDescriptorMatcher> matcher =xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
    SRef<display::IMatchesOverlay> overlayMatches =xpcfComponentManager->create<SolARMatchesOverlayOpencv>()->bindTo<display::IMatchesOverlay>();
    SRef<display::IImageViewer> viewerMatches =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    SRef<solver::pose::I3DTransformFinderFrom2D2D> poseFinderFrom2D2D =xpcfComponentManager->create<SolARPoseFinderFrom2D2DOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D2D>();
    SRef<solver::map::ITriangulator> triangulator =xpcfComponentManager->create<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
    SRef<solver::map::IMapFilter> mapFilter =xpcfComponentManager->create<SolARMapFilter>()->bindTo<solver::map::IMapFilter>();

    SRef<display::I3DPointsViewer> viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();
    //SRef<display::I3DPointsViewer> viewer3DPoints_gv =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    // declarations of data structures used to exange information between components
    SRef<Image>                                         image1;
    SRef<Image>                                         image2;

    std::vector< SRef<Keypoint>>                        keypoints1;
    std::vector< SRef<Keypoint>>                        keypoints2;

    SRef<DescriptorBuffer>                              descriptors1;
    SRef<DescriptorBuffer>                              descriptors2;
    std::vector<DescriptorMatch>                        matches;

    std::vector<SRef<CloudPoint>>                       cloud, filteredCloud;
    std::vector<SRef<CloudPoint>>                       cloud_opengv;

    SRef<Image>                                         matchesImage;

    Transform3Df                                        poseFrame1 = Transform3Df::Identity();
    Transform3Df                                        poseFrame2;

    // initialize components requiring the camera intrinsic parameters (please refeer to the use of intrinsic parameters file)
    poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    triangulator->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    triangulator_opengv->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // Get first image
    if (imageLoader1->getImage(image1) != FrameworkReturnCode::_SUCCESS){

        LOG_ERROR("Cannot load image 1 with path {}", imageLoader1->bindTo<xpcf::IConfigurable>()->getProperty("pathFile")->getStringValue());
        return -1;
    }

    // Get second image
    if (imageLoader2->getImage(image2) != FrameworkReturnCode::_SUCCESS)
    {
        LOG_ERROR("Cannot load image 2 with path {}", imageLoader2->bindTo<xpcf::IConfigurable>()->getProperty("pathFile")->getStringValue());
        return -1;
    }

    // Detect the keypoints for the first image
    keypointsDetector->detect(image1, keypoints1);
    // Detect the keypoints for the second image
    keypointsDetector->detect(image2, keypoints2);
    // Compute the descriptor for each keypoint extracted from the first image
    descriptorExtractor->extract(image1, keypoints1, descriptors1);
    // Compute the descriptor for each keypoint extracted from the second image
    descriptorExtractor->extract(image2, keypoints2, descriptors2);
    // Compute the matches between the keypoints of the first image and the keypoints of the second image
    matcher->match(descriptors1, descriptors2, matches);
    int nbMatches = (int)matches.size();

    // Estimate the pose of the second frame (the first frame being the reference of our coordinate system)
    poseFinderFrom2D2D->estimate(keypoints1, keypoints2, poseFrame1, poseFrame2, matches);
    LOG_INFO("Number of matches used for triangulation {}//{}", matches.size(), nbMatches);
    LOG_INFO("Estimated pose of the camera for the frame 2: \n {}", poseFrame2.matrix());

    // Create a image showing the matches used for pose estimation of the second camera
    overlayMatches->draw(image1, image2, matchesImage, keypoints1, keypoints2, matches);

    // Triangulate the inliers keypoints which match
    double reproj_error =triangulator->triangulate(keypoints1,keypoints2,matches,std::make_pair(0, 1),poseFrame1,poseFrame2,cloud);
        
    double reproj_error_opengv = triangulator_opengv->triangulate(keypoints1,keypoints2,matches,std::make_pair(0, 1),poseFrame1,poseFrame2,cloud_opengv);

     std::ofstream myfile;
     myfile.open ("debug_point_position.txt");
     myfile << "#p1 and then p1 from open Gv point cloud";

    for ( unsigned int k =0; k <  cloud.size(); k++){

       myfile << cloud[k]->getX()<<" "<<cloud[k]->getY()<<" "<<cloud[k]->getZ();
       myfile<<"    :     :";
       myfile << cloud_opengv[k]->getX()<<" "<<cloud_opengv[k]->getY()<<" "<<cloud_opengv[k]->getZ();
       myfile<<"\n";
    };

    myfile.close();

    LOG_INFO("Reprojection error: {}", reproj_error);
    LOG_INFO("Reprojection error using opengv: {}", reproj_error_opengv);
    
    mapFilter->filter(poseFrame1, poseFrame2, cloud_opengv, filteredCloud);

    // Display the matches and the 3D point cloudfilteredCloud
    while (true){
        if (viewer3DPoints->display(cloud_opengv, poseFrame2) == FrameworkReturnCode::_STOP ||
            viewerMatches->display(matchesImage) == FrameworkReturnCode::_STOP  ){

            LOG_INFO("End of Triangulation sample");
           break;
        }
    }
    return 0;

    
    std::cout<<"Load ok"<<std::endl;

    return(0);
    
}
