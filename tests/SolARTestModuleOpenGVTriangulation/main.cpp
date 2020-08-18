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
#include "xpcf/xpcf.h"

// ADD COMPONENTS HEADERS HERE

#include "api/solver/map/ITriangulator.h"
#include "api/input/devices/ICamera.h"

#include <iostream>
#include <string>
#include <vector>

#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "xpcf/xpcf.h"
#include "core/Log.h"
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
  LOG_ADD_LOG_TO_CONSOLE();

  try {

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolAROpenGVTriangulation_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolAROpenGVTriangulation_conf.xml")
            return -1;
        }

        SRef<input::devices::ICamera> camera =xpcfComponentManager->resolve<input::devices::ICamera>();
        SRef<image::IImageLoader> imageLoader1 =xpcfComponentManager->resolve<image::IImageLoader>("image1");
        SRef<image::IImageLoader> imageLoader2 =xpcfComponentManager->resolve<image::IImageLoader>("image2");
        SRef<features::IKeypointDetector> keypointsDetector =xpcfComponentManager->resolve<features::IKeypointDetector>();
        SRef<features::IDescriptorsExtractor> descriptorExtractor =xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
        SRef<features::IDescriptorMatcher> matcher =xpcfComponentManager->resolve<features::IDescriptorMatcher>();
        SRef<display::IMatchesOverlay> overlayMatches =xpcfComponentManager->resolve<display::IMatchesOverlay>();
        SRef<display::IImageViewer> viewerMatches =xpcfComponentManager->resolve<display::IImageViewer>();
        SRef<solver::pose::I3DTransformFinderFrom2D2D> poseFinderFrom2D2D =xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D2D>();
        SRef<solver::map::ITriangulator> triangulator_opengv = xpcfComponentManager->resolve<solver::map::ITriangulator>();
        SRef<solver::map::IMapFilter> mapFilter =xpcfComponentManager->resolve<solver::map::IMapFilter>();
        SRef<display::I3DPointsViewer> viewer3DPoints =xpcfComponentManager->resolve<display::I3DPointsViewer>();

        // declarations of data structures used to exange information between components
        SRef<Image>                     image1;
        SRef<Image>                     image2;

        std::vector<Keypoint>           keypoints1;
        std::vector<Keypoint>           keypoints2;

        SRef<DescriptorBuffer>          descriptors1;
        SRef<DescriptorBuffer>          descriptors2;
        std::vector<DescriptorMatch>    matches;

        std::vector<SRef<CloudPoint>>   cloud, filteredCloud;

        SRef<Image>                     matchesImage;

        Transform3Df                    poseFrame1 = Transform3Df::Identity();
        Transform3Df                    poseFrame2;

        // initialize components requiring the camera intrinsic parameters (please refeer to the use of intrinsic parameters file)
        poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistortionParameters());
        triangulator_opengv->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistortionParameters());

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
        LOG_DEBUG("Number of matches used for triangulation {}//{}", matches.size(), nbMatches);
        LOG_DEBUG("Estimated pose of the camera for the frame 2: \n {}", poseFrame2.matrix());

        // Create a image showing the matches used for pose estimation of the second camera
        overlayMatches->draw(image1, image2, matchesImage, keypoints1, keypoints2, matches);

        // Triangulate the inliers keypoints which match
        double reproj_error_opengv = triangulator_opengv->triangulate(keypoints1,keypoints2,matches,std::make_pair(0, 1),poseFrame1,poseFrame2,cloud);

         std::ofstream myfile;
         myfile.open ("debug_point_position.txt");
         myfile << "#p1 and then p1 from open Gv point cloud";

        for ( unsigned int k =0; k <  cloud.size(); k++){

           myfile << cloud[k]->getX()<<" "<<cloud[k]->getY()<<" "<<cloud[k]->getZ();
           myfile<<"\n";
        };

        myfile.close();

        //    LOG_DEBUG("Reprojection error: {}", reproj_error);
        LOG_INFO("\n\n Reprojection error using opengv: {}\n\n", reproj_error_opengv);

        mapFilter->filter(poseFrame1, poseFrame2, cloud, filteredCloud);

        // Display the matches and the 3D point cloudfilteredCloud
        while (true){
            if (viewer3DPoints->display(cloud, poseFrame2) == FrameworkReturnCode::_STOP ||
                viewerMatches->display(matchesImage) == FrameworkReturnCode::_STOP  ){

                LOG_DEBUG("End of Triangulation sample");
               break;
            }
        }
    }
    catch (xpcf::Exception e)
    {
      LOG_ERROR ("The following exception has been catched: {}", e.what());
      return -1;
    }
    return 0;
}
