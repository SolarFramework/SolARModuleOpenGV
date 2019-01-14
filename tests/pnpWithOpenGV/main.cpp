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

void help()
{
    std::cout << "\n\n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
    std::cout << "Something went wrong with input files \n";
    std::cout << "please refer to README.adoc in the project directory \n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n";
    exit(-1);
}

int main()
{
    
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

  SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

#ifdef USE_FREE
    if(xpcfComponentManager->load("conf_Pnp.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Pnp.xml")
        return -1;
    }
#else
    if(xpcfComponentManager->load("conf_Pnp.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Pnp.xml")
        return -1;
    }
#endif

    //Opengv Triangulation
    SRef<solver::map::ITriangulator> triangulator_opengv = xpcfComponentManager->create<SolAR::MODULES::OPENGV::SolARTriangulationOpengv>()->bindTo<solver::map::ITriangulator>();
   
    LOG_ADD_LOG_TO_CONSOLE();

    LOG_INFO("Camera loaded");
    SRef<image::IImageLoader> imageLoader1 =xpcfComponentManager->create<SolARImageLoaderOpencv>("image1")->bindTo<image::IImageLoader>();
    LOG_INFO("Image 1 loaded");
    SRef<image::IImageLoader> imageLoader2 =xpcfComponentManager->create<SolARImageLoaderOpencv>("image2")->bindTo<image::IImageLoader>();
    LOG_INFO("Image 2 loaded");


    // component declaration and creation
    SRef<input::devices::ICamera> camera =xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    LOG_INFO("Intrinsic parameters for the camera for the frame 2: \n {}",camera->getIntrinsicsParameters().matrix());

    CamCalibration tmp_cam_calib = camera->getIntrinsicsParameters();

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
 
    // declarations of data structures used to exange information between components
    SRef<Image>                                         image1;
    SRef<Image>                                         image2;

    std::vector< SRef<Keypoint>>                        keypoints1;
    std::vector< SRef<Keypoint>>                        keypoints2;

    SRef<DescriptorBuffer>                              descriptors1;
    SRef<DescriptorBuffer>                              descriptors2;
    std::vector<DescriptorMatch>                        matches;

    SRef<Image>                                         matchesImage;

    Transform3Df                                        poseFrame1 = Transform3Df::Identity();
    Transform3Df                                        poseFrame2;

    // initialize components requiring the camera intrinsic parameters (please refeer to the use of intrinsic parameters file)
    // poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    // triangulator->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    // triangulator_opengv->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // Get first image
    if (imageLoader1->getImage(image1) != FrameworkReturnCode::_SUCCESS){
        
        LOG_ERROR("Cannot load image 1 with path {}", imageLoader1->bindTo<xpcf::IConfigurable>()->getProperty("pathFile")->getStringValue());
        return -1;
    }
    
}
