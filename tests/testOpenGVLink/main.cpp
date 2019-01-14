
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>
#include <sstream>
#include <fstream>

#include "random_generators.hpp"
#include "experiment_helpers.hpp"
#include "time_measurement.hpp"

using namespace Eigen;
using namespace opengv;

/*
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
*/
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
    /*
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

    // component declaration and creation
    SRef<input::devices::ICamera> camera =xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    LOG_INFO("Intrinsic parameters for the camera for the frame 2: \n {}",camera->getIntrinsicsParameters().matrix());

    CamCalibration tmp_cam_calib = camera->getIntrinsicsParameters();

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
*/
    // initialize components requiring the camera intrinsic parameters (please refeer to the use of intrinsic parameters file)
    // poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    // triangulator->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    // triangulator_opengv->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    /*
    // Get first image
    if (imageLoader1->getImage(image1) != FrameworkReturnCode::_SUCCESS){

        LOG_ERROR("Cannot load image 1 with path {}", imageLoader1->bindTo<xpcf::IConfigurable>()->getProperty("pathFile")->getStringValue());
        return -1;
    }
    */
    /*
    //initialize random seed
    opengv::initializeRandomSeed();

    //set experiment parameters
    double noise = 0.2;
    double outlierFraction = 0.0;
    size_t numberPoints = 100;

    //create a random viewpoint pose
    opengv::translation_t position = opengv::generateRandomTranslation(2.0);
    opengv::rotation_t rotation = opengv::generateRandomRotation(0.5);

    //create a fake central camera
    opengv::translations_t camOffsets;
    opengv::rotations_t camRotations;
    opengv::generateCentralCameraSystem(camOffsets, camRotations);

    //derive correspondences based on random point-cloud
    opengv::bearingVectors_t bearingVectors;
    opengv::points_t points;
    std::vector<int> camCorrespondences; //unused in the central case!
    Eigen::MatrixXd gt(3, numberPoints);

    opengv::generateRandom2D3DCorrespondences(
        position, rotation, camOffsets, camRotations, numberPoints, noise, outlierFraction,
        bearingVectors, points, camCorrespondences, gt);

    //print the experiment characteristics
     opengv::printExperimentCharacteristics(
      position, rotation, noise, outlierFraction );

    //create a central absolute adapter
    opengv::absolute_pose::CentralAbsoluteAdapter adapter(
      bearingVectors,
      points,
      rotation );

  //timer
  struct timeval tic;
  struct timeval toc;
  size_t iterations = 50;

  //run the experiments
  /*
  std::cout << "running Kneip's P2P (first two correspondences)" << std::endl;
  opengv::translation_t p2p_translation;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    p2p_translation = opengv::absolute_pose::p2p(adapter);
  gettimeofday( &toc, 0 );
  double p2p_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running Kneip's P3P (first three correspondences)" << std::endl;
   opengv::transformations_t p3p_kneip_transformations;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    p3p_kneip_transformations = opengv::absolute_pose::p3p_kneip(adapter);
  gettimeofday( &toc, 0 );
  double p3p_kneip_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running Gao's P3P (first three correspondences)" << std::endl;
  opengv::transformations_t p3p_gao_transformations;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    p3p_gao_transformations = opengv::absolute_pose::p3p_gao(adapter);
  gettimeofday( &toc, 0 );
  double p3p_gao_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running epnp (all correspondences)" << std::endl;
  opengv::transformation_t epnp_transformation;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    epnp_transformation = opengv::absolute_pose::epnp(adapter);
  gettimeofday( &toc, 0 );
  double epnp_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running epnp with 6 correspondences" << std::endl;
  std::vector<int> indices6 = opengv::getNindices(6);
  opengv::transformation_t epnp_transformation_6 =
      opengv::absolute_pose::epnp( adapter, indices6 );

  std::cout << "running upnp with all correspondences" << std::endl;
  opengv::transformations_t upnp_transformations;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    upnp_transformations = opengv::absolute_pose::upnp(adapter);
  gettimeofday( &toc, 0 );
  double upnp_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

    //LOG_INFO("End of PNP sample");

//initialize random seed
  initializeRandomSeed();
  
  //set experiment parameters
  double noise = 0.0;
  double outlierFraction = 0.0;
  size_t numberPoints = 100;

  //create a random viewpoint pose
  translation_t position = generateRandomTranslation(2.0);
  rotation_t rotation = generateRandomRotation(0.5);
  
  //create a fake central camera
  translations_t camOffsets;
  rotations_t camRotations;
  generateCentralCameraSystem( camOffsets, camRotations );
  
  //derive correspondences based on random point-cloud
  bearingVectors_t bearingVectors;
  points_t points;
  std::vector<int> camCorrespondences; //unused in the central case!
  Eigen::MatrixXd gt(3,numberPoints);
  generateRandom2D3DCorrespondences(
      position, rotation, camOffsets, camRotations, numberPoints, noise, outlierFraction,
      bearingVectors, points, camCorrespondences, gt );

  //print the experiment characteristics
  printExperimentCharacteristics(
      position, rotation, noise, outlierFraction );

  //create a central absolute adapter
  absolute_pose::CentralAbsoluteAdapter adapter(
      bearingVectors,
      points,
      rotation );

  //timer
  struct timeval tic;
  struct timeval toc;
  size_t iterations = 50;

  //run the experiments
  std::cout << "running Kneip's P2P (first two correspondences)" << std::endl;
  translation_t p2p_translation;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    p2p_translation = absolute_pose::p2p(adapter);
  gettimeofday( &toc, 0 );
  double p2p_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running Kneip's P3P (first three correspondences)" << std::endl;
  transformations_t p3p_kneip_transformations;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    p3p_kneip_transformations = absolute_pose::p3p_kneip(adapter);
  gettimeofday( &toc, 0 );
  double p3p_kneip_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running Gao's P3P (first three correspondences)" << std::endl;
  transformations_t p3p_gao_transformations;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    p3p_gao_transformations = absolute_pose::p3p_gao(adapter);
  gettimeofday( &toc, 0 );
  double p3p_gao_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running epnp (all correspondences)" << std::endl;
  transformation_t epnp_transformation;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    epnp_transformation = absolute_pose::epnp(adapter);
  gettimeofday( &toc, 0 );
  double epnp_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running epnp with 6 correspondences" << std::endl;
  std::vector<int> indices6 = getNindices(6);
  transformation_t epnp_transformation_6 =
      absolute_pose::epnp( adapter, indices6 );

  std::cout << "running upnp with all correspondences" << std::endl;
  transformations_t upnp_transformations;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    upnp_transformations = absolute_pose::upnp(adapter);
  gettimeofday( &toc, 0 );
  double upnp_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "running upnp with 3 correspondences" << std::endl;
  std::vector<int> indices3 = getNindices(3);
  transformations_t upnp_transformations_3 =
      absolute_pose::upnp( adapter, indices3 );

  std::cout << "setting perturbed pose";
  std::cout << "and performing nonlinear optimization" << std::endl;
  //add a small perturbation to the pose
  translation_t t_perturbed; rotation_t R_perturbed;
  getPerturbedPose( position, rotation, t_perturbed, R_perturbed, 0.1 );
  transformation_t nonlinear_transformation;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
  {
    adapter.sett(t_perturbed);
    adapter.setR(R_perturbed);
    nonlinear_transformation = absolute_pose::optimize_nonlinear(adapter);
  }
  gettimeofday( &toc, 0 );
  double nonlinear_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "setting perturbed pose";
  std::cout << "and performing nonlinear optimization with 10 correspondences";
  std::cout << std::endl;
  std::vector<int> indices10 = getNindices(10);
  //add a small perturbation to the pose
  getPerturbedPose( position, rotation, t_perturbed, R_perturbed, 0.1 );
  adapter.sett(t_perturbed);
  adapter.setR(R_perturbed);
  transformation_t nonlinear_transformation_10 =
      absolute_pose::optimize_nonlinear(adapter,indices10);

  //print the results
  std::cout << "results from P2P algorithm:" << std::endl;
  std::cout << p2p_translation << std::endl << std::endl;
  std::cout << "results from Kneip's P3P algorithm:" << std::endl;
  for(size_t i = 0; i < p3p_kneip_transformations.size(); i++)
    std::cout << p3p_kneip_transformations[i] << std::endl << std::endl;
  std::cout << "results from Gao's P3P algorithm:" << std::endl;
  for(size_t i = 0; i < p3p_gao_transformations.size(); i++)
    std::cout << p3p_gao_transformations[i] << std::endl << std::endl;
  std::cout << "results from epnp algorithm:" << std::endl;
  std::cout << epnp_transformation << std::endl << std::endl;
  std::cout << "results from epnp algorithm with only 6 correspondences:";
  std::cout << std::endl;
  std::cout << epnp_transformation_6 << std::endl << std::endl;
  std::cout << "results from upnp:" << std::endl;
  for(size_t i = 0; i < upnp_transformations.size(); i++)
    std::cout << upnp_transformations[i] << std::endl << std::endl;
  std::cout << "results form upnp algorithm with only 3 correspondences:";
  std::cout << std::endl;
  for(size_t i = 0; i < upnp_transformations_3.size(); i++)
    std::cout << upnp_transformations_3[i] << std::endl << std::endl;
  std::cout << "results from nonlinear algorithm:" << std::endl;
  std::cout << nonlinear_transformation << std::endl << std::endl;
  std::cout << "results from nonlinear algorithm with only 10 correspondences:";
  std::cout << std::endl;
  std::cout << nonlinear_transformation_10 << std::endl << std::endl;

  std::cout << "timings from P2P algorithm: ";
  std::cout << p2p_time << std::endl;
  std::cout << "timings from Kneip's P3P algorithm: ";
  std::cout << p3p_kneip_time << std::endl;
  std::cout << "timings from Gao's P3P algorithm: ";
  std::cout << p3p_gao_time << std::endl;
  std::cout << "timings from epnp algorithm: ";
  std::cout << epnp_time << std::endl;
  std::cout << "timings for the upnp algorithm: ";
  std::cout << upnp_time << std::endl;
  std::cout << "timings from nonlinear algorithm: ";
  std::cout << nonlinear_time << std::endl;

  */
    //initialize random seed
    initializeRandomSeed();

    //set experiment parameters
    double noise = 0.0;
    double outlierFraction = 0.0;
    size_t numberPoints = 100;

    //create a random viewpoint pose
    translation_t position = generateRandomTranslation(2.0);
    rotation_t rotation = generateRandomRotation(0.5);

    //create a fake central camera
    translations_t camOffsets;
    rotations_t camRotations;
    generateCentralCameraSystem(camOffsets, camRotations);

    //derive correspondences based on random point-cloud
    bearingVectors_t bearingVectors;
    points_t points;
    std::vector<int> camCorrespondences; //unused in the central case!
    Eigen::MatrixXd gt(3, numberPoints);
    generateRandom2D3DCorrespondences(
        position, rotation, camOffsets, camRotations, numberPoints, noise, outlierFraction,
        bearingVectors, points, camCorrespondences, gt);

    //print the experiment characteristics
    printExperimentCharacteristics(
        position, rotation, noise, outlierFraction);

    //create a central absolute adapter
    absolute_pose::CentralAbsoluteAdapter adapter(
        bearingVectors,
        points,
        rotation);

    //timer
    struct timeval tic;
    struct timeval toc;
    size_t iterations = 50;

    //run the experiments
    std::cout << "running Kneip's P2P (first two correspondences)" << std::endl;
    translation_t p2p_translation;
    gettimeofday(&tic, 0);
    for (size_t i = 0; i < iterations; i++)
        p2p_translation = absolute_pose::p2p(adapter);
    gettimeofday(&toc, 0);
    double p2p_time = TIMETODOUBLE(timeval_minus(toc, tic)) / iterations;

    std::cout << "running Kneip's P3P (first three correspondences)" << std::endl;
    transformations_t p3p_kneip_transformations;
    gettimeofday(&tic, 0);
    for (size_t i = 0; i < iterations; i++)
        p3p_kneip_transformations = absolute_pose::p3p_kneip(adapter);
    gettimeofday(&toc, 0);
    double p3p_kneip_time = TIMETODOUBLE(timeval_minus(toc, tic)) / iterations;

    std::cout << "running Gao's P3P (first three correspondences)" << std::endl;
    transformations_t p3p_gao_transformations;
    gettimeofday(&tic, 0);
    for (size_t i = 0; i < iterations; i++)
        p3p_gao_transformations = absolute_pose::p3p_gao(adapter);
    gettimeofday(&toc, 0);
    double p3p_gao_time = TIMETODOUBLE(timeval_minus(toc, tic)) / iterations;

    std::cout << "running epnp (all correspondences)" << std::endl;
    transformation_t epnp_transformation;
    gettimeofday(&tic, 0);
    for (size_t i = 0; i < iterations; i++)
        epnp_transformation = absolute_pose::epnp(adapter);
    gettimeofday(&toc, 0);
    double epnp_time = TIMETODOUBLE(timeval_minus(toc, tic)) / iterations;

    std::cout << "running epnp with 6 correspondences" << std::endl;
    std::vector<int> indices6 = getNindices(6);
    transformation_t epnp_transformation_6 =
        absolute_pose::epnp(adapter, indices6);

    std::cout << "running upnp with all correspondences" << std::endl;
    transformations_t upnp_transformations;
    gettimeofday(&tic, 0);
    for (size_t i = 0; i < iterations; i++)
        upnp_transformations = absolute_pose::upnp(adapter);
    gettimeofday(&toc, 0);
    double upnp_time = TIMETODOUBLE(timeval_minus(toc, tic)) / iterations;

    std::cout << "running upnp with 3 correspondences" << std::endl;
    std::vector<int> indices3 = getNindices(3);
    transformations_t upnp_transformations_3 =
        absolute_pose::upnp(adapter, indices3);

    std::cout << "setting perturbed pose";
    std::cout << "and performing nonlinear optimization" << std::endl;
    //add a small perturbation to the pose
    translation_t t_perturbed;
    rotation_t R_perturbed;
    getPerturbedPose(position, rotation, t_perturbed, R_perturbed, 0.1);
    transformation_t nonlinear_transformation;
    gettimeofday(&tic, 0);
    for (size_t i = 0; i < iterations; i++)
    {
        adapter.sett(t_perturbed);
        adapter.setR(R_perturbed);
        nonlinear_transformation = absolute_pose::optimize_nonlinear(adapter);
    }
    gettimeofday(&toc, 0);
    double nonlinear_time = TIMETODOUBLE(timeval_minus(toc, tic)) / iterations;

    std::cout << "setting perturbed pose";
    std::cout << "and performing nonlinear optimization with 10 correspondences";
    std::cout << std::endl;
    std::vector<int> indices10 = getNindices(10);
    //add a small perturbation to the pose
    getPerturbedPose(position, rotation, t_perturbed, R_perturbed, 0.1);
    adapter.sett(t_perturbed);
    adapter.setR(R_perturbed);
    transformation_t nonlinear_transformation_10 =
        absolute_pose::optimize_nonlinear(adapter, indices10);

    //print the results
    std::cout << "results from P2P algorithm:" << std::endl;
    std::cout << p2p_translation << std::endl
              << std::endl;
    std::cout << "results from Kneip's P3P algorithm:" << std::endl;
    for (size_t i = 0; i < p3p_kneip_transformations.size(); i++)
        std::cout << p3p_kneip_transformations[i] << std::endl
                  << std::endl;
    std::cout << "results from Gao's P3P algorithm:" << std::endl;
    for (size_t i = 0; i < p3p_gao_transformations.size(); i++)
        std::cout << p3p_gao_transformations[i] << std::endl
                  << std::endl;
    std::cout << "results from epnp algorithm:" << std::endl;
    std::cout << epnp_transformation << std::endl
              << std::endl;
    std::cout << "results from epnp algorithm with only 6 correspondences:";
    std::cout << std::endl;
    std::cout << epnp_transformation_6 << std::endl
              << std::endl;
    std::cout << "results from upnp:" << std::endl;
    for (size_t i = 0; i < upnp_transformations.size(); i++)
        std::cout << upnp_transformations[i] << std::endl
                  << std::endl;
    std::cout << "results form upnp algorithm with only 3 correspondences:";
    std::cout << std::endl;
    for (size_t i = 0; i < upnp_transformations_3.size(); i++)
        std::cout << upnp_transformations_3[i] << std::endl
                  << std::endl;
    std::cout << "results from nonlinear algorithm:" << std::endl;
    std::cout << nonlinear_transformation << std::endl
              << std::endl;
    std::cout << "results from nonlinear algorithm with only 10 correspondences:";
    std::cout << std::endl;
    std::cout << nonlinear_transformation_10 << std::endl
              << std::endl;

    std::cout << "timings from P2P algorithm: ";
    std::cout << p2p_time << std::endl;
    std::cout << "timings from Kneip's P3P algorithm: ";
    std::cout << p3p_kneip_time << std::endl;
    std::cout << "timings from Gao's P3P algorithm: ";
    std::cout << p3p_gao_time << std::endl;
    std::cout << "timings from epnp algorithm: ";
    std::cout << epnp_time << std::endl;
    std::cout << "timings for the upnp algorithm: ";
    std::cout << upnp_time << std::endl;
    std::cout << "timings from nonlinear algorithm: ";
    std::cout << nonlinear_time << std::endl;

    return (0);
}
