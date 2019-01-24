#include <iostream>
#include <fstream>

#include <string>
#include <vector>


//#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>
#include <sstream>

#include "random_generators.hpp"
#include "experiment_helpers.hpp"
#include "time_measurement.hpp"

using namespace Eigen;
using namespace opengv;


#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"
#include "SolARModuleOpengv_traits.h"



// ADD COMPONENTS HEADERS HERE
#define USE_FREE


#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "xpcf/xpcf.h"

#include "api/image/IImageLoader.h"
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"

#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"



using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
//using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
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
    


#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

  SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

#ifdef USE_FREE
    if(xpcfComponentManager->load("conf_Pnp.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        //
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

    CamCalibration  intrinsicParams;
    CamDistortion   distorsionParams;
    distorsionParams[0] =0;
    distorsionParams[1] =0;
    distorsionParams[2] =0;
    distorsionParams[3] =0;
    distorsionParams[4] =0;

    //set to identity Matrix
    intrinsicParams(0,0) = 1; intrinsicParams(0,1) = 0; intrinsicParams(0,2) = 0;
    intrinsicParams(1,0) = 0; intrinsicParams(1,1) = 1; intrinsicParams(1,2) = 0;
    intrinsicParams(2,0) = 0; intrinsicParams(2,1) = 0; intrinsicParams(2,2) = 1;

    //
    auto poseEstimation_p3p_kneip = xpcfComponentManager->create<SolAR::MODULES::OPENGV::PoseEstimationP3PKneip>()->bindTo<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>();
    auto poseEstimation_p3p_gao   = xpcfComponentManager->create<SolAR::MODULES::OPENGV::PoseEstimationP3PGao>()->bindTo<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>();
    auto poseEstimation_p3p_epnp  = xpcfComponentManager->create<SolAR::MODULES::OPENGV::PoseEstimationEPnp>()->bindTo<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>();
    auto poseEstimation_p3p_upnp  = xpcfComponentManager->create<SolAR::MODULES::OPENGV::PoseEstimationUPnp>()->bindTo<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>();

    // initialize pose estimation
    poseEstimation_p3p_kneip->setCameraParameters(  intrinsicParams, distorsionParams);
    poseEstimation_p3p_gao->setCameraParameters(    intrinsicParams, distorsionParams);
    poseEstimation_p3p_epnp->setCameraParameters(   intrinsicParams, distorsionParams);
    poseEstimation_p3p_upnp->setCameraParameters(   intrinsicParams, distorsionParams);

    //  points
    //  camCorrespondences

     std::vector<SRef<Point2Df>>  imagePoints;
     std::vector<SRef<Point3Df>>  worldPoints;

     Transform3Df pose_p3p_kneip;
     Transform3Df pose_p3p_gao;
     Transform3Df pose_p3p_epnp;
     Transform3Df pose_p3p_upnp;

      poseEstimation_p3p_kneip->estimate(   imagePoints, worldPoints, pose_p3p_kneip);
      poseEstimation_p3p_gao->estimate(     imagePoints, worldPoints, pose_p3p_gao);
      poseEstimation_p3p_epnp->estimate(    imagePoints, worldPoints, pose_p3p_epnp);
      poseEstimation_p3p_upnp->estimate(    imagePoints, worldPoints, pose_p3p_upnp);



}
