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

#include <boost/log/core.hpp>

using namespace Eigen;
using namespace opengv;

#include "core/Log.h"

// ADD COMPONENTS HEADERS HERE

#include "xpcf/xpcf.h"

#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
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

    //initialize random seed
    initializeRandomSeed();

    //set experiment parameters
    double noise = 0.0;
    double outlierFraction = 0.0;
    size_t numberPoints = 100;

    //create a fake central camera
    translations_t camOffsets;
    rotations_t camRotations;
    generateCentralCameraSystem(camOffsets, camRotations);

    //create a random viewpoint pose
    translation_t position = generateRandomTranslation(2.0);
    rotation_t rotation = generateRandomRotation(0.5);

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

    try {

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolARTest_ModuleOpenGV_PnP_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenGV_PnP_conf.xml")
            return -1;
        }

        CamCalibration  intrinsicParams;
        //set to identity Matrix
        intrinsicParams(0,0) = 1; intrinsicParams(0,1) = 0; intrinsicParams(0,2) = 0;
        intrinsicParams(1,0) = 0; intrinsicParams(1,1) = 1; intrinsicParams(1,2) = 0;
        intrinsicParams(2,0) = 0; intrinsicParams(2,1) = 0; intrinsicParams(2,2) = 1;

        CamDistortion   distortionParams;
        distortionParams(0,0) =0;
        distortionParams(1,0) =0;
        distortionParams(2,0) =0;
        distortionParams(3,0) =0;
        distortionParams(4,0) =0;

		CameraParameters camParams;
		camParams.intrinsic = intrinsicParams;
		camParams.distortion = distortionParams;

        auto poseEstimation_p3p_kneip = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D3D>("OpenGVP3PKNEIP");
        auto poseEstimation_p3p_epnp  = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D3D>("OpenGVEPNP");
        auto poseEstimation_p3p_gao   = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D3D>("OpenGVP3PGAO");
        auto poseEstimation_p3p_upnp  = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D3D>("OpenGVUPNP");

        auto poseEstimation_epnp_sac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("OpenGVSACEPNP");
        auto poseEstimation_p3p_sac_gao = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("OpenGVSACP3PGAO");
        auto poseEstimation_p3p_sac_kneip  = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("OpenGVSACP3PKNEIP");

         //synthetize 2d points and 3d points to test the components.
         std::vector<Point2Df>  imagePoints;
         std::vector<Point3Df>  worldPoints;

         unsigned int Npoints = 1024;

         imagePoints.resize(Npoints);
         worldPoints.resize(Npoints);

         for(unsigned int kc =0; kc < Npoints; kc++){

             double minDepth = 4;
             double maxDepth = 8;

             Eigen::Vector3d  current_random_point = generateRandomPoint( maxDepth, minDepth );

             //get the camera transformation
             translation_t camOffset = camOffsets[0];
             rotation_t camRotation = camRotations[0];

             //project the point into the viewpoint frame
             point_t bodyPoint = rotation.transpose()*(current_random_point - position);

             imagePoints[kc] = Point2Df(bodyPoint(0,0), bodyPoint(1,0)) ;
             worldPoints[kc] = Point3Df(current_random_point(0,0),current_random_point(1,0),current_random_point(2,0));
         }

         Transform3Df pose_p3p_kneip;
         Transform3Df pose_p3p_gao;
         Transform3Df pose_p3p_epnp;
         Transform3Df pose_p3p_upnp;
         Transform3Df pose_p3p_epnp_sac;
         Transform3Df pose_p3p_gao_sac;
         Transform3Df pose_p3p_kneip_sac;

         std::vector<uint32_t> inlier_epnp_sac;
         std::vector<uint32_t> inlier_gao_sac;
         std::vector<uint32_t> inlier_kneip_sac;

         //timer
         struct timeval tic;
         struct timeval toc;
         size_t iterations = 50;

         std::cout<< "********************* pose_p3p_kneip **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_p3p_kneip->estimate(imagePoints, worldPoints, camParams, pose_p3p_kneip);
         std::cout<<pose_p3p_kneip.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_p3p_kneip_sac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_p3p_sac_kneip->estimate(imagePoints, worldPoints, camParams, inlier_kneip_sac, pose_p3p_kneip_sac);
         std::cout<<pose_p3p_kneip_sac.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_p3p_gao **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_p3p_gao->estimate(imagePoints, worldPoints, camParams, pose_p3p_gao);
         std::cout<<pose_p3p_kneip_sac.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_p3p_gao_sac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_p3p_sac_gao->estimate(imagePoints, worldPoints, camParams, inlier_gao_sac, pose_p3p_gao_sac);
         std::cout<<pose_p3p_gao_sac.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_p3p_epnp **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_p3p_epnp->estimate(imagePoints, worldPoints, camParams, pose_p3p_epnp);
         std::cout<<pose_p3p_epnp.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_p3p_epnp_sac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_epnp_sac->estimate(imagePoints, worldPoints, camParams, inlier_epnp_sac, pose_p3p_epnp_sac);
         std::cout<<pose_p3p_epnp_sac.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_p3p_upnp **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_p3p_upnp->estimate(imagePoints, worldPoints, camParams, pose_p3p_upnp);
         std::cout<<pose_p3p_upnp.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

    return 0;
}
