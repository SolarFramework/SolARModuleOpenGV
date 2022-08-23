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
#include "random_generators.hpp"
#include <boost/log/core.hpp>
#include <core/Log.h>
#include <xpcf/xpcf.h>
#include <core/Timer.h>
#include <api/solver/pose/I3DTransformFinderFrom2D3D.h>
#include <api/solver/pose/I3DTransformSACFinderFrom2D3D.h>
#include <algorithm>
#include <random>

using namespace Eigen;
using namespace SolAR::PnPTest;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf = org::bcom::xpcf;

int main()
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

	LOG_ADD_LOG_TO_CONSOLE();

	//initialize random seed
	initializeRandomSeed();
    
    try {

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolARTest_ModuleOpenGV_PnP_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenGV_PnP_conf.xml")
            return -1;
        }

        auto poseEstimation_epnp_sac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("OpenGVSACEPNP");
        auto poseEstimation_p3p_sac_gao = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("OpenGVSACP3PGAO");
        auto poseEstimation_p3p_sac_kneip  = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("OpenGVSACP3PKNEIP");

		// Init camera parameters
		CameraParameters camParams;
		camParams.intrinsic = CamCalibration::Identity();
		camParams.distortion = CamDistortion::Zero();

		// generate a random pose
		Transform3Df poseGT;
		Vector3f position = generateRandomTranslation(2.0);
		Eigen::Matrix3f rotation = generateRandomRotation(0.5);
		poseGT.translation() = position;
		poseGT.linear() = rotation;
		Transform3Df poseGTInv = poseGT.inverse();
		std::cout << "Ground truth pose: \n" << poseGT.matrix() << std::endl;

		//synthetize 2d points and 3d points to test the components.
		std::vector<Point2Df>  imagePoints;
		std::vector<Point3Df>  worldPoints;

		// Nb of correspondences
		unsigned int nbPoints = 2000;

		imagePoints.resize(nbPoints);
		worldPoints.resize(nbPoints);

		for (unsigned int kc = 0; kc < nbPoints; kc++) {
			Eigen::Vector3f  current_random_point = generateRandom3DPointInFrontOfCam(poseGT);
			//project the point into the viewpoint frame
			Eigen::Vector3f pt2D = poseGTInv * current_random_point;

			imagePoints[kc] = Point2Df(pt2D(0) / pt2D(2), pt2D(1) / pt2D(2));
			worldPoints[kc] = Point3Df(current_random_point(0, 0), current_random_point(1, 0), current_random_point(2, 0));
		}

		// inject outliers
		float outlierFraction = 0.3f;
		uint32_t nbOutliers = (uint32_t)(nbPoints * outlierFraction);
		std::vector<bool> checkInOut(nbPoints, true);
		std::vector<uint32_t> inliersGT;
		for (int i = 0; i < nbOutliers; ++i)
			checkInOut[i] = false;
		auto rng = std::default_random_engine{};
		std::shuffle(checkInOut.begin(), checkInOut.end(), rng);
		for (int i = 0; i < nbPoints; ++i)
			if (!checkInOut[i])
				imagePoints[i] += Point2Df(10, 10);
			else
				inliersGT.push_back(i);

		// Verify output of pnp
		auto verifyPnP = [](const Transform3Df& poseGT,
			const std::vector<uint32_t>& inliersGT,
			const Transform3Df& pose,
			const std::vector<uint32_t>& inliers)
		{
			if (!(poseGT.matrix() - pose.matrix()).isZero(1e-3f))
				return false;
			if (inliersGT.size() != inliers.size())
				return false;
			for (int i = 0; i < inliers.size(); ++i)
				if (inliers[i] != inliersGT[i])
					return false;
			return true;
		};

        Transform3Df pose_p3p_epnp_sac;
        Transform3Df pose_p3p_gao_sac;
        Transform3Df pose_p3p_kneip_sac;

        std::vector<uint32_t> inlier_epnp_sac;
        std::vector<uint32_t> inlier_gao_sac;
        std::vector<uint32_t> inlier_kneip_sac;

        //timer
		size_t iterations = 500;
		Timer timer;

        std::cout<< "********************* pose_p3p_kneip_sac **************************"<<std::endl;
		timer.restart();
        poseEstimation_p3p_sac_kneip->estimate(imagePoints, worldPoints, camParams, inlier_kneip_sac, pose_p3p_kneip_sac);
		std::cout << "Computed in " << timer.elapsed() << "ms" << std::endl;
		std::cout << pose_p3p_kneip_sac.matrix() << std::endl;
		if (verifyPnP(poseGT, inliersGT, pose_p3p_kneip_sac, inlier_kneip_sac))
			std::cout << "====> OK\n\n";
		else
			std::cout << "====> NOK\n\n";

        std::cout<< "********************* pose_p3p_gao_sac **************************"<<std::endl;
		timer.restart();
        poseEstimation_p3p_sac_gao->estimate(imagePoints, worldPoints, camParams, inlier_gao_sac, pose_p3p_gao_sac);
		std::cout << "Computed in " << timer.elapsed() << "ms" << std::endl;
		std::cout << pose_p3p_gao_sac.matrix() << std::endl;
		if (verifyPnP(poseGT, inliersGT, pose_p3p_gao_sac, inlier_gao_sac))
			std::cout << "====> OK\n\n";
		else
			std::cout << "====> NOK\n\n";

        std::cout<< "********************* pose_p3p_epnp_sac **************************"<<std::endl;
		timer.restart();
        poseEstimation_epnp_sac->estimate(imagePoints, worldPoints, camParams, inlier_epnp_sac, pose_p3p_epnp_sac);
		std::cout << "Computed in " << timer.elapsed() << "ms" << std::endl;
		std::cout << pose_p3p_epnp_sac.matrix() << std::endl;
		if (verifyPnP(poseGT, inliersGT, pose_p3p_epnp_sac, inlier_epnp_sac))
			std::cout << "====> OK\n\n";
		else
			std::cout << "====> NOK\n\n";
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

    return 0;
}
