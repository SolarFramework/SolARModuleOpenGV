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
#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpengv_traits.h"
#include "api/solver/map/ITriangulator.h"
#include "api/input/devices/ICamera.h"
#include <opengv/types.hpp>
#include <core/Log.h>
#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
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
    if(xpcfComponentManager->load("conf_loading_test_opengv.xml")!=org::bcom::xpcf::_SUCCESS){
        LOG_ERROR("Failed to load the configuration file conf_loading_test_opengv.xml")
        return -1;
    }
#endif

    //instantiate a triangulation
    SRef<solver::map::ITriangulator> triangulator_opengv = xpcfComponentManager->create<SolAR::MODULES::OPENGV::Triangulation>()->bindTo<solver::map::ITriangulator>();
   

    opengv::translation_t position1 = Eigen::Vector3d::Zero();
    opengv::rotation_t rotation1 = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>();
    
    opengv::translation_t position2  = Eigen::Vector3d::Zero(); 
    opengv::rotation_t rotation2 = Eigen::Matrix<double, 3,3, Eigen::ColMajor>();  

   //Extract the relative pose
    opengv::translation_t relativePosition;
    opengv::rotation_t relativeRotation;
  
    relativeRotation = rotation1.transpose() * rotation2;
    relativePosition = rotation1.transpose() * (position2 - position1);


    opengv::bearingVectors_t bearingVectors1;
    opengv::bearingVectors_t bearingVectors2;
    
    size_t numberPoints = 10;
     for (size_t i = 0; i<numberPoints; i++) {

        bearingVectors1.push_back(opengv::point_t(i, i*2,1.0));
        bearingVectors2.push_back(opengv::point_t(i + 5 , i*2+5,1.0 ));
    
    }

    //create a central relative adapter and pass the relative pose
    opengv::relative_pose::CentralRelativeAdapter adapter(
      bearingVectors1,
      bearingVectors2,
      relativePosition,
      relativeRotation);

    std::cout<<"The module was loaded properly. "<<std::endl;

    return(0);
    
}
