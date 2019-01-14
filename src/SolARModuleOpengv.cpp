/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include <iostream>

#include "xpcf/module/ModuleFactory.h"
#include "SolARModuleOpengv_traits.h"

#include "SolARTriangulationOpengv.h"
#include "SolARPoseEstimationPnpOpengv.h"
#include "SolARPoseEstimationP3PKneip.h"

namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_MODULE("9f5f0fb9-a8c8-4532-bddf-f988052f63c3", "SolARModuleOpenGV", "SolARModuleOpenGV module description")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
   
     if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
     {

        errCode =  xpcf::tryCreateComponent<SolAR::MODULES::OPENGV::SolARTriangulationOpengv>(componentUUID,interfaceRef);
     }

    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
         errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENGV::SolARPoseEstimationPnpOpengv>(componentUUID,interfaceRef);
    }

    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
         errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENGV::SolARPoseEstimationP3PKneip>(componentUUID,interfaceRef);
    }

    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENGV::SolARTriangulationOpengv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENGV::SolARPoseEstimationPnpOpengv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENGV::SolARPoseEstimationP3PKneip)
XPCF_END_COMPONENTS_DECLARATION
