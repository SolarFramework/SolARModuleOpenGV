cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.5.2")
project("SolARModuleOpenGV")
set (SOURCES 
	src/SolARModuleOpengv.cpp
	src/PoseEstimationEPnp.cpp
	src/PoseEstimationP3PGao.cpp
	src/PoseEstimationP3PKneip.cpp
	src/PoseEstimationUPnp.cpp
	src/PoseEstimationSACEPnp.cpp
	src/PoseEstimationSACP3PGao.cpp
	src/PoseEstimationSACP3PKneip.cpp
	src/Triangulation.cpp 
    )

set (HEADERS
	interfaces/SolARModuleOpengv_traits.h
	interfaces/SolAROpengvAPI.h
	interfaces/SolAROpenGVHelper.h
	interfaces/PoseEstimationEPnp.h
	interfaces/PoseEstimationP3PGao.h
	interfaces/PoseEstimationP3PKneip.h
	interfaces/PoseEstimationUPnp.h
	interfaces/PoseEstimationSACEPnp.h
	interfaces/PoseEstimationSACP3PGao.h
	interfaces/PoseEstimationSACP3PKneip.h
	interfaces/Triangulation.h 
	)

install (FILES "${CMAKE_CURRENT_SOURCE_DIR}/xpcf_SolARModuleOpenGV_registry.xml" DESTINATION $ENV{BCOMDEVROOT}/.xpcf/SolAR/	)

##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library" "")


