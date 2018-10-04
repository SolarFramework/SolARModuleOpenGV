cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "1.0.0")
project("SolARModuleOpenGV")
set (SOURCES 
	src/SolARModuleOpengv.cpp 
	src/SolARTriangulationOpengv.cpp
    )

set (HEADERS
	interfaces/SolARModuleOpengv_traits.h 
	interfaces/SolAROpengvAPI.h 
	interfaces/SolAROpenGVHelper.h 
	interfaces/SolARTriangulationOpengv.h
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


