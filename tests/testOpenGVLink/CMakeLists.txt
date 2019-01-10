cmake_minimum_required(VERSION 3.7.2)

##################################################
project("testOpenGVLink")
set (SOURCES main.cpp)
##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()

# define targets (library, install and uninstall)
defineTargets("testOpenGVLink")

target_include_directories("testOpenGVLink" PRIVATE BCOMDEVROOT/thirdParties/opengv/1.0.0/interfaces/)
target_link_libraries("testOpenGVLink" BCOMDEVROOT/thirdParties/opengv/1.0.0/lib/x86_64/static/debug/libopengv.a)
target_link_libraries("testOpenGVLink" BCOMDEVROOT/thirdParties/opengv/1.0.0/lib/x86_64/static/debug/librandom_generators.a)


