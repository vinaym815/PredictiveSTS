# Replace the main and main to build the executables

project(stsSynthesis,)
cmake_minimum_required(VERSION 3.2)

set( PROJ_SOURCES "src/main.cpp" "src/modelFunsCommon.cpp" "src/modelFunsStanding.cpp" "src/modelFunsSitting.cpp" "src/parameterization.cpp" 
					"src/RegisterTypes_osimPlugin.cpp" "src/seatConstraintHandler.cpp"
					"src/termination.cpp")

set( PROJ_INC "inc/" )

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

include_directories(
	${PROJ_INC}
	/home/opensim/Libs/opensim_dependencies_install/spdlog/include
	/home/opensim/Libs/opensim_dependencies_install/simbody/include/simbody
	/home/opensim/Libs/opensim-install/include
	/home/opensim/Libs/opensim-install/include/OpenSim
	/home/opensim/Libs/opensim-install/include/OpenSim/Common
	/home/opensim/Libs/opensim-install/include/OpenSim/Simulation/Wrap
	/home/opensim/Libs/libcmaes/src	
	/home/opensim/Libs/libcmaes/build/src
	/usr/include/eigen3)

link_directories(
	/home/opensim/Libs/opensim-install/lib
	/home/opensim/Libs/opensim_dependencies_install/spdlog/lib
	/home/opensim/Libs/opensim_dependencies_install/simbody/lib
	/home/opensim/Libs/libcmaes/build/src/)

add_executable( main ${PROJ_SOURCES} )

target_link_libraries( main
osimActuators
osimAnalyses
osimCommon
osimSimulation
osimTools
SimTKcommon
SimTKmath
SimTKsimbody
cmaes
pthread
)