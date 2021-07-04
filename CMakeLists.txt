# Replace the main and main to build the executables

project(stsSynthesis,)
cmake_minimum_required(VERSION 3.2)

set( PROJ_SOURCES "src/main.cpp" "src/modelFunsCommon.cpp" "src/modelFunsStanding.cpp" "src/modelFunsSitting.cpp" "src/parameterization.cpp" 
					"src/MtxNew.cpp" "src/WrapCylinderNew.cpp" "src/WrapMathNew.cpp" "src/RegisterTypes_osimPlugin.cpp" "src/seatConstraintHandler.cpp"
					"src/termination.cpp")

set( PROJ_INC "inc/" )

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

include_directories(
	${PROJ_INC}
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/spdlog/include
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/Simbody/include/simbody
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/include
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/include/OpenSim
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/include/OpenSim/Common
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/include/OpenSim/Simulation/Wrap
	/home/vinay/Repos/libcmaes/src
	/home/vinay/Repos/libcmaes/build/src/
	/usr/include/eigen3)

link_directories(
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/lib
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/spdlog/lib
	/home/vinay/OpenSim/moco_dependencies_install/opensim-core/sdk/Simbody/lib
	/home/vinay/Repos/libcmaes/build/src/)

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