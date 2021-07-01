project(stsSynthesis,)
cmake_minimum_required(VERSION 3.2)

set( PROJ_SOURCES "src/replay.cpp" "src/modelFunsCommon.cpp" "src/modelFunsStanding.cpp" "src/modelFunsSitting.cpp" "src/parameterization.cpp" 
					"src/MtxNew.cpp" "src/WrapCylinderNew.cpp" "src/WrapMathNew.cpp" "src/RegisterTypes_osimPlugin.cpp" "src/termination.cpp"
					"src/seatConstraintHandler.cpp")

set( PROJ_INC "inc/" )

##set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

include_directories(
	${PROJ_INC}
	/home/vinay/OpenSim/opensim_install/include
	/home/vinay/OpenSim/opensim_install/include/OpenSim
	/home/vinay/OpenSim/opensim_install/include/OpenSim/Common
	/home/vinay/OpenSim/opensim_install/include/OpenSim/Simulation/Wrap
	/home/vinay/OpenSim/opensim_install/include/simbody
	/home/vinay/Repos/libcmaes/src
	/home/vinay/Repos/libcmaes/build/src/
	/usr/include/eigen3)

link_directories(
	/home/vinay/OpenSim/opensim_install/lib
	/home/vinay/Repos/libcmaes/build/src/)

add_executable( replay ${PROJ_SOURCES} )

target_link_libraries( replay
osimActuators 
osimAnalyses
osimCommon 
osimSimulation
osimTools
SimTKcommon
SimTKmath 
SimTKsimbody
cmaes
)