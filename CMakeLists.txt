cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(PatrolAgent src/PatrolAgent.cpp src/getgraph.cpp src/algorithms.cpp src/config.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

############ ALGORITMOS: ##############

rosbuild_add_executable(Random src/Random_Agent.cpp)
target_link_libraries(Random PatrolAgent)
rosbuild_add_executable(Conscientious_Reactive src/Conscientious_Reactive_Agent.cpp)
target_link_libraries(Conscientious_Reactive PatrolAgent)
rosbuild_add_executable(Conscientious_Cognitive src/Conscientious_Cognitive_Agent.cpp)
target_link_libraries(Conscientious_Cognitive PatrolAgent)
rosbuild_add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive_Agent.cpp)
target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent)
rosbuild_add_executable(Cyclic src/Cyclic_Agent.cpp)
target_link_libraries(Cyclic PatrolAgent)
rosbuild_add_executable(MSP src/MSP_Agent.cpp)
target_link_libraries(MSP PatrolAgent)
rosbuild_add_executable(GBS src/GBS_Agent.cpp)
target_link_libraries(GBS PatrolAgent)
rosbuild_add_executable(SEBS src/SEBS_Agent.cpp)
target_link_libraries(SEBS PatrolAgent)
rosbuild_add_executable(DTAGreedy src/DTAGreedy_Agent.cpp)
target_link_libraries(DTAGreedy PatrolAgent)
rosbuild_add_executable(DTASSI src/DTASSI_Agent.cpp)
target_link_libraries(DTASSI PatrolAgent)

############ RESULTS & MONITOR NODE: ##############
rosbuild_add_executable(monitor src/monitor.cpp src/getgraph.cpp)
rosbuild_add_executable(idlHistogram src/idlHistogram.cpp)
