#
# Author: Jakob Engel <jajuengel@gmail.com>
# Contributor: Stefan Wilkes <stefan.wilkes@gmail.com>
#
cmake_minimum_required(VERSION 2.8.3)
project(tum_ardrone)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  roscpp
  rospy
)

# Compile third party libs
include(ExternalProject)
ExternalProject_Add(thirdparty
	URL ${PROJECT_SOURCE_DIR}/thirdparty/thirdparty.tar.gz
	PREFIX ${CMAKE_BINARY_DIR}/thirdparty
	CONFIGURE_COMMAND ""
	BUILD_COMMAND make
	INSTALL_COMMAND ""
	BUILD_IN_SOURCE 1
)

# ------------------- add dynamic reconfigure api ------------------------------------
generate_dynamic_reconfigure_options(
  cfg/AutopilotParams.cfg
  cfg/GUIParams.cfg
  cfg/StateestimationParams.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES filter_state.msg)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	SetReference.srv
	SetMaxControl.srv
	SetInitialReachDistance.srv
	SetStayWithinDistance.srv
	SetStayTime.srv
)

## Generate added messages 
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS message_runtime std_msgs ardrone_autonomy)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# --------------------------- stateestimation & PTAMM --------------------------------
# set header ans source files
set(STATEESTIMATION_SOURCE_FILES       
  src/stateestimation/main_stateestimation.cpp
  src/stateestimation/Predictor.cpp
  src/stateestimation/EstimationNode.cpp
  src/stateestimation/DroneKalmanFilter.cpp
  src/stateestimation/MapView.cpp
  src/stateestimation/PTAMWrapper.cpp
  src/stateestimation/PTAMM/ATANCamera.cc
  src/stateestimation/PTAMM/Bundle.cc
  src/stateestimation/PTAMM/HomographyInit.cc
  src/stateestimation/PTAMM/KeyFrame.cc
  src/stateestimation/PTAMM/Map.cc
  src/stateestimation/PTAMM/MapMaker.cc
  src/stateestimation/PTAMM/MapPoint.cc
  src/stateestimation/PTAMM/MiniPatch.cc
  src/stateestimation/PTAMM/PatchFinder.cc
  src/stateestimation/PTAMM/Relocaliser.cc
  src/stateestimation/PTAMM/ShiTomasi.cc
  src/stateestimation/PTAMM/SmallBlurryImage.cc
  src/stateestimation/PTAMM/Tracker.cc
  src/stateestimation/PTAMM/MapLockManager.cc
  src/stateestimation/PTAMM/tinyxml.cc
  src/stateestimation/PTAMM/tinyxmlerror.cc
  src/stateestimation/PTAMM/tinyxmlparser.cc
  src/stateestimation/PTAMM/Utils.cc
  src/stateestimation/PTAMM/MD5.cc
  src/stateestimation/PTAMM/MD5Wrapper.cc
)
set(STATEESTIMATION_HEADER_FILES    
  src/HelperFunctions.h   
  src/stateestimation/Predictor.h 
  src/stateestimation/EstimationNode.h
  src/stateestimation/PTAMWrapper.h
  src/stateestimation/DroneKalmanFilter.h
  src/stateestimation/MapView.h
  src/stateestimation/PTAMM/ATANCamera.h
  src/stateestimation/PTAMM/Bundle.h
  src/stateestimation/PTAMM/customFixes.h
  src/stateestimation/PTAMM/HomographyInit.h
  src/stateestimation/PTAMM/KeyFrame.h
  src/stateestimation/PTAMM/LevelHelpers.h
  src/stateestimation/PTAMM/Map.h
  src/stateestimation/PTAMM/MapMaker.h
  src/stateestimation/PTAMM/MapPoint.h
  src/stateestimation/PTAMM/MEstimator.h
  src/stateestimation/PTAMM/MiniPatch.h
  src/stateestimation/PTAMM/OpenGL.h
  src/stateestimation/PTAMM/PatchFinder.h
  src/stateestimation/PTAMM/Relocaliser.h
  src/stateestimation/PTAMM/settingsCustom.h
  src/stateestimation/PTAMM/ShiTomasi.h
  src/stateestimation/PTAMM/SmallBlurryImage.h
  src/stateestimation/PTAMM/SmallMatrixOpts.h
  src/stateestimation/PTAMM/TrackerData.h
  src/stateestimation/PTAMM/Tracker.h
  src/stateestimation/PTAMM/VideoSource.h
  src/stateestimation/PTAMM/MapLockManager.h
  src/stateestimation/PTAMM/tinyxml.h
  src/stateestimation/PTAMM/Utils.h
  src/stateestimation/PTAMM/MD5.h
  src/stateestimation/PTAMM/MD5Wrapper.h
)

# set required libs and headers
include_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/TooN/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/include 
)  

link_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/lib 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/lib 
)
set(PTAMM_LIBRARIES GL glut cvd GVars3 blas lapack)
add_definitions(-DKF_REPROJ)

# build!
add_executable(drone_stateestimation ${STATEESTIMATION_SOURCE_FILES} ${STATEESTIMATION_HEADER_FILES})
set_target_properties(drone_stateestimation PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3") 
target_link_libraries(drone_stateestimation ${PTAMM_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(drone_stateestimation thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ------------------------- autopilot & KI -----------------------------------------
# set header ans source files
set(AUTOPILOT_SOURCE_FILES         
	src/autopilot/main_autopilot.cpp  
	src/autopilot/ControlNode.cpp
	src/autopilot/DroneController.cpp
	src/autopilot/KI/KILand.cpp
	src/autopilot/KI/KIAutoInit.cpp
	src/autopilot/KI/KIFlyTo.cpp
)
set(AUTOPILOT_HEADER_FILES        
	src/autopilot/ControlNode.h
	src/autopilot/DroneController.h
	src/autopilot/KI/KILand.h
	src/autopilot/KI/KIAutoInit.h
	src/autopilot/KI/KIFlyTo.h
	src/autopilot/KI/KIProcedure.h
)

# build!
add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})
target_link_libraries(drone_autopilot ${catkin_LIBRARIES})
add_dependencies(drone_autopilot thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ---------------------------- GUI --------------------------------------------------
# set header ans source files
set(GUI_SOURCE_FILES         
	src/UINode/main_GUI.cpp  
	src/UINode/tum_ardrone_gui.cpp  
	src/UINode/RosThread.cpp  
	src/UINode/PingThread.cpp  
)

set(GUI_HEADER_FILES        
	src/UINode/tum_ardrone_gui.h 
	src/UINode/RosThread.h  
	src/UINode/PingThread.h  
)

# *.ui
set(GUI_UI_FILES        
	src/UINode/tum_ardrone_gui.ui 
)

# *.qrc
set(GUI_RESOURCE_FILES        
)

# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# build!
add_executable(drone_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(drone_gui ${QT_LIBRARIES} cvd ${catkin_LIBRARIES})
add_dependencies(drone_gui thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


