cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_decisionmaking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_data
			 rosneuro_msgs)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(WTKPROCESSING REQUIRED wtkprocessing)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
#find_library(WTKPROCESSING_LIBRARIES wtkprocessing)
#find_path(WTKPROCESSING_INCLUDE_DIRS Buffer.hpp)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_data
	rosneuro_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
                    ${Eigen3_INCLUDE_DIRS}
		    ${WTKPROCESSING_INCLUDE_DIRS}
		    include
		    
)

add_definitions(${EIGEN3_DEFINITIONS})


add_library(${PROJECT_NAME}
	src/Integrator.cpp
)

target_link_libraries(${PROJECT_NAME} 
		     ${catkin_LIBRARIES} 
		     ${WTKPROCESSING_LIBRARIES}
			 ${Eigen3_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
	         ${catkin_EXPORTED_TARGETS}
)


#################
## Executables ##
#################


add_executable(test_decisionmaking src/test_decisionmaking.cpp)
target_link_libraries(test_decisionmaking rosneuro_decisionmaking)


