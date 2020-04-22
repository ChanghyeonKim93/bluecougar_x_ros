cmake_minimum_required(VERSION 2.8.3)
project(bluecougar)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  std_msgs
  message_filters
  image_transport
)

catkin_package(
	DEPENDS Eigen
)

set(mvIMPACT_DIR /opt/mvIMPACT_Acquire)
set(mvIMPACT_LIBRARIES_DIR /opt/mvIMPACT_Acquire/lib/x86_64)

include_directories( 
	${mvIMPACT_DIR}
)
link_directories(
	${mvIMPACT_LIBRARIES_DIR}
)
link_libraries(
	mvDeviceManager 
	mvImageProcessing 
	mvIMPACT_Acquire.java
	mvPropHandling  
	usb-1.0
)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

set(SRC ${SRC} src/main.cpp)

set(DCMAKE_build_type release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")

add_executable (bluecougar_multiple_node ${SRC} )
target_link_libraries (bluecougar_multiple_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries (bluecougar_multiple_node ${Eigen_LIBRARIES} )
