cmake_minimum_required(VERSION 3.0)
project( learn_slam )

# Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
message(STATUS "C++11 support has been enabled by default.")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")

# opencv
find_package( OpenCV REQUIRED )
SET(LIBS ${OpenCV_LIBS})
SET(DIRS ${OpenCV_INCLUDE_DIRS})

# eigen3
# include_directories("/usr/include/eigen3/")
find_package(Eigen3 3.3.0 REQUIRED)
LIST(APPEND DIRS ${EIGEN3_INCLUDE_DIR})
# sophus
# find_package( Sophus REQUIRED )

# find_package( glog REQUIRED )
# LIST(APPEND LIBS glog)

# ceres
find_package( Ceres REQUIRED)
LIST(APPEND DIRS "/usr/local/include/")
LIST(APPEND LIBS ${CERES_LIBRARIES})

# g2o
include(FindG2O.cmake)
find_package(G2O REQUIRED)
include_directories( ${G2O_INCLUDE_DIRS} )
# LIST(APPEND LIBS ${G2O_LIBRARIES})


message(STATUS "Include DIRS: " ${DIRS})
add_subdirectory(learn_so3)