cmake_minimum_required(VERSION 3.0.2)
project(fast_limo)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf2
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp pcl_ros sensor_msgs geometry_msgs tf2
 DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INLCUDE_DIRS}
  ${PCL_INLCUDE_DIRS}
)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# IKD_TREE
add_library(ikd_tree 
              include/ikd-Tree/ikd_Tree/ikd_Tree.cpp
            )

target_link_libraries(ikd_tree ${PCL_LIBRARIES} ${OpenMP_LIBS})
target_include_directories(ikd_tree
  PUBLIC include/ikd-Tree/ikd_Tree
)

# IKFOM
add_library(ikfom
            include/IKFoM/IKFoM_toolkit
            include/IKFoM/use-ikfom.cpp
            )

target_link_libraries(ikfom ${PCL_LIBRARIES} ${OpenMP_LIBS})
target_include_directories(ikfom
  PUBLIC include/IKFoM/ include/IKFoM/IKFoM_toolkit 
)

# Fast LIMO
add_library(${PROJECT_NAME} 
            src/Modules/Localizer.cpp
            src/Modules/Mapper.cpp

            src/Objects/State.cpp
            src/Objects/Plane.cpp
            src/Objects/Match.cpp
            )
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenMP_LIBS} ikfom ikd_tree)

# ROS execs
add_executable(${PROJECT_NAME}_multi_exec
              src/main.cpp
              )
add_executable(${PROJECT_NAME}_exec
              src/onethread.cpp
              )

add_dependencies(${PROJECT_NAME}_multi_exec ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_exec ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_multi_exec ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Binaries
install( TARGETS ${PROJECT_NAME}_multi_exec
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
install( TARGETS ${PROJECT_NAME}_exec
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )