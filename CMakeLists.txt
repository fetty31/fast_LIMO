cmake_minimum_required(VERSION 3.5)
project(fast_limo)

# Default to C++17
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

option(ENABLE_RELOCATION "Enable relocation module if dependencies are available" ON)

find_package( PCL 1.10 REQUIRED )
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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nano_gicp REQUIRED)

find_package(robin QUIET)
find_package(kiss_matcher QUIET)
find_package(TBB QUIET)

set(USE_RELOCATION FALSE)
if(ENABLE_RELOCATION AND robin_FOUND AND kiss_matcher_FOUND AND TBB_FOUND)
  message(STATUS "ROBIN, KISS-Matcher and TBB found: enabling relocation features.")
  set(USE_RELOCATION TRUE)
  add_compile_definitions(USE_RELOCATION)
else()
  message(WARNING "Relocation dependencies NOT found. Relocation will be disabled.\n\
  One of the following dependencies is missing:\n- KISS-Matcher\n- ROBIN\n- TBB")
endif()

# ==================== Paths de includes propios ====================
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  # headers generados de servicios
)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

message("PCL version: ${PCL_VERSION}")

# IKFOM
add_library(ikfom
            include/IKFoM/IKFoM_toolkit
            include/IKFoM/use-ikfom.cpp
            )

target_link_libraries(ikfom ${PCL_LIBRARIES} ${OpenMP_LIBS})
target_include_directories(ikfom
  PUBLIC include/IKFoM/ include/IKFoM/IKFoM_toolkit 
)

set(FAST_LIMO_CORE_SOURCES
  include/fast_limo/Modules/Localizer.cpp
  include/fast_limo/Modules/Mapper.cpp

  include/fast_limo/Objects/State.cpp
  include/fast_limo/Objects/Plane.cpp
  include/fast_limo/Objects/Match.cpp
  include/fast_limo/Objects/Octree.hpp
)

if(USE_RELOCATION)
  list(APPEND FAST_LIMO_CORE_SOURCES
    include/fast_limo/Modules/Relocator.cpp
  )
endif()

# Fast LIMO
add_library(${PROJECT_NAME}_core ${FAST_LIMO_CORE_SOURCES})

target_link_libraries(${PROJECT_NAME}_core
  ${PCL_LIBRARIES}
  ${OpenMP_LIBS}
  ikfom
)

target_compile_options(${PROJECT_NAME}_core PRIVATE ${OpenMP_FLAGS})

if(USE_RELOCATION)
  target_link_libraries(${PROJECT_NAME}_core
    TBB::tbb
    kiss_matcher::kiss_matcher_core
    robin::robin
  )
  ament_target_dependencies(${PROJECT_NAME}_core nano_gicp)
endif()


# ROS execs
add_executable(${PROJECT_NAME}_multi_exec
              src/LimoWrapper.cpp
              )
ament_target_dependencies(${PROJECT_NAME}_multi_exec rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros pcl_conversions)
target_link_libraries(${PROJECT_NAME}_multi_exec ${PROJECT_NAME}_core ${PCL_LIBRARIES})

if(USE_RELOCATION)
  add_executable(${PROJECT_NAME}_reloca_exec
                src/RelocaWrapper.cpp
                )
  ament_target_dependencies(${PROJECT_NAME}_reloca_exec rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros pcl_conversions)
  target_link_libraries(${PROJECT_NAME}_reloca_exec ${PROJECT_NAME}_core ${PCL_LIBRARIES})
endif()

# Binaries
install( TARGETS 
  ${PROJECT_NAME}_multi_exec
  DESTINATION lib/${PROJECT_NAME} )

if(USE_RELOCATION)
  install( TARGETS
    ${PROJECT_NAME}_reloca_exec
    DESTINATION lib/${PROJECT_NAME} )
endif()

# Install launch files.
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  "srv/SendPointCloud.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(${PROJECT_NAME}_multi_exec
  ${cpp_typesupport_target}
)

if(USE_RELOCATION)
target_link_libraries(${PROJECT_NAME}_reloca_exec
  ${cpp_typesupport_target}
)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()