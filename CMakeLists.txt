cmake_minimum_required(VERSION 3.8)
project(covariance_geometry_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
  covariance_geometry_cpp
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/covariance_geometry_cpp
  GIT_REPOSITORY https://github.com/andreaostuni/covariance_geometry.git
  TIMEOUT 10
  UPDATE_COMMAND ${GIT_EXECUTABLE} pull
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/covariance_geometry_cpp/src
  LOG_DOWNLOAD ON
)

add_definitions(${EIGEN3_DEFINITIONS})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/covariance_geometry_cpp/src/include
)

add_library(${PROJECT_NAME} SHARED src/covariance_geometry_ros.cpp)
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_dependencies(
  ${PROJECT_NAME}
  covariance_geometry_cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}/covariance_geometry_cpp/src/lib/libcovariance_geometry_cpp.so
)

ament_target_dependencies(
  ${PROJECT_NAME}
  Eigen3
  geometry_msgs
  tf2_ros
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/covariance_geometry_cpp/src/include/ DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/covariance_geometry_cpp/src/lib/ DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies( 
  Eigen3
  covariance_geometry_cpp
  geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  set(TEST_NAMES
    conversion_test
    # composition_test
    )
  
  foreach(TEST_NAME ${TEST_NAMES})
    ament_add_gtest(${TEST_NAME} test/${TEST_NAME}.cpp)

    add_dependencies(
      ${TEST_NAME}
      covariance_geometry_cpp
    )

    target_link_libraries(
      ${TEST_NAME}
      ${CMAKE_CURRENT_BINARY_DIR}/covariance_geometry_cpp/src/lib/libcovariance_geometry_cpp.so
    )

    ament_target_dependencies(
      ${TEST_NAME}
      Eigen3
      geometry_msgs
      tf2_ros
    )

  endforeach()
endif()

ament_package()
