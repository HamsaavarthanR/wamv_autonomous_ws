cmake_minimum_required(VERSION 3.0)
project(unitree_lidar_sdk)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

include_directories(include)

link_directories(lib/${CMAKE_SYSTEM_PROCESSOR})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

add_executable(example_lidar
  examples/example_lidar.cpp
)
target_link_libraries(example_lidar  libunitree_lidar_sdk.a )

add_executable(example_lidar_udp
  examples/example_lidar_udp.cpp
)
target_link_libraries(example_lidar_udp  libunitree_lidar_sdk.a )

add_executable(unilidar_publisher_udp
  examples/unilidar_publisher_udp.cpp
)
target_link_libraries(unilidar_publisher_udp  libunitree_lidar_sdk.a )

add_executable(unilidar_subscriber_udp
  examples/unilidar_subscriber_udp.cpp
)
target_link_libraries(unilidar_subscriber_udp  libunitree_lidar_sdk.a )


# Expose the include/ folder (ADDED)
# target_include_directories(unitree_lidar_sdk
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# ament_export_include_directories(include)
# ament_export_libraries(unitree_lidar_sdk)

# install(
#   TARGETS unitree_lidar_sdk
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
# )
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

