cmake_minimum_required(VERSION 3.8)
project(avoid_plan_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find message deps
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# find the message files
file(GLOB srv_sources RELATIVE ${PROJECT_SOURCE_DIR} srv/*.srv) 

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_sources}
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
