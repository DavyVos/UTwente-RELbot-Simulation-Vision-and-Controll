cmake_minimum_required(VERSION 3.8)
project(assignment1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(relbot_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPosition.msg"
  "msg/ObjectBrightness.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(brightness src/CheckBrightness.cpp)
ament_target_dependencies(brightness
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(brightness ${OpenCV_LIBS} ${cpp_typesupport_target})

add_executable(findobject src/FindObject.cpp)
ament_target_dependencies(findobject
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(findobject ${OpenCV_LIBS} ${cpp_typesupport_target})
rosidl_target_interfaces(findobject
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sequencecontroller src/SequenceController.cpp)
ament_target_dependencies(sequencecontroller
  rclcpp
  std_msgs
  sensor_msgs
  relbot_msgs
  cv_bridge
  image_transport
)
target_link_libraries(sequencecontroller ${OpenCV_LIBS} ${cpp_typesupport_target})
rosidl_target_interfaces(sequencecontroller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  brightness
  findobject
  sequencecontroller
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
