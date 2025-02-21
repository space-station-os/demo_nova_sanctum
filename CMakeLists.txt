cmake_minimum_required(VERSION 3.8)
project(demo_nova_sanctum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  
  "src/wrs_systems/msg/WasteCollection.msg"
  "src/wrs_systems/msg/ProcessWater.msg"
  "src/wrs_systems/msg/StorageStatus.msg"
  "src/wrs_systems/msg/Filteration.srv"

  "src/ars_systems/msg/AirData.msg"
  "src/ors_systems/msg/Sabatier.msg"
  "src/ors_systems/msg/Water.msg"
  "src/wrs_systems/srv/Water.srv"


)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs
)

  



include_directories(include)

add_executable(collector src/ars_systems/air_collector_tank.cpp)
ament_target_dependencies(collector std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(collector ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(desiccant src/ars_systems/desiccant_tank.cpp)
ament_target_dependencies(desiccant std_srvs rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(desiccant ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(adsorbent src/ars_systems/adsorbent_tank.cpp)
ament_target_dependencies(adsorbent std_srvs rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(adsorbent ${PROJECT_NAME} "rosidl_typesupport_cpp")



add_executable(sabatier src/ors_systems/src/sabatier.cpp)
ament_target_dependencies(sabatier std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(sabatier ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(water_pub src/ors_systems/src/water_pub.cpp)
ament_target_dependencies(water_pub std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(water_pub ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(deionization_bed src/ors_systems/src/deionization_chamber.cpp)
ament_target_dependencies(deionization_bed std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(deionization_bed ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(electrolysis src/ors_systems/src/electrolysis.cpp)
ament_target_dependencies(electrolysis std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(electrolysis ${PROJECT_NAME} "rosidl_typesupport_cpp")



add_executable(waste_collector src/wrs_systems/src/waste_collection.cpp)
ament_target_dependencies(waste_collector rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(waste_collector ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(process_water src/wrs_systems/src/process_water.cpp)
ament_target_dependencies(process_water rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(process_water ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(filteration src/wrs_systems/src/filteration.cpp)
ament_target_dependencies(filteration rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(filteration ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  collector
  desiccant
  adsorbent
  sabatier
  water_pub
  deionization_bed
  electrolysis
  
  waste_collector
  filteration 
  process_water
  
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
