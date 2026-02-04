# cmake_minimum_required(VERSION 3.8)
# project(avatar_follower)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # 1. 라이브러리 찾기 (find_package)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(urdf REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(fcl REQUIRED)
# find_package(kdl_parser REQUIRED)
# find_package(orocos_kdl REQUIRED)
# # KDL 관련 라이브러리도 있다면 찾기
# # find_package(orocos_kdl_vendor REQUIRED) 등...
# include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(/usr/include/fcl)

# # 2. 실행 파일 만들기 (add_executable)
# add_executable(collision_node collision/collision_node.cpp) # 본인 파일명

# # 3. 라이브러리 연결 (ament_target_dependencies)
# # 이 부분이 없으면 헤더 파일을 못 찾습니다!
# ament_target_dependencies(collision_node
#   rclcpp
#   sensor_msgs
#   std_msgs
#   visualization_msgs
#   urdf
#   kdl_parser
#   orocos_kdl
# )

# target_link_libraries(collision_node
#   fcl   # FCL 라이브러리 이름 (보통 fcl 또는 libfcl)
# )

# install(TARGETS
#   collision_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()