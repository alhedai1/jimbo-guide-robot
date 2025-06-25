cmake_minimum_required(VERSION 3.8)
project(jimbo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 메시지 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorRPM.msg"
  DEPENDENCIES builtin_interfaces
)

# 런타임 의존성 내보내기
ament_export_dependencies(rosidl_default_runtime)

# 코드 린트 테스트 (선택사항)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 반드시 마지막에 위치
ament_package()