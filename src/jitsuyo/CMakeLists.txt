cmake_minimum_required(VERSION 3.5)

project(jitsuyo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gtest_vendor REQUIRED)
find_package(keisan REQUIRED)
find_package(OpenCV REQUIRED)

install(DIRECTORY include DESTINATION .)

add_library(jitsuyo SHARED
  src/image.cpp
  src/linux.cpp
  src/cli.cpp)

ament_target_dependencies(jitsuyo
  gtest_vendor
  keisan
  OpenCV)

target_include_directories(jitsuyo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS jitsuyo
  EXPORT export_jitsuyo
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(jitsuyo_tests
    "test/linux_test.cpp"
    "test/config_test.cpp"
    "test/cli_test.cpp")

  target_link_libraries(jitsuyo_tests jitsuyo)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(gtest_vendor OpenCV)
ament_export_include_directories("include")
ament_export_libraries(jitsuyo)

ament_package()
