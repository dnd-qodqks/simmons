cmake_minimum_required(VERSION 3.8)
project(qt6_gui)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt6 COMPONENTS Core Gui Widgets Network REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(result_msgs REQUIRED)

# Globbing Source Files
file(GLOB PRO_SOURCE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS  src/*.cpp)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB PRO_INCLUDE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS  include/*.h *.hpp)
file(GLOB PRO_RESOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)

# UI Wrapping
qt6_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt6_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt6_add_resources(QT_RESOUCE_HPP ${PRO_RESOURCE_DIR})

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}
	${OpenCV_INCLUDE_DIRS}
  ${result_msgs_INCLUDE_DIRS})

# include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}
    ${PRO_SOURCE_DIR}
    ${PRO_INCLUDE_DIR}
    ${PRO_UI_DIR}
    ${QT_MOC_HPP}
  )
  
# add_executable(qt6_gui src/mainwindow.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  Qt6
  cv_bridge
)

target_link_libraries(${PROJECT_NAME} Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Network ${OpenCV_LIBS} ${result_msgs_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

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
