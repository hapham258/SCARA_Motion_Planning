cmake_minimum_required(VERSION 2.8.3)
project(gui)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 REQUIRED
  COMPONENTS
    Core
    Gui
    Widgets
    PrintSupport
)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

catkin_package()

add_executable(gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(gui ${catkin_LIBRARIES})
target_link_libraries(gui
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::PrintSupport
)
