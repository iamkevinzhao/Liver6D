cmake_minimum_required(VERSION 2.8)
project(Liver6D CXX)

find_package(PCL 1.7 REQUIRED)
find_package(Qt5Widgets)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_executable(
  liver6d
    main.cpp
#    main_window.cpp
#    main_window.h
#    main_window.ui
    pclviewer.cpp
    pclviewer.h
    pclviewer.ui
    frame.cpp
    frame.h
)

target_link_libraries(
  liver6d
    ${PCL_LIBRARIES}
    Qt5::Widgets
)
