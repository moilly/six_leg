cmake_minimum_required(VERSION 3.5)

project(AlgoTest LANGUAGES CXX)

enable_testing()

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Test)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Test)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(AlgoTest tst_algotest.cpp
    hexapod_kinematics.cpp hexapod_kinematics.h)
add_test(NAME AlgoTest COMMAND AlgoTest)

include_directories(third_party/eigenmaster/)

target_link_libraries(AlgoTest PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Test)

