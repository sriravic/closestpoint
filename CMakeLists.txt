cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(closestpoint)

set(INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install)

add_subdirectory(src)