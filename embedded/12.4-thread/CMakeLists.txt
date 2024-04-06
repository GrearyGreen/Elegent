cmake_minimum_required(VERSION 3.0.0)
project(THREAD VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_executable(mutex mutex.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
