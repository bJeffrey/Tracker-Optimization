cmake_minimum_required(VERSION 3.16)
project(simple_la CXX)

# C++98 (portable)
set(CMAKE_CXX_STANDARD 98)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

option(USE_MKL "Use Intel MKL backend" OFF)

add_library(simple_la STATIC)
target_include_directories(simple_la PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (USE_MKL)
  find_package(MKL REQUIRED)                 # requires MKL installed & discoverable
  target_sources(simple_la PRIVATE src/la_mkl.cpp)
  target_link_libraries(simple_la PUBLIC MKL::MKL)
else()
  find_package(Eigen3 REQUIRED)              # header-only; package or local path both ok
  target_sources(simple_la PRIVATE src/la_eigen.cpp)
  target_link_libraries(simple_la PUBLIC Eigen3::Eigen)
endif()

add_executable(demo src/main.cpp)
target_link_libraries(demo PRIVATE simple_la)
