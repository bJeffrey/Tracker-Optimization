cmake_minimum_required(VERSION 3.16)
project(simple_la CXX)

# ------------------------------
# C++ standard and defaults
# ------------------------------
set(CMAKE_CXX_STANDARD 98)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# If the user didn't specify a build type, default to Release (important for perf)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified. Defaulting to Release.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# ------------------------------
# Options
# ------------------------------
option(USE_MKL "Use Intel MKL backend" OFF)
option(USE_STD "Use baseline C++98 backend (no Eigen/MKL)" OFF)
option(ENABLE_OPENMP "Enable OpenMP (used mainly for Eigen's threading)" ON)

# ------------------------------
# Library target
# ------------------------------
add_library(simple_la STATIC)
target_include_directories(simple_la PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ------------------------------
# Backend selection
# ------------------------------
if (USE_STD)
  message(STATUS "Building with baseline C++98 backend (no Eigen/MKL)")
  target_sources(simple_la PRIVATE src/la_std.cpp)

elseif (USE_MKL)
  message(STATUS "Building with Intel MKL backend")
  find_package(MKL REQUIRED)
  target_sources(simple_la PRIVATE src/la_mkl.cpp)
  target_link_libraries(simple_la PUBLIC MKL::MKL)

else()
  message(STATUS "Building with Eigen backend (default)")
  find_package(Eigen3 REQUIRED)
  target_sources(simple_la PRIVATE src/la_eigen.cpp)
  target_link_libraries(simple_la PUBLIC Eigen3::Eigen)

  # Optional OpenMP to let Eigen use multi-threading via OMP (for larger problems)
  if (ENABLE_OPENMP)
    find_package(OpenMP)
    if (OpenMP_CXX_FOUND)
      message(STATUS "Enabling OpenMP for Eigen backend")
      target_link_libraries(simple_la PUBLIC OpenMP::OpenMP_CXX)
      target_compile_definitions(simple_la PUBLIC EIGEN_USE_THREADS)
      # You can control threads at runtime with OMP_NUM_THREADS
    else()
      message(STATUS "OpenMP not found; proceeding without it")
    endif()
  endif()
endif()

# ------------------------------
# Demo executable
# ------------------------------
add_executable(demo src/main.cpp)
target_link_libraries(demo PRIVATE simple_la)
