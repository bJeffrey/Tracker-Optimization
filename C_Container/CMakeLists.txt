cmake_minimum_required(VERSION 3.16)
project(simple_la CXX)

# ------------------------------
# C++ standard and defaults
# ------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to Release if nothing specified (important for perf)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified. Defaulting to Release.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# ------------------------------
# Options
# ------------------------------
option(USE_MKL          "Use Intel MKL backend"               OFF)
option(USE_STD          "Use baseline std backend (no Eigen/MKL)" OFF)
option(ENABLE_OPENMP    "Enable OpenMP (outer parallel loops)" ON)
option(ENABLE_LTO       "Try Link-Time Optimization (IPO)"     ON)
option(EIGEN_DONT_PAR   "Disable Eigen internal parallelism"   ON)  # keeps Eigen single-threaded; we parallel outside

# ------------------------------
# Library target
# ------------------------------
add_library(simple_la STATIC)
target_include_directories(simple_la PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Mildly aggressive but portable release flags (GCC/Clang)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(simple_la PRIVATE -O3 -fno-math-errno -fno-trapping-math)
    # Uncomment to tighten FP reproducibility (may reduce FMA fusion/perf)
    # target_compile_options(simple_la PRIVATE -ffp-contract=off)
  endif()
endif()

# ------------------------------
# Backend selection
# ------------------------------
if (USE_STD)
  message(STATUS "Building with baseline STD backend (no Eigen/MKL)")
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
  if (EIGEN_DONT_PAR)
    target_compile_definitions(simple_la PUBLIC EIGEN_DONT_PARALLELIZE)
  endif()
endif()

# ------------------------------
# OpenMP (outer-loop parallelism across tracks)
# ------------------------------
if (ENABLE_OPENMP)
  find_package(OpenMP)
  if (OpenMP_CXX_FOUND)
    message(STATUS "Enabling OpenMP for outer batch parallelism")
    target_link_libraries(simple_la PUBLIC OpenMP::OpenMP_CXX)
  else()
    message(STATUS "OpenMP not found; proceeding without it")
  endif()
endif()

# ------------------------------
# Link-Time Optimization (IPO/LTO) when available
# ------------------------------
if (ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if (ipo_ok)
    set_property(TARGET simple_la PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(STATUS "IPO/LTO not enabled: ${ipo_msg}")
  endif()
endif()

# ------------------------------
# Demo executable
# ------------------------------
add_executable(demo src/main.cpp)
target_link_libraries(demo PRIVATE simple_la)
if (ENABLE_LTO AND TARGET demo)
  set_property(TARGET demo PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
