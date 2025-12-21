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
option(ENABLE_CONFIG    "Build XML config loader tools (libxml2)" ON)

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
# NOTE: We now also compile the backend-selected fast RW batch update:
#   P <- P + Q*dt   (full NxN; currently N=9 for CA9 integration)
if (USE_STD)
  message(STATUS "Building with baseline STD backend (no Eigen/MKL)")
  target_sources(simple_la PRIVATE
    src/la_std.cpp
    src/la_batch_rw_std.cpp
  )

elseif (USE_MKL)
  message(STATUS "Building with Intel MKL backend")
  find_package(MKL REQUIRED)
  target_sources(simple_la PRIVATE
    src/la_mkl.cpp
    src/la_batch_rw_mkl.cpp
  )
  target_link_libraries(simple_la PUBLIC MKL::MKL)

else()
  message(STATUS "Building with Eigen backend (default)")
  find_package(Eigen3 REQUIRED)
  target_sources(simple_la PRIVATE
    src/la_eigen.cpp
    src/la_batch_rw_eigen.cpp
  )
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
# Config loader library (libxml2)
# ------------------------------
# NOTE: demo now loads + validates XML, so ENABLE_CONFIG must be ON for this integration step.
if (ENABLE_CONFIG)
  message(STATUS "ENABLE_CONFIG=ON: building config_loader (libxml2) and config_check")

  add_library(config_loader STATIC
    src/config/config_loader.cpp
    src/config/xml_utils.cpp
    src/config/path_utils.cpp
  )
  target_include_directories(config_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_include_directories(config_loader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  #target_include_directories(config_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # Prefer CMake's FindLibXml2; fallback to pkg-config if needed.
  find_package(LibXml2 QUIET)
  if (LibXml2_FOUND)
    target_include_directories(config_loader PRIVATE ${LIBXML2_INCLUDE_DIR})
    # IMPORTANT: libxml2 must be PUBLIC so consumers of this static lib link correctly.
    target_link_libraries(config_loader PUBLIC ${LIBXML2_LIBRARIES})
    target_compile_definitions(config_loader PRIVATE HAVE_LIBXML2=1)
  else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
    target_include_directories(config_loader PRIVATE ${LIBXML2_INCLUDE_DIRS})
    # IMPORTANT: libxml2 must be PUBLIC so consumers of this static lib link correctly.
    target_link_libraries(config_loader PUBLIC ${LIBXML2_LIBRARIES})
    target_compile_definitions(config_loader PRIVATE HAVE_LIBXML2=1)
  endif()

  add_executable(config_check src/config_check.cpp)
  target_link_libraries(config_check PRIVATE config_loader)

  if (ENABLE_LTO AND TARGET config_check)
    set_property(TARGET config_check PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
else()
  message(WARNING "ENABLE_CONFIG=OFF: config loader tools will not be built.")
endif()

# ------------------------------
# Demo executable (config-driven)
# ------------------------------
add_executable(demo
  src/main.cpp
  src/targets/targets_generator.cpp
)
target_link_libraries(demo PRIVATE simple_la)

# demo now depends on the XML config loader
if (ENABLE_CONFIG)
  target_link_libraries(demo PRIVATE config_loader)
else()
  message(FATAL_ERROR "demo requires ENABLE_CONFIG=ON (config-driven main.cpp). Reconfigure with -DENABLE_CONFIG=ON.")
endif()

if (ENABLE_LTO AND TARGET demo)
  set_property(TARGET demo PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
