cmake_minimum_required(VERSION 3.16)
project(TrackerOptimization CXX)

# ------------------------------
# C++ standard and defaults
# ------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified. Defaulting to Release.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# ------------------------------
# Plugin Options
# ------------------------------
option(ENABLE_PLUGIN_DATABASE    "Build database plugin"    ON)
option(ENABLE_PLUGIN_PROPAGATOR  "Build propagator plugin"  ON)
option(ENABLE_PLUGIN_SIMULATION  "Build simulation plugin"  ON)
option(ENABLE_PLUGIN_TRACK_MGMT  "Build track mgmt plugin"  ON)
option(ENABLE_PLUGIN_ASSOCIATOR  "Build associator plugin"  OFF)
option(ENABLE_PLUGIN_FILTER      "Build filter plugin"      OFF)
option(ENABLE_PLUGIN_GATING      "Build gating plugin"      OFF)
option(ENABLE_PLUGIN_MEASUREMENT "Build measurement plugin" OFF)

# ------------------------------
# Backend & Feature Options
# ------------------------------
option(USE_MKL          "Use Intel MKL backend"                    OFF)
option(USE_STD          "Use baseline std backend (no Eigen/MKL)"  OFF)
option(ENABLE_OPENMP    "Enable OpenMP (outer parallel loops)"     ON)
option(ENABLE_LTO       "Try Link-Time Optimization (IPO)"         ON)
option(EIGEN_DONT_PAR   "Disable Eigen internal parallelism"       ON)
option(ENABLE_CONFIG    "Build XML config loader tools (libxml2)"  ON)

# ------------------------------
# Include directories
# ------------------------------
# Top-level include for plugin framework
set(TRACKER_PUBLIC_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Legacy flat includes (for backward compatibility during transition)
list(APPEND TRACKER_PUBLIC_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/config
  ${CMAKE_CURRENT_SOURCE_DIR}/include/motion
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugin_framework
)

# Plugin-specific includes (new structure)
list(APPEND TRACKER_PUBLIC_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/database
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/propagator
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/propagator/la
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/simulation
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/track_mgmt
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/associator
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/filter
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/gating
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/measurement
)

# ------------------------------
# Plugin Framework (Core - Always Built)
# ------------------------------
add_library(plugin_framework STATIC
  src/plugin_framework/plugin_registry.cpp
  src/plugin_framework/plugin_factory.cpp
)
target_include_directories(plugin_framework PUBLIC ${TRACKER_PUBLIC_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(plugin_framework PRIVATE -O3 -fno-math-errno -fno-trapping-math)
  endif()
endif()

# ------------------------------
# Config loader library (libxml2)
# ------------------------------
if (ENABLE_CONFIG)
  message(STATUS "ENABLE_CONFIG=ON: building config_loader (libxml2) and config_check")

  add_library(config_loader STATIC
    src/config/config_loader.cpp
    src/config/xml_utils.cpp
    src/config/path_utils.cpp
  )
  target_include_directories(config_loader PUBLIC ${TRACKER_PUBLIC_INCLUDE_DIRS})
  target_include_directories(config_loader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  find_package(LibXml2 QUIET)
  if (LibXml2_FOUND)
    target_include_directories(config_loader PRIVATE ${LIBXML2_INCLUDE_DIR})
    target_link_libraries(config_loader PUBLIC ${LIBXML2_LIBRARIES})
    target_compile_definitions(config_loader PRIVATE HAVE_LIBXML2=1)
  else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
    target_include_directories(config_loader PRIVATE ${LIBXML2_INCLUDE_DIRS})
    target_link_libraries(config_loader PUBLIC ${LIBXML2_LIBRARIES})
    target_compile_definitions(config_loader PRIVATE HAVE_LIBXML2=1)
  endif()

  add_executable(config_check src/config_check.cpp)
  target_include_directories(config_check PRIVATE ${TRACKER_PUBLIC_INCLUDE_DIRS})
  target_link_libraries(config_check PRIVATE config_loader)

  if (ENABLE_LTO AND TARGET config_check)
    set_property(TARGET config_check PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
else()
  message(WARNING "ENABLE_CONFIG=OFF: config loader tools will not be built.")
endif()

# ------------------------------
# PLUGIN: Propagator (LA Backend)
# ------------------------------
if (ENABLE_PLUGIN_PROPAGATOR)
  message(STATUS "Building PLUGIN: Propagator")
  
  add_library(plugin_propagator STATIC)
  target_include_directories(plugin_propagator PUBLIC ${TRACKER_PUBLIC_INCLUDE_DIRS})
  target_link_libraries(plugin_propagator PUBLIC plugin_framework)

  if (CMAKE_BUILD_TYPE STREQUAL "Release")
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(plugin_propagator PRIVATE -O3 -fno-math-errno -fno-trapping-math)
    endif()
  endif()

  # Backend selection (Eigen / MKL / STD)
  if (USE_STD)
    message(STATUS "  - Using STD backend")
    target_sources(plugin_propagator PRIVATE
      src/plugins/propagator/la/la_std.cpp
      src/plugins/propagator/la/la_batch_rw_std.cpp
    )

  elseif (USE_MKL)
    message(STATUS "  - Using MKL backend")
    find_package(MKL REQUIRED)
    target_sources(plugin_propagator PRIVATE
      src/plugins/propagator/la/la_mkl.cpp
      src/plugins/propagator/la/la_batch_rw_mkl.cpp
    )
    target_link_libraries(plugin_propagator PUBLIC MKL::MKL)

  else()
    message(STATUS "  - Using Eigen backend (default)")
    find_package(Eigen3 REQUIRED)
    target_sources(plugin_propagator PRIVATE
      src/plugins/propagator/la/la_eigen.cpp
      src/plugins/propagator/la/la_batch_rw_eigen.cpp
    )
    target_link_libraries(plugin_propagator PUBLIC Eigen3::Eigen)
    if (EIGEN_DONT_PAR)
      target_compile_definitions(plugin_propagator PUBLIC EIGEN_DONT_PARALLELIZE)
    endif()
  endif()

  # OpenMP (outer-loop parallelism across tracks)
  if (ENABLE_OPENMP)
    find_package(OpenMP)
    if (OpenMP_CXX_FOUND)
      message(STATUS "  - Enabling OpenMP for outer batch parallelism")
      target_link_libraries(plugin_propagator PUBLIC OpenMP::OpenMP_CXX)
    else()
      message(STATUS "  - OpenMP not found; proceeding without it")
    endif()
  endif()

  # Link-Time Optimization (IPO/LTO)
  if (ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
    if (ipo_ok)
      set_property(TARGET plugin_propagator PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    else()
      message(STATUS "  - IPO/LTO not enabled: ${ipo_msg}")
    endif()
  endif()
endif()

# ------------------------------
# PLUGIN: Database (Spatial Index)
# ------------------------------
if (ENABLE_PLUGIN_DATABASE)
  message(STATUS "Building PLUGIN: Database")
  
  find_package(SQLite3 REQUIRED)

  add_library(plugin_database STATIC
    src/plugins/database/rtree_sqlite.cpp
    src/plugins/database/scan_volume.cpp
    src/plugins/database/uniform_grid_index.cpp
    src/plugins/database/index_update_manager.cpp
    src/plugins/database/sqlite_rtree_backend.cpp
    src/plugins/database/spatial_index_factory.cpp
  )
  target_include_directories(plugin_database PUBLIC ${TRACKER_PUBLIC_INCLUDE_DIRS})
  target_link_libraries(plugin_database PUBLIC plugin_framework)
  
  if (TARGET SQLite::SQLite3)
    target_link_libraries(plugin_database PUBLIC SQLite::SQLite3)
  else()
    target_link_libraries(plugin_database PUBLIC sqlite3)
  endif()

  if (ENABLE_LTO)
    set_property(TARGET plugin_database PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

# ------------------------------
# PLUGIN: Simulation (Truth Generation)
# ------------------------------
if (ENABLE_PLUGIN_SIMULATION)
  message(STATUS "Building PLUGIN: Simulation")
  
  add_library(plugin_simulation STATIC
    src/plugins/simulation/targets_generator.cpp
  )
  target_include_directories(plugin_simulation PUBLIC ${TRACKER_PUBLIC_INCLUDE_DIRS})
  target_link_libraries(plugin_simulation PUBLIC plugin_framework)
  
  if (ENABLE_CONFIG)
    target_link_libraries(plugin_simulation PUBLIC config_loader)
  endif()

  if (ENABLE_LTO)
    set_property(TARGET plugin_simulation PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

# ------------------------------
# PLUGIN: Track Management
# ------------------------------
if (ENABLE_PLUGIN_TRACK_MGMT)
  message(STATUS "Building PLUGIN: Track Management")
  
  add_library(plugin_track_mgmt STATIC
    src/plugins/track_mgmt/trk/seed_tracks_from_xml.cpp
  )
  target_include_directories(plugin_track_mgmt PUBLIC ${TRACKER_PUBLIC_INCLUDE_DIRS})
  target_link_libraries(plugin_track_mgmt PUBLIC plugin_framework)
  
  # Track management depends on simulation for truth seeding
  if (ENABLE_PLUGIN_SIMULATION)
    target_link_libraries(plugin_track_mgmt PUBLIC plugin_simulation)
  endif()

  if (ENABLE_LTO)
    set_property(TARGET plugin_track_mgmt PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

# ------------------------------
# PLUGIN: Associator (TODO - Empty for now)
# ------------------------------
if (ENABLE_PLUGIN_ASSOCIATOR)
  message(STATUS "Building PLUGIN: Associator (placeholder)")
  add_library(plugin_associator INTERFACE)
  target_include_directories(plugin_associator INTERFACE ${TRACKER_PUBLIC_INCLUDE_DIRS})
  # TODO: Add implementation files when ready
endif()

# ------------------------------
# PLUGIN: Filter (TODO - Empty for now)
# ------------------------------
if (ENABLE_PLUGIN_FILTER)
  message(STATUS "Building PLUGIN: Filter (placeholder)")
  add_library(plugin_filter INTERFACE)
  target_include_directories(plugin_filter INTERFACE ${TRACKER_PUBLIC_INCLUDE_DIRS})
  # TODO: Add implementation files when ready
endif()

# ------------------------------
# PLUGIN: Gating (TODO - Empty for now)
# ------------------------------
if (ENABLE_PLUGIN_GATING)
  message(STATUS "Building PLUGIN: Gating (placeholder)")
  add_library(plugin_gating INTERFACE)
  target_include_directories(plugin_gating INTERFACE ${TRACKER_PUBLIC_INCLUDE_DIRS})
  # TODO: Add implementation files when ready
endif()

# ------------------------------
# PLUGIN: Measurement (TODO - Empty for now)
# ------------------------------
if (ENABLE_PLUGIN_MEASUREMENT)
  message(STATUS "Building PLUGIN: Measurement (placeholder)")
  add_library(plugin_measurement INTERFACE)
  target_include_directories(plugin_measurement INTERFACE ${TRACKER_PUBLIC_INCLUDE_DIRS})
  # TODO: Add implementation files when ready
endif()

# ------------------------------
# Demo executable
# ------------------------------
add_executable(demo src/main.cpp)
target_include_directories(demo PRIVATE ${TRACKER_PUBLIC_INCLUDE_DIRS})

# Link plugin framework (always)
target_link_libraries(demo PRIVATE plugin_framework)

# Link enabled plugins
if (ENABLE_PLUGIN_PROPAGATOR)
  target_link_libraries(demo PRIVATE plugin_propagator)
endif()

if (ENABLE_PLUGIN_DATABASE)
  target_link_libraries(demo PRIVATE plugin_database)
endif()

if (ENABLE_PLUGIN_SIMULATION)
  target_link_libraries(demo PRIVATE plugin_simulation)
endif()

if (ENABLE_PLUGIN_TRACK_MGMT)
  target_link_libraries(demo PRIVATE plugin_track_mgmt)
endif()

if (ENABLE_PLUGIN_ASSOCIATOR)
  target_link_libraries(demo PRIVATE plugin_associator)
endif()

if (ENABLE_PLUGIN_FILTER)
  target_link_libraries(demo PRIVATE plugin_filter)
endif()

if (ENABLE_PLUGIN_GATING)
  target_link_libraries(demo PRIVATE plugin_gating)
endif()

if (ENABLE_PLUGIN_MEASUREMENT)
  target_link_libraries(demo PRIVATE plugin_measurement)
endif()

# Config loader (required for demo)
if (ENABLE_CONFIG)
  target_link_libraries(demo PRIVATE config_loader)
else()
  message(FATAL_ERROR "demo requires ENABLE_CONFIG=ON (config-driven main.cpp). Reconfigure with -DENABLE_CONFIG=ON.")
endif()

if (ENABLE_LTO AND TARGET demo)
  set_property(TARGET demo PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# ------------------------------
# Build Summary
# ------------------------------
message(STATUS "")
message(STATUS "========================================")
message(STATUS "Tracker Optimization Build Configuration")
message(STATUS "========================================")
message(STATUS "Build Type:          ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:        ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Plugin Framework:    ENABLED (always)")
message(STATUS "  - Database:        ${ENABLE_PLUGIN_DATABASE}")
message(STATUS "  - Propagator:      ${ENABLE_PLUGIN_PROPAGATOR}")
if (ENABLE_PLUGIN_PROPAGATOR)
  if (USE_MKL)
    message(STATUS "      Backend:     MKL")
  elseif (USE_STD)
    message(STATUS "      Backend:     STD")
  else()
    message(STATUS "      Backend:     Eigen")
  endif()
endif()
message(STATUS "  - Simulation:      ${ENABLE_PLUGIN_SIMULATION}")
message(STATUS "  - Track Mgmt:      ${ENABLE_PLUGIN_TRACK_MGMT}")
message(STATUS "  - Associator:      ${ENABLE_PLUGIN_ASSOCIATOR}")
message(STATUS "  - Filter:          ${ENABLE_PLUGIN_FILTER}")
message(STATUS "  - Gating:          ${ENABLE_PLUGIN_GATING}")
message(STATUS "  - Measurement:     ${ENABLE_PLUGIN_MEASUREMENT}")
message(STATUS "")
message(STATUS "Features:")
message(STATUS "  - Config Loader:   ${ENABLE_CONFIG}")
message(STATUS "  - OpenMP:          ${ENABLE_OPENMP}")
message(STATUS "  - LTO/IPO:         ${ENABLE_LTO}")
message(STATUS "========================================")
message(STATUS "")
