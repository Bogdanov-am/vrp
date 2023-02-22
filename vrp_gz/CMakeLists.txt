cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(vrp_gz)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS graphics)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-fuel_tools8 REQUIRED)
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-msgs9 REQUIRED)
find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
find_package(gz-sensors7 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})
find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})
find_package(sdformat13 REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================

# Waves
add_library(Waves SHARED
  src/Wavefield.cc
)
target_link_libraries(Waves PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Eigen3::Eigen
)
install(
  TARGETS Waves
  DESTINATION lib)

# Buoyancy
add_library(PolyhedraBuoyancyDrag SHARED
  src/PolyhedraBuoyancyDrag.cc
  src/PolyhedronVolume.cc
  src/ShapeVolume.cc
)
target_link_libraries(PolyhedraBuoyancyDrag PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Waves
)
install(
  TARGETS PolyhedraBuoyancyDrag
  DESTINATION lib)


# Plugins
list(APPEND VRP_GZ_PLUGINS
  SimpleHydrodynamics
  Surface
  WaveVisual
)

foreach(PLUGIN ${VRP_GZ_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    gz-sim${GZ_SIM_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    Waves
    Eigen3::Eigen
  )
endforeach()

install(
  TARGETS ${VRP_GZ_PLUGINS}
  DESTINATION lib)

ament_python_install_package(
  vrp_gz
  PACKAGE_DIR src/vrp_gz
)

#============================================================================
# Resources
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
