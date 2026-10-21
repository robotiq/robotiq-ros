cmake_minimum_required(VERSION 3.16)
project(robotiq_tactile_sdk_vendor NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# The pinned release, in a file of its own so tactile_helpers.sh reads it too.
file(STRINGS sdk_version _sdk_version LIMIT_COUNT 1)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS sdk_version)

set(ROBOTIQ_TACTILE_SDK_SOURCE_DIR "" CACHE PATH
  "Local checkout of github.com/robotiq/tactile_sensors to build instead of the pinned release")

# One target per source: an external project's build tree stays tied to the
# source it was first configured from, so switching would fail to configure.
if(ROBOTIQ_TACTILE_SDK_SOURCE_DIR)
  set(_sdk_target tactile_sdk_local)
  # VCS_TYPE path takes no SOURCE_SUBDIR.
  set(_sdk_source VCS_TYPE path VCS_URL "${ROBOTIQ_TACTILE_SDK_SOURCE_DIR}/sdk_cpp")
else()
  set(_sdk_target tactile_sdk)
  set(_sdk_source
    VCS_URL https://github.com/robotiq/tactile_sensors.git
    VCS_VERSION ${_sdk_version}
    SOURCE_SUBDIR sdk_cpp)
endif()

ament_vendor(${_sdk_target} ${_sdk_source})

# The SDK installs its library and headers but no CMake package, so the
# imported target comes from this package's config.
ament_package(CONFIG_EXTRAS cmake/robotiq_tactile_sdk_vendor-extras.cmake)
