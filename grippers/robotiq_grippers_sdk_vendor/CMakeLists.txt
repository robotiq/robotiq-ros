cmake_minimum_required(VERSION 3.16)
project(robotiq_grippers_sdk_vendor NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# The pinned release, in a file of its own like the tactile SDK's.
file(STRINGS sdk_version _sdk_version LIMIT_COUNT 1)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS sdk_version)

set(ROBOTIQ_GRIPPERS_SDK_SOURCE_DIR "" CACHE PATH
  "Local checkout of github.com/robotiq/grippers to build instead of the pinned release")

# One target per source: an external project's build tree stays tied to the
# source it was first configured from, so switching would fail to configure.
if(ROBOTIQ_GRIPPERS_SDK_SOURCE_DIR)
  set(_sdk_target grippers_sdk_local)
  # VCS_TYPE path takes no SOURCE_SUBDIR.
  set(_sdk_source VCS_TYPE path VCS_URL "${ROBOTIQ_GRIPPERS_SDK_SOURCE_DIR}/sdk_cpp")
else()
  set(_sdk_target grippers_sdk)
  set(_sdk_source
    VCS_URL https://github.com/robotiq/grippers.git
    VCS_VERSION ${_sdk_version}
    SOURCE_SUBDIR sdk_cpp)
endif()

# Here the SDK is the top-level project, which defaults its examples, tests and
# warnings-as-errors on. A newer compiler on the build farm must not fail the
# release on a warning ours does not raise.
#
# Static because robotiq_driver folds every SDK object into its plugin library,
# so nothing loads the SDK at run time.
ament_vendor(${_sdk_target}
  ${_sdk_source}
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=OFF
    -DGRIPPERS_BUILD_EXAMPLES=OFF
    -DGRIPPERS_BUILD_TESTS=OFF
    -DGRIPPERS_WARNINGS_AS_ERRORS=OFF
)

ament_package()
