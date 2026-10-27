# Copyright (c) 2026 Robotiq
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

if(NOT TARGET robotiq_tactile_sdk_vendor::RobotiqTactileSensor)
  set(_tactile_sdk_prefix "${robotiq_tactile_sdk_vendor_DIR}/../../../opt/robotiq_tactile_sdk_vendor")

  find_package(Threads REQUIRED)
  find_library(_tactile_sdk_serialport_library NAMES serialport libserialport REQUIRED)
  find_path(_tactile_sdk_serialport_include_dir libserialport.h REQUIRED)

  add_library(robotiq_tactile_sdk_vendor::RobotiqTactileSensor STATIC IMPORTED)
  set_target_properties(robotiq_tactile_sdk_vendor::RobotiqTactileSensor PROPERTIES
    IMPORTED_LOCATION "${_tactile_sdk_prefix}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}RobotiqTactileSensor${CMAKE_STATIC_LIBRARY_SUFFIX}"
    INTERFACE_INCLUDE_DIRECTORIES "${_tactile_sdk_prefix}/include/RobotiqTactileSensor;${_tactile_sdk_serialport_include_dir}"
    INTERFACE_LINK_LIBRARIES "${_tactile_sdk_serialport_library};Threads::Threads"
  )
endif()
