# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/dvt/esp/esp-idf/components/bootloader/subproject"
  "/home/dvt/check_tof/examples/build/bootloader"
  "/home/dvt/check_tof/examples/build/bootloader-prefix"
  "/home/dvt/check_tof/examples/build/bootloader-prefix/tmp"
  "/home/dvt/check_tof/examples/build/bootloader-prefix/src/bootloader-stamp"
  "/home/dvt/check_tof/examples/build/bootloader-prefix/src"
  "/home/dvt/check_tof/examples/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/dvt/check_tof/examples/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/dvt/check_tof/examples/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
