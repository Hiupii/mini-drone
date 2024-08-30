# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/hieu/esp/esp-idf/components/bootloader/subproject"
  "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader"
  "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader-prefix"
  "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader-prefix/tmp"
  "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader-prefix/src/bootloader-stamp"
  "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader-prefix/src"
  "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/hieu/esp/datn/firmware/mini_drone/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
