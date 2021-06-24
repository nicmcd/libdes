cmake_minimum_required(VERSION 3.10)
project(libdes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# numactl
find_library(NUMACTL_A NAMES numactl.a PATHS ${CMAKE_INSTALL_RPATH})
if(NUMACTL_A MATCHES "NOTFOUND")
  message(FATAL_ERROR "numactl.a not found")
endif()
string(REPLACE lib/numactl.a include/ NUMACTL_INC ${NUMACTL_A})
if(NOT EXISTS ${NUMACTL_INC})
  message(FATAL_ERROR "numactl include dir doesn't exist")
endif()

# zlib
find_library(ZLIB_A NAMES zlib.a PATHS ${CMAKE_INSTALL_RPATH})
if(ZLIB_A MATCHES "NOTFOUND")
  message(FATAL_ERROR "zlib.a not found")
endif()
string(REPLACE lib/zlib.a include/ ZLIB_INC ${ZLIB_A})
if(NOT EXISTS ${ZLIB_INC})
  message(FATAL_ERROR "zlib include dir doesn't exist")
endif()

# libprim
find_library(LIBPRIM_A NAMES libprim.a PATHS ${CMAKE_INSTALL_RPATH})
if(LIBPRIM_A MATCHES "NOTFOUND")
  message(FATAL_ERROR "libprim.a not found")
endif()
string(REPLACE lib/libprim.a include/ LIBPRIM_INC ${LIBPRIM_A})
if(NOT EXISTS ${LIBPRIM_INC})
  message(FATAL_ERROR "libprim include dir doesn't exist")
endif()

# librnd
find_library(LIBRND_A NAMES librnd.a PATHS ${CMAKE_INSTALL_RPATH})
if(LIBRND_A MATCHES "NOTFOUND")
  message(FATAL_ERROR "librnd.a not found")
endif()
string(REPLACE lib/librnd.a include/ LIBRND_INC ${LIBRND_A})
if(NOT EXISTS ${LIBRND_INC})
  message(FATAL_ERROR "librnd include dir doesn't exist")
endif()

add_library(
  des
  STATIC
  ${PROJECT_SOURCE_DIR}/src/des/Logger.cc
  ${PROJECT_SOURCE_DIR}/src/des/Component.cc
  ${PROJECT_SOURCE_DIR}/src/des/Observer.cc
  ${PROJECT_SOURCE_DIR}/src/des/Event.cc
  ${PROJECT_SOURCE_DIR}/src/des/Time.cc
  ${PROJECT_SOURCE_DIR}/src/des/ActiveComponent.cc
  ${PROJECT_SOURCE_DIR}/src/des/MpScQueue.cc
  ${PROJECT_SOURCE_DIR}/src/des/Simulator.cc
  ${PROJECT_SOURCE_DIR}/src/des/Mapper.cc
  ${PROJECT_SOURCE_DIR}/src/des/util/RoundRobinMapper.cc
  ${PROJECT_SOURCE_DIR}/src/des/util/BasicObserver.cc
  ${PROJECT_SOURCE_DIR}/src/des/util/Scheduler.cc
  ${PROJECT_SOURCE_DIR}/src/des/util/SpinLock.cc
  ${PROJECT_SOURCE_DIR}/src/des/util/RandomMapper.cc
  ${PROJECT_SOURCE_DIR}/src/des/Mapper.h
  ${PROJECT_SOURCE_DIR}/src/des/timeconf.h
  ${PROJECT_SOURCE_DIR}/src/des/Time.h
  ${PROJECT_SOURCE_DIR}/src/des/Logger.h
  ${PROJECT_SOURCE_DIR}/src/des/Component.h
  ${PROJECT_SOURCE_DIR}/src/des/Event.h
  ${PROJECT_SOURCE_DIR}/src/des/Simulator.h
  ${PROJECT_SOURCE_DIR}/src/des/ActiveComponent.h
  ${PROJECT_SOURCE_DIR}/src/des/Observer.h
  ${PROJECT_SOURCE_DIR}/src/des/des.h
  ${PROJECT_SOURCE_DIR}/src/des/MpScQueue.h
  ${PROJECT_SOURCE_DIR}/src/des/cacheline_util.h
  ${PROJECT_SOURCE_DIR}/src/des/util/SpinLock.h
  ${PROJECT_SOURCE_DIR}/src/des/util/RandomMapper.h
  ${PROJECT_SOURCE_DIR}/src/des/util/BasicObserver.h
  ${PROJECT_SOURCE_DIR}/src/des/util/Scheduler.h
  ${PROJECT_SOURCE_DIR}/src/des/util/RoundRobinMapper.h
  )

set_target_properties(
  des
  PROPERTIES
  LINKER_LANGUAGE
  CXX
  )

target_include_directories(
  des
  PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${NUMACTL_INC}
  ${ZLIB_INC}
  ${LIBPRIM_INC}
  ${LIBRND_INC}
  )

target_link_libraries(
  des
  ${NUMACTL_A}
  ${ZLIB_A}
  ${LIBPRIM_A}
  ${LIBRND_A}
  )

include(GNUInstallDirs)

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/Mapper.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/timeconf.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/Time.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/Logger.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/Component.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/Event.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/Simulator.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/ActiveComponent.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/Observer.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/des.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/MpScQueue.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/cacheline_util.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/SpinLock.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/RandomMapper.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/BasicObserver.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/Scheduler.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/RoundRobinMapper.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/
  )

install(
  TARGETS
  des
  )

