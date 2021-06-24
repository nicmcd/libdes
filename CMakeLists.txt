cmake_minimum_required(VERSION 3.20)
project(libdes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(FindPkgConfig)

# numactl
pkg_check_modules(numactl REQUIRED IMPORTED_TARGET numa)
  get_target_property(
  NUMACTL_INC
  PkgConfig::numactl
  INTERFACE_INCLUDE_DIRECTORIES
)

# zlib
pkg_check_modules(zlib REQUIRED IMPORTED_TARGET zlib)
  get_target_property(
  ZLIB_INC
  PkgConfig::zlib
  INTERFACE_INCLUDE_DIRECTORIES
)

# libprim
pkg_check_modules(libprim REQUIRED IMPORTED_TARGET libprim)
  get_target_property(
  LIBPRIM_INC
  PkgConfig::libprim
  INTERFACE_INCLUDE_DIRECTORIES
)

# librnd
pkg_check_modules(librnd REQUIRED IMPORTED_TARGET librnd)
  get_target_property(
  LIBRND_INC
  PkgConfig::librnd
  INTERFACE_INCLUDE_DIRECTORIES
)

add_library(
  des
  SHARED
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
  POSITION_INDEPENDENT_CODE
  ON
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
  PkgConfig::numactl
  PkgConfig::zlib
  PkgConfig::libprim
  PkgConfig::librnd
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
  ${CMAKE_INSTALL_INCLUDEDIR}/des/util/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/RandomMapper.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/util/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/BasicObserver.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/util/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/Scheduler.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/util/
  )

install(
  FILES
  ${PROJECT_SOURCE_DIR}/src/des/util/RoundRobinMapper.h
  DESTINATION
  ${CMAKE_INSTALL_INCLUDEDIR}/des/util/
  )

install(
  TARGETS
  des
  )

configure_file(
  ${PROJECT_SOURCE_DIR}/libdes.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/libdes.pc
  @ONLY
  )
install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/libdes.pc
  DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/pkgconfig/
  )
