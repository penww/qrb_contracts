cmake_minimum_required(VERSION 3.16)
project(qrb_contracts_proto VERSION 1.0.0 LANGUAGES CXX)

# Protocol Buffer schemas of qrb_contracts.
# Generated headers keep the proto package layout, e.g.
#   #include <qrb/contract/geometry/v1/pose.pb.h>
#
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Protobuf REQUIRED)

set(PROTO_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(GLOB_RECURSE PROTO_FILES CONFIGURE_DEPENDS ${PROTO_ROOT}/*.proto)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

# Run protoc by hand so the output keeps the package layout (qrb/contract/...) relative to
# the include root, protobuf_generate() would prefix it with the source directory name.
set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(proto_file ${PROTO_FILES})
  file(RELATIVE_PATH proto_rel ${PROTO_ROOT} ${proto_file})
  string(REGEX REPLACE "\\.proto$" "" proto_base ${proto_rel})
  set(proto_src ${PROTO_GEN_DIR}/${proto_base}.pb.cc)
  set(proto_hdr ${PROTO_GEN_DIR}/${proto_base}.pb.h)
  add_custom_command(
    OUTPUT ${proto_src} ${proto_hdr}
    COMMAND protobuf::protoc
    ARGS --cpp_out=${PROTO_GEN_DIR} -I ${PROTO_ROOT} ${proto_file}
    DEPENDS ${proto_file} protobuf::protoc
    COMMENT "Generating C++ code for ${proto_rel}"
    VERBATIM)
  list(APPEND PROTO_SRCS ${proto_src})
  list(APPEND PROTO_HDRS ${proto_hdr})
endforeach()

add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
# BEFORE: the freshly generated headers must win over any stale copy of the same headers that
# is already installed in a directory added by a parent project (e.g. a previous install
# into the same prefix), otherwise a .pb.cc can be compiled against an older .pb.h.
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC
  $<BUILD_INTERFACE:${PROTO_GEN_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME} PUBLIC protobuf::libprotobuf)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# Generated headers (*.pb.h) and the schemas themselves.
install(DIRECTORY ${PROTO_GEN_DIR}/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.pb.h")
install(DIRECTORY ${PROTO_ROOT}/ DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/proto
  FILES_MATCHING PATTERN "*.proto")

install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.in
"@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(Protobuf)
include(\"\${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake\")
check_required_components(@PROJECT_NAME@)
")
configure_package_config_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
