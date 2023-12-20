cmake_minimum_required(VERSION 3.13)
cmake_policy(VERSION 3.13)
project(rnnoise VERSION 1.1 LANGUAGES C)

option(RNNOISE_COMPILE_OPUS "COMPILE_OPUS" ON)
option(RNNOISE_USE_MALLOC "USE_MALLOC" ON)
option(BUILD_EXAMPLES "Build example projects" OFF)

include(CMakeDependentOption)
cmake_dependent_option(
  BUILD_SHARED_LIBS             # option variable
  "Build shared library"    # description
  ON                        # default value if exposed; user can override
  "NOT BUILD_STATIC_LIBS"   # condition to expose option
  OFF                       # value if not exposed; user can't override
)
if(RNNOISE_COMPILE_OPUS)
  add_definitions(-DCOMPILE_OPUS)
endif(RNNOISE_COMPILE_OPUS)

if(RNNOISE_USE_MALLOC)
  add_definitions(-DUSE_MALLOC)
endif(RNNOISE_USE_MALLOC)
# Ignore CRT warnings on MSVC
if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)

set(rnnoise-header-files
    include/rnnoise.h
)

# Get source files
file(GLOB SOURCES "src/*.c" "src/*.h" ${rnnoise-header-files})

# Build rnnoise
add_definitions(-DRNNOISE_BUILD)

# Compile the library
add_library(rnnoise ${SOURCES})

# Build DLL if needed
if(BUILD_SHARED_LIBS)
  if(WIN32)
    target_compile_definitions(rnnoise PRIVATE DLL_EXPORT)
  else(WIN32)
    include(CheckCCompilerFlag)
    check_c_compiler_flag(-fvisibility=hidden COMPILER_HAS_HIDDEN_VISIBILITY)
    if(COMPILER_HAS_HIDDEN_VISIBILITY)
      set_target_properties(rnnoise PROPERTIES C_VISIBILITY_PRESET hidden)
    endif(COMPILER_HAS_HIDDEN_VISIBILITY)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

# Include dirs
target_include_directories(rnnoise PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)


# ## Export ###
include(GNUInstallDirs)

install(
  TARGETS rnnoise
  EXPORT rnnoiseTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rnnoise)

install(FILES ${rnnoise-header-files}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rnnoise)

install(
  EXPORT rnnoiseTargets
  FILE rnnoiseTargets.cmake
  NAMESPACE rnnoise::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rnnoise)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rnnoiseConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/rnnoiseConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rnnoise)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/rnnoiseConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rnnoiseConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/rnnoiseConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rnnoise)

#use relative path, since CMAKE can't reconfigure on install with different prefix path 
set(PC_PREFIX "\${pcfiledir}/../..")  
configure_file(cmake/rnnoise.pc.in "${CMAKE_CURRENT_BINARY_DIR}/rnnoise.pc" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rnnoise.pc"
        CONFIGURATIONS Release RelWithDebInfo
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)


if(BUILD_EXAMPLES)
    subdirs ("examples")
endif(BUILD_EXAMPLES)


# make uninstall
if(NOT TARGET uninstall)
 configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
 add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
endif()