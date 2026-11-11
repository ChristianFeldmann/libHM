# executable
set( SHARDE_LIB_NAME HMDecoder )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# get additional libs for gcc on Ubuntu systems
if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    if( USE_ADDRESS_SANITIZER )
      set( ADDITIONAL_LIBS asan )
    endif()
  endif()
endif()

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )
endif()

# add executable
add_library( ${SHARDE_LIB_NAME} SHARED ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} )
include_directories(${CMAKE_CURRENT_BINARY_DIR})

if( HIGH_BITDEPTH )
  target_compile_definitions( ${SHARDE_LIB_NAME} PUBLIC RExt__HIGH_BIT_DEPTH_SUPPORT=1 )
endif()

if( SET_ENABLE_TRACING )
  if( ENABLE_TRACING )
    target_compile_definitions( ${SHARDE_LIB_NAME} PUBLIC ENABLE_TRACING=1 )
  else()
    target_compile_definitions( ${SHARDE_LIB_NAME} PUBLIC ENABLE_TRACING=0 )
  endif()
endif()

if( CMAKE_COMPILER_IS_GNUCC AND BUILD_STATIC )
  set( ADDITIONAL_LIBS ${ADDITIONAL_LIBS} -static -static-libgcc -static-libstdc++ )
  target_compile_definitions( ${SHARDE_LIB_NAME} PUBLIC ENABLE_WPP_STATIC_LINK=1 )
endif()

target_link_libraries( ${SHARDE_LIB_NAME} TLibCommon TLibDecoder Utilities Threads::Threads ${ADDITIONAL_LIBS} )

if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  add_custom_command( TARGET ${SHARDE_LIB_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
                                                          $<$<CONFIG:Debug>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/libHMDecoderd.so>
                                                          $<$<CONFIG:Release>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}/libHMDecoder.so>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO}/libHMDecoderp.so>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL}/libHMDecoderm.so>
                                                          $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/bin/libHMDecoderd.so>
                                                          $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/bin/libHMDecoder.so>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_SOURCE_DIR}/bin/libHMDecoderp.so>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_SOURCE_DIR}/bin/libHMDecoderm.so> )
endif()

# example: place header files in different folders
source_group( "Natvis Files" FILES ${NATVIS_FILES} )

# set the folder where to place the projects
set_target_properties( ${SHARDE_LIB_NAME} PROPERTIES FOLDER app LINKER_LANGUAGE CXX )
