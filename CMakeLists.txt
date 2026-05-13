cmake_minimum_required(VERSION 3.26)
project(AutoDrive VERSION 1.0.0 LANGUAGES C CXX)

# From top project
add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)
set(THREADS_PREFER_PTHREAD_FLAG ON)
# global settings
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

set(LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/WinLibs)
set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
SET(BINARY_OUT_PATH ${CMAKE_CURRENT_LIST_DIR}/Release)

option(CLOUD_PLATFORM "option for cloud build" OFF)

IF (CLOUD_PLATFORM)
    add_definitions(-DCLOUD_PLATFORM)
ENDIF(CLOUD_PLATFORM)

function(SetDefaultTargetProperties target)
    set_target_properties(${target} PROPERTIES
            CXX_STANDARD 14
    )
    set_target_properties(${target} PROPERTIES
            # ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../Output/lib
            # LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../Output/lib
            # RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../Output/bin
            RUNTIME_OUTPUT_DIRECTORY ${BINARY_OUT_PATH}
    )
endfunction()

# From subdirectory, win32 only
file(GLOB_RECURSE srcs Impl/*.h Impl/*.hh Impl/*i Impl/*.c Impl/*.cc Impl/*.cpp)
file(GLOB_RECURSE headers Impl/*.h)
add_executable(AutoDrive ""
        Impl/bezier.h
        Impl/prediction.h
)
target_sources(AutoDrive PRIVATE ${srcs} ${headers})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})
target_include_directories(AutoDrive SYSTEM PRIVATE "${INCLUDE_DIR}")

IF (WIN32)
    add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()
add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

SetDefaultTargetProperties(AutoDrive)

target_link_libraries(AutoDrive ${LIB_DIR}/SimOneAPI.lib ${LIB_DIR}/HDMapModule.lib ${LIB_DIR}/ProtobufModule.lib ${LIB_DIR}/SSD.lib)

add_custom_command(TARGET AutoDrive
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SimOneAPI.dll ${BINARY_OUT_PATH}/SimOneAPI.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SSD.dll ${BINARY_OUT_PATH}/SSD.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/HDMapModule.dll ${BINARY_OUT_PATH}/HDMapModule.dll
)
