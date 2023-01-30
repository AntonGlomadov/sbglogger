cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Release CACHE STRING "build type")

project(sbgECom)

#
# Project configuration
#
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#
# sbgECom
#
add_library(${PROJECT_NAME} STATIC)


file(GLOB_RECURSE COMMON_SRC ${PROJECT_SOURCE_DIR}/common/*.c)
file(GLOB_RECURSE ECOM_SRC ${PROJECT_SOURCE_DIR}/src/*.c)

# Exclude platform specific files
if (NOT MSVC)
    list(REMOVE_ITEM COMMON_SRC ${PROJECT_SOURCE_DIR}/common/interfaces/sbgInterfaceSerialWin.c)
else ()
    list(REMOVE_ITEM COMMON_SRC ${PROJECT_SOURCE_DIR}/common/interfaces/sbgInterfaceSerialUnix.c)
endif()

target_sources(${PROJECT_NAME} PRIVATE ${COMMON_SRC} ${ECOM_SRC})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/common
        INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common>)

target_compile_definitions(${PROJECT_NAME} PUBLIC SBG_COMMON_STATIC_USE)
set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if (MSVC)
    target_link_libraries(${PROJECT_NAME} PUBLIC Ws2_32)
endif()


