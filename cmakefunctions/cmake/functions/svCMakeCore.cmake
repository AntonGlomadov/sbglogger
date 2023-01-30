macro(svCMakeCore BUILD_TYPE)

    #create project folder structure
    GroupSources(inc)
    GroupSources(src)
    GroupSources(res)

    setName()#create project name
    file( GLOB_RECURSE all_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*" )

    if(${BUILD_TYPE} STREQUAL "LIB")
        add_library( "${PROJECT_NAME}" ${all_SRCS} )
    elseif (${BUILD_TYPE} STREQUAL "SHARED_LIB")
        add_library( "${PROJECT_NAME}" SHARED ${all_SRCS} )
        add_custom_command(
                TARGET ${PROJECT_NAME}
                PRE_BUILD
                COMMAND ${CMAKE_COMMAND}
                -DVERSION_FILE:PATH="${CMAKE_CURRENT_SOURCE_DIR}/version.txt"
                -DRESOURCE_FILE:PATH="${CMAKE_CURRENT_SOURCE_DIR}/res/version.rc"
                -DPACKAGE_NAME:STRING="${PROJECT_NAME}"
                -P "${CMAKE_EXE_DIR}/build_number.cmake"
        )
    elseif (${BUILD_TYPE} STREQUAL "APP")
        add_executable( "${PROJECT_NAME}" ${all_SRCS} )
        #execute the script at compile time
        add_custom_command(
                TARGET ${PROJECT_NAME}
                PRE_BUILD
                COMMAND ${CMAKE_COMMAND}
                -DVERSION_FILE:PATH="${CMAKE_CURRENT_SOURCE_DIR}/version.txt"
                -DRESOURCE_FILE:PATH="${CMAKE_CURRENT_SOURCE_DIR}/res/version.rc"
                -DPACKAGE_NAME:STRING="${PROJECT_NAME}"
                -P "${CMAKE_EXE_DIR}/build_number.cmake"
        )
    endif()

    #add config options to different compilers
    Config( "${PROJECT_NAME}" )

    target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc/" )
endmacro()
