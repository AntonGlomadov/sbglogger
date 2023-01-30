macro(config PROJECT_NAME)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_compile_options(${PROJECT_NAME} PRIVATE -Wall -pedantic -pthread )
        # using Clang
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${PROJECT_NAME} PRIVATE -Wall -pedantic -pthread )
        # using GCC
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
        # using Intel C++
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        # using Visual Studio C++
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
        set_property(TARGET "${PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
        target_compile_options(${PROJECT_NAME} PRIVATE /W4 )
        target_compile_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:RELEASE>:/WX>" )
    endif()
endmacro()
