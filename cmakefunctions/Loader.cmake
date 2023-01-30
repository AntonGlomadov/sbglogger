macro(Loader DIR)
    include( ${DIR}/cmakefunctions/cmake/functions/group.cmake )
    include( ${DIR}/cmakefunctions/cmake/functions/config.cmake )
    include( ${DIR}/cmakefunctions/cmake/functions/setName.cmake )
    set ( CMAKE_EXE_DIR ${DIR}/cmakefunctions/cmake/executable )

    include( ${DIR}/cmakefunctions/cmake/functions/svCMakeCore.cmake )
endmacro()
