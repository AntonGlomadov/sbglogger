macro(setName)
    get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    string(REPLACE " " "_" PROJECT_NAME ${PROJECT_NAME})
endmacro()
