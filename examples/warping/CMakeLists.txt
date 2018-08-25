# determine source and header files

get_filename_component(_TUTORIAL_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(_EXE_NAME ${_TUTORIAL_NAME})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
  include_directories( ${INCLUDE_PATHS} ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/examples/warping)

set(SOURCES Navigator.hpp)
add_executable(${_EXE_NAME} main.cpp Navigator.cpp)

if(${PLUGIN_guacamole_lod})
  target_link_libraries(${_EXE_NAME} guacamole-lod)
else(${PLUGIN_guacamole_lod})
  target_link_libraries(${_EXE_NAME} guacamole)
endif(${PLUGIN_guacamole_lod})

# copy runtime libraries as a post-build process
IF (MSVC)
  FOREACH(_LIB ${GUACAMOLE_RUNTIME_LIBRARIES})
    get_filename_component(_FILE ${_LIB} NAME)
    get_filename_component(_PATH ${_LIB} DIRECTORY)
    SET(COPY_DLL_COMMAND_STRING ${COPY_DLL_COMMAND_STRING} robocopy \"${_PATH}\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" ${_FILE} /R:0 /W:0 /NP &)
  ENDFOREACH()

  SET(COPY_DLL_COMMAND_STRING ${COPY_DLL_COMMAND_STRING} robocopy \"${LIBRARY_OUTPUT_PATH}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" *.dll /R:0 /W:0 /NP &)
  ADD_CUSTOM_COMMAND ( TARGET ${_EXE_NAME} POST_BUILD COMMAND ${COPY_DLL_COMMAND_STRING} \n if %ERRORLEVEL% LEQ 7 (exit /b 0) else (exit /b 1))
ENDIF (MSVC)