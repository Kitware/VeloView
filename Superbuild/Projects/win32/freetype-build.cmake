set(ENV{JAM_TOOLSET} VISUALC)
execute_process(COMMAND ${FTJAM_EXECUTABLE}
                WORKING_DIRECTORY ${WORKING_DIRECTORY}
                OUTPUT_VARIABLE output
                ERROR_VARIABLE error
                RESULT_VARIABLE rv)

if(NOT ${rv} EQUAL 0)
  message(FATAL_ERROR "Error building freetype!\n${output}\n${error}")
endif()
