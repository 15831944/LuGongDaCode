cmake_minimum_required(VERSION 3.5)
# example applications
########################################################################
SET(example_sources
        main.cpp
        )
#for each source: build an executable and install
FOREACH(example_source ${example_sources})
    GET_FILENAME_COMPONENT(example_name ${example_source} NAME_WE)
    ADD_EXECUTABLE(${example_name} ${example_source})
    TARGET_LINK_LIBRARIES(${example_name} DSP)
    #INSTALL(TARGETS ${example_name} RUNTIME DESTINATION ${PKG_LIB_DIR}/examples COMPONENT examples)
ENDFOREACH(example_source)

