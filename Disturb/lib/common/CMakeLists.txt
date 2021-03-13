cmake_minimum_required(VERSION 3.5)


add_library(common
    Types.cpp
    Configuration.cpp
    GNUPlot.cpp
    Sclient.cpp
    Thread.cpp
    GPS.cpp
    SCPIMessage.cpp
    SCPIPrase.cpp
    Logger.cpp
    Sockets.cpp
   )

target_link_libraries(common ${DISTURB_LINK_LIBS})
