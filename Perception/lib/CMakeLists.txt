#add_library(Perception STATIC Perception.cpp DFT.cpp GNUPlot.cpp GNUPlotIface.cpp NTPclient.cpp USB.cpp)
add_library(Perception STATIC Perception.cpp DFT.cpp NTPclient.cpp USB.cpp)
target_link_libraries(Perception ${LINK_LIBS})