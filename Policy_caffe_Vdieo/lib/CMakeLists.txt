set(SRC_LIBS Policy.cpp DFT.cpp GNUPlot.cpp GNUPlotIface.cpp NTPclient.cpp USB.cpp dqn.cpp)
add_library(Policy SHARED ${SRC_LIBS})
target_link_libraries(Policy ${POLICY_LINK_LIBS})

