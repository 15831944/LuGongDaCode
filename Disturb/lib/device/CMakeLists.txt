cmake_minimum_required(VERSION 3.5)



add_library(device
		Disturb.cpp
		IntelligentDecision.cpp
		CtrlUsrp.cpp
		Transmitter.cpp
		)

target_link_libraries(device ${DISTURB_LINK_LIBS})
