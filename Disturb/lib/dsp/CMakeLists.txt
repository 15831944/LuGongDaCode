cmake_minimum_required(VERSION 3.5)


add_library(dsp
	DFT.cpp
	Filter.cpp
	SigProcLib.cpp
	DigitalDemod.cpp
	FirDesign.cpp
	SSE.cpp
	DataSource.cpp
	DigitalMod.cpp
	PrbsGenerator.cpp
	Resampler.cpp
	Ols_filter.cpp
	FFT.cpp
        )
target_link_libraries(dsp ${DISTURB_LINK_LIBS})
