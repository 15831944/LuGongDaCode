add_subdirectory(common)
add_subdirectory(dsp)
add_subdirectory(device)




add_library(DSP lDSP.cc)
target_link_libraries(DSP
  "-Wl,--whole-archive" 
  common
  device
  dsp
  "-Wl,--no-whole-archive" 
  )
