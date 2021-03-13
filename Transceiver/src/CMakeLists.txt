#    add_executable(TransceiverDevice TransceiverDevice.cpp)
#    target_link_libraries(TransceiverDevice TransceiverDemo)
#
#    add_executable(Client Client.cpp)
#    target_link_libraries(Client TransceiverDemo)

add_executable(Trans TransceiverClient.cpp)
target_link_libraries(Trans TransceiverDemo)