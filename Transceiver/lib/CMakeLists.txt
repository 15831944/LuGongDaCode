    add_library(TransceiverDemo STATIC
            BitVector.cpp
            DetectSynSeq.cpp
            Frame.cpp
            GmskImpl.cpp
            LinkLayer.cpp
            ModulatorBase.cpp
            NTPclient.cpp
            PhysicsLayer.cpp
            SigProcLib.cpp
            UHDDevice.cpp
            BPSKImpl.cpp
            RootRaisedCosine.cpp
            )
    target_link_libraries(TransceiverDemo ${LINK_LIBS})