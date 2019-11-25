#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#ifdef WIN32
#else
    #include <math.h>
    #include <iostream>
    #include <websocketpp/client.hpp>
    #include <sys/socket.h>
    #include <vector>
    #include <algorithm>
    #include <numeric>
    #include <future>
    #include <string>
    #include <mutex>
    #include <thread>
    #include <curlpp/cURLpp.hpp>
#endif

namespace nimbus {
class WebSocketClient{
    std::mutex m_mutex;
private:
    int NimbusImageRaw          = 1;
    int NimbusImageDist         = 2;
    int NimbusImageAmpl         = 4;
    int NimbusImageX            = 8;
    int NimbusImageY            = 16;
    int NimbusImageZ            = 32;
    int NimbusImageConf         = 64;

    int HeaderImgType          = 2;
    int HeaderROIWidth         = 3;
    int HeaderROIHeight        = 4;
    int HeaderROITop           = 5;
    int HeaderROILeft          = 6;
    int HeaderNumSequences     = 7;
    int HeaderFPS              = 8;
    int HeaderTemperature      = 9;
    int HeaderReconfigCnt      = 11;
    int HeaderMetaFrameCounter = 12;
    int HeaderSequenceRestarts = 13;

    int ConfValid          = 0;
    int ConfUnderExposured = 1;
    int ConfOverExposured  = 2;
    int ConfAsymmetric     = 3;
public:
    WebSocketClient(std::string _addr, bool continuousTrig = false, 
                    double port = 8080, double jsonPort = 8383, 
                    int rcvTimeout = 3, int pingTimeout = 3, 
                    int reconnectIntents = 3, double imgBufSize =10);
    ~WebSocketClient();
    /** ToDo
     *Need to check the return data type
     * Asynchronous function for streaming the camera data
     * JSON RPC to control the exposer time is not used*/
    void listenForever();
    void _pollQueue();
    void connect();
    void disconnect();
    void getImage();
    void getUnitVectorX();
    void getUnitVectorY();
    void getUnitVectorZ();
    
    template<typename C, typename P, typename A, typename T>
    T _getJsonParameter(C componet, P paramID, A args);

    std::string _address, _streamURL;
    double _streamPort, _jsonPort, _UR;
    int _rcvTimeout, _pingTimeout, _reconnectIntents, _imgBufSize;
    bool _listenStarted, _listenEnded, _connected, _disconnectMe;
    std::thread _threadUpdate;
    //_imageQueue
};
}

#endif // WEBSOCKET_H
