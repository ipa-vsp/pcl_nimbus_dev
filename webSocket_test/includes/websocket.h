#ifndef WEBSOCKET_H
#define WEBSOCKET_H

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
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <bits/stdc++.h> 
#include <boost/algorithm/string.hpp>  
#include <curl/curl.h>

#include "base64.h"

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
     CURL *curl;
public:
    WebSocketClient(unsigned char * _addr, bool continuousTrig, 
                    double port, double jsonPort, 
                    int rcvTimeout, int pingTimeout, 
                    int reconnectIntents, double imgBufSize);
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
    double getSpreadFactorXYZ();
    
    std::string _getJsonParameter(const char * data);
    static size_t WriteCallback(char *contents, size_t size, size_t nmemb, void *userp);

    unsigned char * _address, _streamURL;
    double _streamPort, _jsonPort, _UR;
    int _rcvTimeout, _pingTimeout, _reconnectIntents, _imgBufSize;
    bool _listenStarted, _listenEnded, _connected, _disconnectMe;
    std::thread _threadUpdate;
    //_imageQueue
};
}

#endif // WEBSOCKET_H
