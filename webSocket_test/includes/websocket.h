#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <math.h>
#include <sys/socket.h>
#include <algorithm>
#include <numeric>
#include <future>
#include <mutex>
#include <thread>
#include <queue>
#include <stdlib.h>
#include <stdio.h>
#include <bits/stdc++.h> 
#include <boost/algorithm/string.hpp>
//#include <boost/python/detail/wrap_python.hpp>  
//#include <boost/python/numpy.hpp>  
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include "base64.h"

#include<iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdint>
#include <locale>
#include <codecvt>
#include <vector>

#include "connectWS.h"


//namespace p = boost::python;
//namespace np = boost::python::numpy;
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

    unsigned char * _address, _streamURL;
    double _streamPort, _jsonPort, _UR;
    int _rcvTimeout, _pingTimeout, _reconnectIntents, _imgBufSize;
    bool _listenStarted;
    bool _listenEnded, _connected, _disconnectMe;
    std::thread _threadUpdate;

    float_t spread;
    float * _uX = new float[286*352];
    float * _uY = new float[286*352];
    float * _uZ = new float[286*352];

    std::thread _listenThread;
    websocket_endpoint endpoint;
    connection_metadata::ptr metadata;

public:
    WebSocketClient(unsigned char * _addr, bool continuousTrig, 
                    double port, double jsonPort, 
                    int rcvTimeout, int pingTimeout, 
                    int reconnectIntents, double imgBufSize);
    ~WebSocketClient();
    //void on_message(client* c, websocketpp::connection_hdl hdl, message_ptr msg);
    void listenerThread();
    void _pollQueue();
    void connect();
    void disconnect();
    void getImage();

    template<typename D>
    D getUnitVectorX();
    template<typename D>
    D getUnitVectorY();
    template<typename D>
    D getUnitVectorZ();
    
    template<typename D>
    D getSpreadFactorXYZ();

    float * unpack(std::string buf);

    void normalize(std::vector<std::vector<int16_t>> unit, std::vector<std::vector<float_t>> * _u);
    
    template<typename T, typename D>
    T _getJsonParameter(D data);
    static size_t WriteCallback(const char* in, std::size_t size, std::size_t num, std::string* out);

     std::queue<std::string> _imageQueue;
};
}

#endif // WEBSOCKET_H
