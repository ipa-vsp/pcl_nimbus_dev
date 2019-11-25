#include "websocket.h"

namespace nimbus{
    WebSocketClient::WebSocketClient(std::string addr, bool continuousTrig = false, 
                    double port = 8080, double jsonPort = 8383, 
                    int rcvTimeout = 3, int pingTimeout = 3, 
                    int reconnectIntents = 3, double imgBufSize =10)
    {
        this->_address = addr;
        this->_streamPort = port;
        this->_streamURL = "ws://%s:%d/stream", addr, port;
        this->_jsonPort = jsonPort;

        this->_rcvTimeout = rcvTimeout;
        this->_pingTimeout = pingTimeout;
        this->_reconnectIntents = reconnectIntents;

        this->_imgBufSize = imgBufSize;
        this->_listenStarted = false;
        this->_listenEnded = false;
        this->_connected = false;
        // thread
        this->_disconnectMe = false;

        // asyncio
        // image Queue

        double c_ = 299792458;
        double fmod = 11.78e6;
        this->_UR = c_/(2 * fmod);

        //acquired thread
        this->connect();
        // ..
    }

    template<typename C, typename P, typename A, typename T>
    T WebSocketClient::_getJsonParameter(C componet, P paramID, A args){
        std::string url = "http://%s:%d/jsonrpc", this->_address, this->_jsonPort;
        this->
    }


}