#include "websocket.h"


int main(int argc, char** argv)
{
    nimbus::WebSocketClient a((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10);  
    std::string uri = "ws://192.168.0.69:8080/stream";
    /*websocket_endpoint endpoint;
    connection_metadata::ptr metadata;
    int id = endpoint.connect("ws://192.168.0.69:8080/stream");
        metadata = endpoint.get_metadata(id);
        if (metadata) {
            std::cout << metadata.get() << std::endl;
        } else {
            std::cout << "> Unknown connection id " << id << std::endl;
        }*/
    while (true){}
    return 0;
}