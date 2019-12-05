#include "websocket.h"


int main(int argc, char** argv)
{
    nimbus::WebSocketClient a((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10);  
    std::string uri = "ws://192.168.0.69:8080/stream";
    // Run the threads inside class 
    while (true){
        a.getImage();
    }
    // Once the main thread reach return it will destruct the class and kills the neccessary thread
    return 0;
}