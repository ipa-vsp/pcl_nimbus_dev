#include "websocket.h"

int main(int argc, char** argv)
{
    nimbus::WebSocketClient a((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 1, 1, 1, 1);
    
    std::string c = a._getJsonParameter("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 8, \"param\": []} }");
    std::cout << c << std::endl;
    return 0;
}