#include "websocket.h"

#include <time.h>

int main(int argc, char** argv)
{
    nimbus::WebSocketClient a((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10);  
    std::string uri = "ws://192.168.0.69:8080/stream";
    // Run the threads inside class
    clock_t start, end;
    double sumTime = 0;
    int count = 1;
    std::vector<std::vector<float>> res = a.getImage();
    //std::cout << raxyz.x[0] << std::endl;
    while (true){
        start = clock();
        res = a.getImage();
        end = clock();
        std::cout << "Row size: " << res.size() << " Column size: " << res[0].size() << std::endl;
        double time = (double)(end - start) / CLOCKS_PER_SEC;
        sumTime += time;
        double average = sumTime / count;
        count ++;
        if(count == 100){
            printf("Get image average time: %f s\n", average);
            break;
        }
        
    }
    // Once the main thread reach return it will destruct the class and kills the neccessary thread
    return 0;
}