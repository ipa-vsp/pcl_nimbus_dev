#include "websocket.h"

namespace nimbus{
    WebSocketClient::WebSocketClient(unsigned char * addr, bool continuousTrig = false, 
                    double port = 8080, double jsonPort = 8383, 
                    int rcvTimeout = 3, int pingTimeout = 3, 
                    int reconnectIntents = 3, double imgBufSize =10)
    {
        this->_address = addr;
        this->_streamPort = port;
        // this->_streamURL = "ws://%s:%d/stream", addr, port;
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

        this->curl = curl_easy_init();

        //acquired thread
        //this->connect();
        // ..
        //std::string spreadString = this->getSpreadFactorXYZ<std::string>();
        //double spread = std::stof(spreadString);
        this->getUnitVectorX<std::string>();
    }

    WebSocketClient::~WebSocketClient(){}

    template<typename D>
    D WebSocketClient::getUnitVectorX(){
        D resultValue;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 4, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0")
        {
            std::string data(result_r["result"].asString());
            std::string decode = base64_decode(data);
            //int a = std::stoi(decode);
            std::cout << "decoded: " << std::endl << decode << std::endl;
            // the UTF-8 / UTF-16 standard conversion facet
            std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> utf16conv;
            std::u16string utf16 = utf16conv.from_bytes(decode);
            std::cout << "UTF16 conversion produced " << utf16.size() << " code units:\n";
            for (char16_t c : utf16)
                std::cout << std::hex << std::showbase << c << '\n';
           
        }
        //result_r = result_r.erase(std::remove(result_r.begin(), result_r.end(), "\\"), result_r.end());
        //std::cout << result_r << std::endl;
        //std::string decoded = base64_decode(result_r);
        //std::cout << "decoded: " << std::endl << decoded << std::endl;
    }

    template<typename D>
    D WebSocketClient::getSpreadFactorXYZ()
    {
        D returnDouble = NULL;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 7, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0"){
            std::string data(result_r["result"].asString());
            returnDouble = data;
        }
        return (returnDouble);
    }

    size_t WebSocketClient::WriteCallback(const char* in, std::size_t size, std::size_t num, std::string* out)
    {
        const std::size_t totalBytes(size * num);
        out->append(in, totalBytes);
        return totalBytes;
    }

    template<typename T, typename D>
    T WebSocketClient::_getJsonParameter(D data)
    {
        //CURL *curl = curl_easy_init();
        struct curl_slist *headers = NULL;
        CURLcode res;
        std::string readBuffer;
        unsigned char * url = this->_address;       //ToDo Change this to string
        long httpCode(0);
        std::unique_ptr<std::string> httpData(new std::string());
        const std::string result_string;
        T resultJson;

        if(this->curl)
        {
            /** ToDo:
             * @brief Use the json paeser construct that payload and use this as string!!
             */
            const char *payload = data;                           
            //"{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"%s\", \"ID\": %d, \"param\": []} }", componet, paramID;
            headers = curl_slist_append(headers, "content-type: application/json;");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_URL, url);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long) strlen(payload));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &this->WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, httpData.get());

            res = curl_easy_perform(curl);
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
            if(res != CURLE_OK){
                fprintf(stderr, "curl_easy_persform() failed: %s\n", curl_easy_strerror(res));
            }
            curl_easy_cleanup(curl);
        }

        if(httpCode == 200)
        {
            Json::Value jsonData;
            Json::Reader jsonReader;
            if(jsonReader.parse(*httpData.get(), jsonData))
            {
                /*std::cout << "Successfully parsed JSON data" << std::endl;
                std::cout << "\nJSON data received:" << std::endl;
                std::cout << jsonData.toStyledString() << std::endl;*/

                jsonReader.parse(jsonData["result"].toStyledString(), jsonData);
                const std::string dataString(jsonData["result"].asString());
                std::cout << "Data------:  " << dataString <<std::endl;
                resultJson = jsonData;
            }else
            {
                std::cout << "Could not parse HTTP data as JSON" << std::endl;
                std::cout << "HTTP data was:\n" << *httpData.get() << std::endl;
                return "error";
            }
            
        }else
        {
            std::cout << "Couldn't GET from " << url << " - exiting" << std::endl;
            return "error";
        }
        curl_global_cleanup();
        return (resultJson);
    }


}