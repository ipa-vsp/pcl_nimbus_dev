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

        //acquired thread
        //this->connect();
        // ..
        double spread = this->getSpreadFactorXYZ<double>();
        std::vector<std::vector<int16_t> > unitX = this->getUnitVectorX<std::vector<std::vector<int16_t> >>();
        std::vector<std::vector<int16_t> > unitY = this->getUnitVectorY<std::vector<std::vector<int16_t> >>();
        std::vector<std::vector<int16_t> > unitZ = this->getUnitVectorZ<std::vector<std::vector<int16_t> >>();
        
        for (size_t j = 0; j < unitX[0].size() ; j++)
            for(size_t i = 0; i < unitX.size(); i++)
            {
                int16_t temp = unitX[i][j];
                this->_uX[i][j] = (double)temp / spread;
            }
        
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
            //std::u16string wDecode = std::u16string(decode.begin(), decode.end());
            int16_t* my_vec_x = (int16_t*)decode.c_str();
            std::vector<std::vector<int16_t> > my_2d_vec(286, std::vector<int16_t>(352, 0));
            //std::cout << "Row size: " << my_2d_vec.size() << "Column size: " << my_2d_vec[0].size() << std::endl;
            int counter = 0;
            for (size_t j = 0; j < my_2d_vec[0].size(); j++)           //column
            {
                for(size_t i = 0; i <my_2d_vec.size() ; i++)        //Row
                {
                    my_2d_vec[i][j] = my_vec_x[counter];
                    counter ++;
                }
            }
            resultValue =  my_2d_vec;
        }
        return (resultValue);
    }
    template<typename D>
    D WebSocketClient::getUnitVectorY(){
        D resultValue;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 5, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0")
        {
            std::string data(result_r["result"].asString());
            std::string decode = base64_decode(data);
            //std::u16string wDecode = std::u16string(decode.begin(), decode.end());
            int16_t* my_vec_y = (int16_t*)decode.c_str();
            std::vector<std::vector<int16_t> > my_2d_vec(286, std::vector<int16_t>(352, 0));
            //std::cout << "Row size: " << my_2d_vec.size() << "Column size: " << my_2d_vec[0].size() << std::endl;
            int counter = 0;
            for (size_t j = 0; j < my_2d_vec[0].size(); j++)           //column
            {
                for(size_t i = 0; i <my_2d_vec.size() ; i++)        //Ros
                {
                    my_2d_vec[i][j] = my_vec_y[counter];
                    counter ++;
                }
            }
            resultValue =  my_2d_vec;
        }
        return (resultValue);
    }
    template<typename D>
    D WebSocketClient::getUnitVectorZ(){
        D resultValue;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 6, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0")
        {
            std::string data(result_r["result"].asString());
            std::string decode = base64_decode(data);
            //std::u16string wDecode = std::u16string(decode.begin(), decode.end());
            int16_t* my_vec_z = (int16_t*)decode.c_str();
            std::vector<std::vector<int16_t> > my_2d_vec(286, std::vector<int16_t>(352, 0));
            //std::cout << "Row size: " << my_2d_vec.size() << "Column size: " << my_2d_vec[0].size() << std::endl;
            int counter = 0;
            for (size_t j = 0; j < my_2d_vec[0].size(); j++)           //column
            {
                for(size_t i = 0; i <my_2d_vec.size() ; i++)        //Ros
                {
                    my_2d_vec[i][j] = my_vec_z[counter];
                    counter ++;
                }
            }
            resultValue =  my_2d_vec;
        }
        return (resultValue);
    }

    template<typename D>
    D WebSocketClient::getSpreadFactorXYZ()
    {
        D returnDouble;
        Json::Value result_r = this->_getJsonParameter<Json::Value>("{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 7, \"param\": []} }");
        std::string rv(result_r["success"].asString());
        if(rv == "0"){
            std::string data(result_r["result"].asString());
            double dResult = std::stod(data);
            returnDouble = dResult;
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
        CURL *curl = curl_easy_init();
        struct curl_slist *headers = NULL;
        CURLcode res;
        std::string readBuffer;
        unsigned char * url = this->_address;       //ToDo Change this to string
        long httpCode(0);
        std::unique_ptr<std::string> httpData(new std::string());
        const std::string result_string;
        T resultJson;

        if(curl)
        {
            /** ToDo:
             * @brief Use the json paeser construct that payload and use this as string!!
             */
            const char *payload = data; 
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
                jsonReader.parse(jsonData["result"].toStyledString(), jsonData);
                const std::string dataString(jsonData["result"].asString());
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