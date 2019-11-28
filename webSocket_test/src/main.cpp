#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <bits/stdc++.h> 
#include <boost/algorithm/string.hpp>  
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <iconv.h>


size_t WriteCallback(char *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

namespace
{
    std::size_t callback(
            const char* in,
            std::size_t size,
            std::size_t num,
            std::string* out)
    {
        const std::size_t totalBytes(size * num);
        out->append(in, totalBytes);
        return totalBytes;
    }
}


int main(int argc, char** argv) {
   CURL *curl = curl_easy_init();
   struct curl_slist *headers = NULL;
   CURLcode res;
   std::string readBuffer;
   long httpCode(0);
   std::unique_ptr<std::string> httpData(new std::string());

   if(curl){
       const char *data ="{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 4, \"param\": []} }";
       headers = curl_slist_append(headers, "content-type: application/json;");
       curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

       curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.69:8383/jsonrpc");
       curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long) strlen(data));
       curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, httpData.get());

       res = curl_easy_perform(curl);
       curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
       if(res != CURLE_OK){
           fprintf(stderr, "curl_easy_persform() failed: %s\n", curl_easy_strerror(res));
       }
       curl_easy_cleanup(curl);
       std::cout << httpCode << std::endl;

       if(httpCode == 200)
       {
           std::cout << "\nGot successful response from: " << "http://192.168.0.69:8383/jsonrpc" << std::endl;
           
           // Response looks good - done using Curl now.  Try to parse the results
           // and print them out.
           Json::Value jsonData;
           Json::Reader jsonReader;

           if(jsonReader.parse(*httpData.get(), jsonData))
           {
               std::cout << "Successfully parsed JSON data" << std::endl;
               std::cout << "\nJSON data received:" << std::endl;
               std::cout << jsonData.toStyledString() << std::endl;

               jsonReader.parse(jsonData["result"].toStyledString(), jsonData);
               const std::string dataString(jsonData["result"].asString());
               std::cout << "Data------:  " << dataString <<std::endl;
           }
       }
   }
   curl_global_cleanup();
   return 0;
}


/*std::string Decode(const std::string &encoded, CURL *curl)
{
    int outlength;
    char *cres = curl_easy_unescape(curl, encoded.c_str(), encoded.length(), &outlength);
    std::string res(cres, cres + outlength);
    curl_free(cres);
    curl_easy_cleanup(curl);

    //if it's UTF-8, convert it to ISO_8859-1. Based on https://stackoverflow.com/questions/11156473/is-there-a-way-to-convert-from-utf8-to-iso-8859-1/11156490#11156490
    iconv_t cd = iconv_open("ISO_8859-1", "UTF-8");

    const char *in_buf = res.c_str();
    size_t in_left = res.length();

    char *output = new char[res.length() + 1];
    std::fill(output, output + res.length() + 1, '\0');
    char *out_buf = &output[0];
    size_t out_left = res.length();

    do {
        if (iconv(cd, &in_buf, &in_left, &out_buf, &out_left) == (size_t)-1) {
            //failed to convert, just return the value received from curl
            delete[] output;
            iconv_close(cd);
            return res;
        }
    } while (in_left > 0 && out_left > 0);

    std::string outputString(output);
    delete[] output;
    iconv_close(cd);

    return outputString;
}*/