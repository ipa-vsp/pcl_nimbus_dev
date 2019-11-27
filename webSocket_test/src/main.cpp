#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h> 
#include <curl/curl.h>


size_t WriteCallback(char *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}


int main(int argc, char** argv) {
   CURL *curl = curl_easy_init();
   struct curl_slist *headers = NULL;
   CURLcode res;
   std::string readBuffer;

   if(curl){
       const char *data ="{\"jsonrpc\": \"2.0\", \"id\":\"0\", \"method\": \"getParameter\", \"params\": {\"component\": \"nimbusRaw\", \"ID\": 8, \"param\": []} }";
       headers = curl_slist_append(headers, "content-type: application/json;");
       curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

       curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.69:8383/jsonrpc");
       curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long) strlen(data));
       curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

       res = curl_easy_perform(curl);
       std::cout << readBuffer << std::endl;

       if(res != CURLE_OK){
           fprintf(stderr, "curl_easy_persform() failed: %s\n", curl_easy_strerror(res));
       }
       curl_easy_cleanup(curl);
   }
   curl_global_cleanup();
   return 0;
}