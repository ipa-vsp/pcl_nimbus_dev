#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>

#include <curl/curl.h>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <assert.h>

int main(int argc, char** argv) {
    auto url = cpr::Url{"http://192.168.0.69:8383/jsonrpc"};
    auto header = cpr::Header{{"content-type", "application/json"}};
    auto parameters = cpr::Parameters{{"hello", "world"}};
    cpr::Session session;
    session.SetUrl(url);
    session.SetParameters(parameters);
    session.SetHeader(header);

    auto r = session.Post();             // Equivalent to cpr::Get(url, parameters);
    std::cout << r.url << std::endl;     // Prints http://www.httpbin.org/get?hello=world


}