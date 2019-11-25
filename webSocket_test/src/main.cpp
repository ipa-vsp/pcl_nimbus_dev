#include <iostream>
#include <string>
#include <sstream>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Option.hpp>

int main(int argc, char** argv)
{
    curlpp::Cleanup myCleanup;
    std::ostringstream os;
    os << curlpp::options::Url(std::string("http://www.wikipedia.org"));
    std::cout << "Hello World!" << std::endl;
    return 0;
}