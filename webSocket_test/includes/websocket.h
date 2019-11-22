#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#ifdef WIN32
#else
    #include <websocketpp/client.hpp>
    #include <sys/socket.h>
#endif

#endif // WEBSOCKET_H
