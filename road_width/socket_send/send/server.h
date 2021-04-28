//
// Created by renjie on 2021/2/7.
//

#ifndef TOPICSERVER_SERVER_H
#define TOPICSERVER_SERVER_H


#include<stdlib.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<sys/socket.h>
#include<string.h>
#include<string>
using namespace  std;

class CServer{
private:
    int _servSockfd;
    int _clientSockfd;

    struct sockaddr_in _servAddr;
    struct sockaddr_in _clientAddr;
    char *_port;

    int _dataSize;

public:
    CServer( char* port, int dataSize);
    ~CServer();

    void send(const void *dataPtr);
};

#endif //TOPICSERVER_SERVER_H
