//
// Created by renjie on 2021/2/7.
//

#include "server.h"

CServer::CServer( char* port, int dataSize):_port(port),_dataSize(dataSize){
    _servSockfd=socket(PF_INET, SOCK_STREAM, 0);

    memset(&_servAddr, 0, sizeof(_servAddr));
    _servAddr.sin_family = AF_INET;
    _servAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    _servAddr.sin_port = htons(atoi(_port));


        if( bind( _servSockfd, (struct sockaddr*)&_servAddr, sizeof(_servAddr)) == -1 ) throw string(" function bind error ");
        if( listen( _servSockfd, 5)==-1 ) throw string(" function listen error");
        socklen_t clientAddrSize = sizeof(_clientAddr);
        _clientSockfd = accept( _servSockfd, ( struct sockaddr*)&_clientAddr, &clientAddrSize);
        if( _clientSockfd == -1) throw string("function accept error");

}

void CServer::send(const void *dataPtr) {
    write(_clientSockfd, dataPtr, _dataSize);
}


CServer::~CServer() {
    close(_clientSockfd);
    close(_servSockfd);
}