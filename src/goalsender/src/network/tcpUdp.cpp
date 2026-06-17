#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cstdlib>
#include "network/tcpUdp.hpp"

udpTool::udpTool(struct ipPort _sourceIpPort, MODULE_TYPE moduleId) : netWorkBase(_sourceIpPort, moduleId)
{
    this->threadStopFlag_ = true;
};

udpTool::~udpTool()
{
    this->endRecvThread();
    this->destroryNet();
}

ERROR_NUM udpTool::createNet(int _bufSize)
{
    this->fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(this->fd_ < 0)
    {
        printError("Socket create failed!\n");
        return SOCK_CREATE_FAIL;
    }
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(this->sourceIpPort_.port);
    if(!this->sourceIpPort_.ip.empty())
    {
        if (inet_pton(AF_INET, this->sourceIpPort_.ip.c_str(), &address.sin_addr) <= 0) {
            printError("Ip attach failed!\n");
            return SOCK_CREATE_FAIL;
        }
    }
    else {
        address.sin_addr.s_addr = INADDR_ANY;
    }
    if (bind(this->fd_, (sockaddr*)&address, sizeof(address)) < 0) {
        printError("Socket bind failed!\n");
        return SCOKET_BIND_FAIL;
    }
    this->bufSize_ = _bufSize;
    this->dataBuf_ = new char[_bufSize];
    if(this->dataBuf_ == nullptr)
    {
        printError("alloc socket data buf failed!\n");
        return SCOKET_BIND_FAIL;
    }
    return SUCCESS;
}

void udpTool::serverWorkThread()
{
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char* socketBuf = new char[this->bufSize_];
    if(!socketBuf)
    {
        printError("Thread buf alloc failed\n");
        return;
    }
    while(!this->threadStopFlag_)
    {
        int bytes_read = recvfrom(this->fd_, socketBuf, this->bufSize_, 0, (sockaddr*)&client_addr, &addr_len);
        if(bytes_read > 0)
        {
            this->dataAccess_.lock();
            memcpy(this->dataBuf_, socketBuf, bytes_read);
            this->dataSize = bytes_read;
            this->dataAccess_.unlock();
            if(this->callBack_)
            {
                this->callBack_(this, this->dataBuf_, bytes_read, client_addr);
            }
        }
    }
    delete[] socketBuf;
}

ERROR_NUM udpTool::startRecvThread(callBackFunc _callBack)
{
    if(this->fd_ < 0)
    {
        printError("Socket not created, must call createNet() to create socket before startRecvThread\n");
        exit(1);
    }
    this->callBack_ = _callBack;
    this->threadStopFlag_ = false;

    if(_callBack == NULL)
    {
        printError("_callBack should set when work as server!\n");
        return THREAD_CALLBACK_FAIL;
    }
    this->threadHandler_ = new std::thread(&udpTool::serverWorkThread, this);
    return ERROR_NUM::SUCCESS;
}

ERROR_NUM udpTool::endRecvThread()
{
    this->threadStopFlag_ = true;
    if(this->threadHandler_ != NULL)
    {
        if(this->threadHandler_->joinable())
        {
            this->threadHandler_->join();
        }
    }
    this->threadHandler_ = NULL;
    return ERROR_NUM::SUCCESS;
}

ERROR_NUM udpTool::destroryNet()
{
    if(this->dataBuf_ != NULL)
    {
        delete[] this->dataBuf_;
        this->dataBuf_ = NULL;
        this->bufSize_ = 0;
        this->dataSize = 0;
    }
    if(this->fd_ > 0)
    {
        close(this->fd_);
        this->fd_ = -1;
    }
    print(DEBUG, "Destroyed\n");
    return ERROR_NUM::SUCCESS;
}

NETDATALEN udpTool::getData(void* buf, int len)
{
    int res = -1;
    if(this->threadStopFlag_)
    {
        printError("Receive thread not exist, must call startRecvThread() to create thread\n");
        exit(1);
    }
    this->dataAccess_.lock();
    if(this->dataSize > 0)
    {
        if(this->dataSize > len)
        {
            printError("get data: Buf too small!\n");
        }
        else
        {
            memcpy(buf, this->dataBuf_, this->dataSize);
            res = this->dataSize;
            this->dataSize = 0;
        }
    }
    this->dataAccess_.unlock();
    return res;
}

NETDATALEN udpTool::sendData(void* buf, int len, struct ipPort _destIpPort)
{
    ssize_t res = -1;
    if(this->fd_ < 0)
    {
        printError("Socket not created, must call createNet() to create socket before send data\n");
        exit(1);
    }
    sockaddr_in client_addr;
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(_destIpPort.port);
    inet_pton(AF_INET, _destIpPort.ip.c_str(), &client_addr.sin_addr);
    res = sendto(this->fd_, buf, len, 0, (sockaddr*)&client_addr, sizeof(client_addr));
    return res;
}
