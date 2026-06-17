#ifndef __NETWORKBASE__HPP
#define __NETWORKBASE__HPP

#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "errorNum.hpp"
#include "moduleObj.hpp"

#define NETDATALEN int

struct ipPort
{
    std::string ip;
    unsigned short port;
    bool operator!=(struct ipPort& other){
        if(this->ip != other.ip | this->port != other.port)
        {
            return true;
        }
        return false;
    }
};

enum C_S_TYPE
{
    CLIENT = 0,
    SERVER
};

class netWorkBase;

typedef void (*callBackFunc)(netWorkBase* nethandler, void* buf, int size, sockaddr_in client_addr);

class netWorkBase{
protected:
    int fd_ = -1;

    MODULE_TYPE moduleId;
    struct ipPort sourceIpPort_;
    std::thread* threadHandler_;
    std::mutex dataAccess_;
    std::atomic<bool> threadStopFlag_;

    char* dataBuf_;
    int bufSize_;
    int dataSize;

    void *privateData = nullptr;

    std::unordered_map<int, struct ipPort*> ipMapTable;

    callBackFunc callBack_;

public:
    netWorkBase(struct ipPort _sourceIpPort, MODULE_TYPE moduleId);
    virtual ~netWorkBase();

    virtual ERROR_NUM createNet(int _bufSize) = 0;
    virtual ERROR_NUM startRecvThread(callBackFunc _callBack) = 0;
    virtual ERROR_NUM endRecvThread() = 0;
    virtual ERROR_NUM destroryNet() = 0;

    virtual NETDATALEN getData(void* buf, int len) = 0;
    virtual NETDATALEN sendData(void* buf, int len, struct ipPort _destIpPort) = 0;
    virtual NETDATALEN sendDataByMap(void* buf, int len, unsigned int destObjIndex);

    virtual ERROR_NUM setPrivateData(void *privateData_);
    virtual void* getPrivateData(void);
    virtual ERROR_NUM registerDestIpMap(std::string ip_, unsigned short port_, int destObjIndex);

protected:
    virtual void serverWorkThread() = 0;
};

#endif
