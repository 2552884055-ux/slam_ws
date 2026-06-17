#ifndef __TCP_UDP__HPP
#define __TCP_UDP__HPP

#include "network/netWorkBase.hpp"

class udpTool : public netWorkBase {
protected:
    void serverWorkThread() override;
public:
    udpTool(struct ipPort _sourceIpPort, MODULE_TYPE moduleId);
    ~udpTool();
    ERROR_NUM createNet(int _bufSize) override;
    ERROR_NUM startRecvThread(callBackFunc _callBack) override;
    ERROR_NUM endRecvThread() override;
    ERROR_NUM destroryNet() override;

    NETDATALEN getData(void* buf, int len) override;
    NETDATALEN sendData(void* buf, int len, struct ipPort _destIpPort) override;
};

class tcpTool : public netWorkBase {
private:
    bool connectedFlag_ = false;
    struct ipPort destIpPort_;
    int connected_fd = -1;
protected:
    void serverWorkThread() override;

public:
    tcpTool(struct ipPort _sourceIpPort, MODULE_TYPE moduleId);
    ~tcpTool();
    ERROR_NUM createNet(int _bufSize) override;
    ERROR_NUM startRecvThread(callBackFunc _callBack) override;
    ERROR_NUM endRecvThread() override;
    ERROR_NUM destroryNet() override;

    NETDATALEN getData(void* buf, int len) override;
    NETDATALEN sendData(void* buf, int len, struct ipPort _destIpPort) override;
};

#endif
