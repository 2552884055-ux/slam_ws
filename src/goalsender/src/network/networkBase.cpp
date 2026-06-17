#include <string.h>
#include "network/netWorkBase.hpp"
#include "network/errorNum.hpp"
#include "network/moduleObj.hpp"
#include "network/frame.hpp"

netWorkBase::netWorkBase(struct ipPort _sourceIpPort, MODULE_TYPE moduleId) : sourceIpPort_{_sourceIpPort}, moduleId{moduleId}{};

netWorkBase::~netWorkBase()
{
}

ERROR_NUM netWorkBase::setPrivateData(void *privateData_)
{
    if(privateData_)
    {
        this->privateData = privateData_;
        return ERROR_NUM::SUCCESS;
    }
    return ERROR_NUM::FAILED;
}

void* netWorkBase::getPrivateData(void)
{
    return this->privateData;
}

ERROR_NUM netWorkBase::registerDestIpMap(std::string ip_, unsigned short port_, int destObjIndex)
{
    if(this->ipMapTable.count(destObjIndex) > 0)
    {
        return ERROR_NUM::FAILED;
    }
    struct ipPort* key = new ipPort{.ip = ip_, .port = port_};
    this->ipMapTable.insert({destObjIndex, key});
    return ERROR_NUM::SUCCESS;
}

NETDATALEN netWorkBase::sendDataByMap(void* buf, int len, unsigned int destObjIndex)
{
    if(ipMapTable.count(destObjIndex) > 0)
    {
        struct frame* frameData = (struct frame*)malloc(sizeof(struct frame) + sizeof(char) * len);
        frameData->source = this->moduleId;
        frameData->dest = destObjIndex;
        frameData->len_data = len;
        frameData->type = 0;
        frameData->reserve = 0;
        frameData->checksum = 0;
        memcpy(frameData->data, buf, len);
        return this->sendData(frameData, len + sizeof(struct frame), *(this->ipMapTable[destObjIndex]));
    }
    return 0;
}
