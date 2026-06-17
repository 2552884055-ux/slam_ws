#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <cstdint>
#include <vector>
#include <cstring>
#include "network/frame.hpp"

// 通用帧数据结构
struct commonFrameData {
    uint32_t hasData;
    uint32_t data1;
    uint32_t data2;
    float data3;
    float data4;
};

// 辅助访问函数：获取frame的subObj
inline uint32_t get_frame_subobj(const frame* pf) {
    if (pf == nullptr || pf->len_data < sizeof(mydata)) {
        return 0;
    }
    const mydata* pmeta = reinterpret_cast<const mydata*>(pf->data);
    return static_cast<uint32_t>(pmeta->sub_obj);
}

// 辅助访问函数：获取frame的数据部分
inline bool get_frame_data(const frame* pf, commonFrameData& out) {
    if (pf == nullptr || pf->len_data < (sizeof(mydata) + sizeof(commonFrameData))) {
        return false;
    }
    const commonFrameData* pdata = reinterpret_cast<const commonFrameData*>(pf->data + sizeof(mydata));
    out = *pdata;
    return true;
}

// 辅助访问函数：创建frame的辅助结构（用于方便创建frame）
struct FrameBuilder {
    unsigned int source;
    unsigned int dest;
    unsigned int type;
    uint32_t subObj;
    commonFrameData data;
    
    // 构建完整的frame到buffer中
    void build_to_buffer(std::vector<char>& buffer) const {
        struct WirePayload {
            mydata meta;
            commonFrameData data;
        } __attribute__((packed));
        
        WirePayload payload{};
        payload.meta.seq = 0;
        payload.meta.eof = 1;
        payload.meta.sub_obj = static_cast<int>(subObj);
        payload.meta.type = 0;
        payload.data = data;
        
        const unsigned int data_len = sizeof(WirePayload);
        const unsigned int head_len = sizeof(frame);
        const unsigned int total_len = head_len + data_len;
        
        buffer.resize(total_len);
        frame* pf = reinterpret_cast<frame*>(buffer.data());
        pf->source = source;
        pf->dest = dest;
        pf->type = type;
        pf->len_data = data_len;
        pf->checksum = 0;
        pf->reserve = 0;
        memcpy(pf->data, &payload, data_len);
    }
};

#endif // DATA_TYPE_H
