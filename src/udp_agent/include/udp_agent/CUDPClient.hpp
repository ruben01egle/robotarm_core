#pragma once

#include <string>
#include <vector>
#include <functional>
#include <cstring>
#include <atomic>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "protocol/ProtocolCommon.hpp"

class CUDPClient {
public:
    using LogFunc = std::function<void(const std::string&)>;
    using MsgCallback = std::function<void(MessageType type, const uint8_t* data, uint32_t length)>;

    CUDPClient(MsgCallback msgCallback, LogFunc logFunc);
    ~CUDPClient();

    bool begin(const std::string& stm32Ip, uint16_t port);
    void stop();

    bool sendDiscovery();

    template <typename T>
    bool sendMessage(MessageType type, const T& msg) {
        uint32_t payloadSize = msg.get_serialized_size();
        uint32_t totalSize = sizeof(PacketHeader) + payloadSize;

        std::vector<uint8_t> buffer(totalSize);

        PacketHeader header;
        header.magic = UDP_MAGIC;
        header.msg_type = type;
        header.msg_id = 0;
        header.payload_size = payloadSize;

        std::memcpy(buffer.data(), &header, sizeof(PacketHeader));
        msg.serialize(buffer.data() + sizeof(PacketHeader));

        return sendRawMessage(buffer.data(), totalSize);
    }

    bool sendRawMessage(const uint8_t* pData, uint32_t pDataLength);

private:
    void receiveWorker();

    MsgCallback mMsgHandler;
    LogFunc logger;

    int mSocketFd = -1;
    sockaddr_in mStm32Addr{};
    
    std::atomic<bool> mRunning{false};
    std::thread mReceiveThread;

    static constexpr size_t MAX_PACKET_SIZE = 1024;
};