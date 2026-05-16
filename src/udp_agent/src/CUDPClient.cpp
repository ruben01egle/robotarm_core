#include "udp_agent/CUDPClient.hpp"
#include <iostream>

CUDPClient::CUDPClient(MsgCallback msgCallback, LogFunc logFunc)
    : mMsgHandler(msgCallback), logger(logFunc) {}

CUDPClient::~CUDPClient() {
    stop();
}

bool CUDPClient::begin(const std::string& stm32Ip, uint16_t port) 
{
    mSocketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (mSocketFd < 0) {
        logger("Failed to create socket");
        return false;
    }

    struct sockaddr_in localAddr{};
    std::memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons(port);

    if (bind(mSocketFd, reinterpret_cast<struct sockaddr*>(&localAddr), sizeof(localAddr)) < 0) {
        logger("Failed to bind socket to port " + std::to_string(port));
        close(mSocketFd);
        mSocketFd = -1;
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    setsockopt(mSocketFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::memset(&mStm32Addr, 0, sizeof(mStm32Addr));
    mStm32Addr.sin_family = AF_INET;
    mStm32Addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, stm32Ip.c_str(), &mStm32Addr.sin_addr) <= 0) {
        logger("Invalid STM32 IP Address");
        close(mSocketFd);
        mSocketFd = -1;
        return false;
    }

    mRunning = true;
    mReceiveThread = std::thread(&CUDPClient::receiveWorker, this);

    logger("UDP Client initialized. Target: " + stm32Ip + ":" + std::to_string(port));
    return true;
}

void CUDPClient::stop() {
    mRunning = false;
    if (mReceiveThread.joinable()) {
        mReceiveThread.join();
    }
    if (mSocketFd >= 0) {
        close(mSocketFd);
        mSocketFd = -1;
    }
    logger("UDP Client stopped");
}

bool CUDPClient::sendDiscovery() {
    const char discoveryMsg[] = "ROS2"; 
    logger("Sending discovery packet to STM32...");
    
    bool success = sendRawMessage(reinterpret_cast<const uint8_t*>(discoveryMsg), sizeof(discoveryMsg) - 1);
    
    if (!success) {
        logger("Failed to send discovery packet");
    }
    return success;
}

bool CUDPClient::sendRawMessage(const uint8_t* pData, uint32_t pDataLength) {
    if (mSocketFd < 0) {
        logger("Socket not initialized");
        return false;
    }

    ssize_t sentBytes = sendto(mSocketFd, pData, pDataLength, 0,
                              reinterpret_cast<struct sockaddr*>(&mStm32Addr), sizeof(mStm32Addr));

    if (sentBytes < 0) {
        logger("Failed to send UDP packet");
        return false;
    }

    return true;
}

void CUDPClient::receiveWorker() {
    uint8_t buffer[MAX_PACKET_SIZE];
    sockaddr_in fromAddr{};
    socklen_t fromLen = sizeof(fromAddr);

    while (mRunning) {
        ssize_t receivedBytes = recvfrom(mSocketFd, buffer, MAX_PACKET_SIZE, 0,
                                         reinterpret_cast<struct sockaddr*>(&fromAddr), &fromLen);

        if (receivedBytes < 0) {
            continue; 
        }

        if (fromAddr.sin_addr.s_addr != mStm32Addr.sin_addr.s_addr) {
            logger("Received packet from unknown source, dropping");
            continue;
        }

        if (static_cast<size_t>(receivedBytes) < sizeof(PacketHeader)) {
            logger("Received packet too small for header");
            continue;
        }

        PacketHeader header;
        std::memcpy(&header, buffer, sizeof(PacketHeader));

        if (header.magic != UDP_MAGIC) {
            logger("Invalid magic bytes from STM32");
            continue;
        }

        if (static_cast<size_t>(receivedBytes) < (sizeof(PacketHeader) + header.payload_size)) {
            logger("Invalid packet length (payload mismatch)");
            continue;
        }

        const uint8_t* payloadPtr = buffer + sizeof(PacketHeader);
        mMsgHandler(header.msg_type, payloadPtr, header.payload_size);
    }
}