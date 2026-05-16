#pragma once

#include "rclcpp/rclcpp.hpp"

#include "udp_agent/CUDPClient.hpp"

#include "interface/msg/system_state.hpp"
#include "interface/msg/hardware_command.hpp"
#include "interface/msg/hardware_feedback.hpp"
#include "interface/msg/heartbeat.hpp"
#include "interface/msg/heartbeat_query.hpp"
#include "interface/msg/trajectory_batch.hpp"
#include "interface/msg/trajectory_feedback.hpp"
#include "interface/msg/telemetry_batch.hpp"

#include "protocol/HardwareCommand.hpp"
#include "protocol/HardwareFeedback.hpp"
#include "protocol/Heartbeat.hpp"
#include "protocol/HeartbeatQuery.hpp"
#include "protocol/TrajectoryBatch.hpp"
#include "protocol/TrajectoryFeedback.hpp"
#include "protocol/TelemetryBatch.hpp"


class CUDPAgent: public rclcpp::Node
{
public:
    CUDPAgent();

private:
    void rawUDPDataCB(MessageType type, const uint8_t* data, uint32_t length);

    void publish(const HardwareFeedback& msg);
    void publish(const Heartbeat& msg);
    void publish(const TrajectoryFeedback& msg);
    void publish(const TelemetryBatch& msg);

    void udpSend(const interface::msg::HardwareCommand::SharedPtr msg);
    void udpSend(const interface::msg::HeartbeatQuery::SharedPtr msg);
    void udpSend(const interface::msg::TrajectoryBatch::SharedPtr msg);

    void stateCB(const interface::msg::SystemState::SharedPtr msg);

    template <typename T>
    bool unpackAndPublish(const uint8_t* data, uint32_t length) {
        T msgUDP;
        if (!msgUDP.deserialize(data, length)) {
            RCLCPP_ERROR(get_logger(), "Deserialization error");
            return false;
        }
        publish(msgUDP);
        return true;
    }

private:
    std::unique_ptr<CUDPClient> mUDPClient;

    rclcpp::Subscription<interface::msg::SystemState>::SharedPtr mSystemStateSub;

    rclcpp::Subscription<interface::msg::HardwareCommand>::SharedPtr mHardwareCMDSub;
    rclcpp::Subscription<interface::msg::HeartbeatQuery>::SharedPtr mHeartbeatQuerySub;
    rclcpp::Subscription<interface::msg::TrajectoryBatch>::SharedPtr mTrajectorySub;
    
    rclcpp::Publisher<interface::msg::HardwareFeedback>::SharedPtr mHardwareFeedbackPub;
    rclcpp::Publisher<interface::msg::Heartbeat>::SharedPtr mHeartbeatPub;
    rclcpp::Publisher<interface::msg::TrajectoryFeedback>::SharedPtr mTrajectoryFeedbackPub;
    rclcpp::Publisher<interface::msg::TelemetryBatch>::SharedPtr mTelemetryPub;
};