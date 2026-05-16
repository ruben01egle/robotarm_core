#include "udp_agent/CUDPAgentNode.hpp"
#include "udp_agent/CUDPAgentNode.hpp"

CUDPAgent::CUDPAgent(): Node("udp_agent_node")
{
    mUDPClient = std::make_unique<CUDPClient>(
        [this](MessageType type, const uint8_t* data, uint32_t length) {
            this->rawUDPDataCB(type, data, length);}, 
        [this](const std::string& msg){
            if (msg.find("Error") != std::string::npos || msg.find("failed") != std::string::npos) {
                RCLCPP_ERROR(this->get_logger(), "[UDP-Client] %s", msg.c_str());
            } else if (msg.find("WARN") != std::string::npos) {
                RCLCPP_WARN(this->get_logger(), "[UDP-Client] %s", msg.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "[UDP-Client] %s", msg.c_str());
            }
        });
    mUDPClient->begin("192.168.0.200", 6666);           // TODO: magic number

    mSystemStateSub = this->create_subscription<interface::msg::SystemState>(
        "system_state", 1,
        [this](const interface::msg::SystemState::SharedPtr msg) {
            this->stateCB(msg);
        }
    );

    mHardwareCMDSub = this->create_subscription<interface::msg::HardwareCommand>(
        "hardware/command", 5,
        [this](const interface::msg::HardwareCommand::SharedPtr msg) {
            this->udpSend(msg);
        }
    );

    mHeartbeatQuerySub = this->create_subscription<interface::msg::HeartbeatQuery>(
        "heartbeat/query", 1,
        [this](const interface::msg::HeartbeatQuery::SharedPtr msg) {
            this->udpSend(msg);
        }
    );

    rclcpp::QoS traj_best_effort_qos(30);
    traj_best_effort_qos.best_effort();
    mTrajectorySub = this->create_subscription<interface::msg::TrajectoryBatch>(
        "trajectory/data",
        traj_best_effort_qos,
        [this](const interface::msg::TrajectoryBatch::SharedPtr msg) {
            this->udpSend(msg);
        }
    );

    mHardwareFeedbackPub = this->create_publisher<interface::msg::HardwareFeedback>(
        "hardware/feedback", 
        5
    );

    mHeartbeatPub = this->create_publisher<interface::msg::Heartbeat>(
        "heartbeat/response", 
        1
    );

    mTrajectoryFeedbackPub = this->create_publisher<interface::msg::TrajectoryFeedback>(
        "trajectory/feedback", 
        5
    );

    rclcpp::QoS tel_best_effort_qos(30);
    tel_best_effort_qos.best_effort();
    mTelemetryPub = this->create_publisher<interface::msg::TelemetryBatch>(
        "telemetry", 
        tel_best_effort_qos
    );
}

void CUDPAgent::rawUDPDataCB(MessageType type, const uint8_t *data, uint32_t length)
{
    switch (type) {
        case MessageType::HARDWARE_FEEDBACK:   unpackAndPublish<HardwareFeedback>(data, length); break;
        case MessageType::HEARTBEAT:           unpackAndPublish<Heartbeat>(data, length); break;
        case MessageType::TRAJECTORY_FEEDBACK: unpackAndPublish<TrajectoryFeedback>(data, length); break;
        case MessageType::TELEMETRY_BATCH:     unpackAndPublish<TelemetryBatch>(data, length); break;
        default:
            RCLCPP_WARN(get_logger(), "Unknown MessageType: %d", static_cast<int>(type));
            break;
    }
}


void CUDPAgent::publish(const HardwareFeedback &msg)
{
    interface::msg::HardwareFeedback msgROS;

    msgROS.action = msg.action;
    msgROS.command_id = msg.command_id;
    msgROS.current_state = msg.current_state;
    msgROS.success = msg.success;

    mHardwareFeedbackPub->publish(msgROS);
}

void CUDPAgent::publish(const Heartbeat &msg)
{
    interface::msg::Heartbeat msgROS;

    std::copy(
        std::begin(msg.node_name),
        std::end(msg.node_name),
        msgROS.node_name.begin()
    );
    msgROS.original_sent_timestamp.sec = msg.original_sent_timestamp.sec;
    msgROS.original_sent_timestamp.nanosec = msg.original_sent_timestamp.nsec;
    msgROS.query_id = msg.query_id;
   
    mHeartbeatPub->publish(msgROS);
}

void CUDPAgent::publish(const TrajectoryFeedback &msg)
{
    interface::msg::TrajectoryFeedback msgROS;

    msgROS.current_hardware_idx = msg.current_hardware_idx;
    msgROS.trajectory_id = msg.trajectory_id;
    msgROS.trajectory_status = msg.trajectory_status;
    msgROS.request_next_count = msg.request_next_count;

    mTrajectoryFeedbackPub->publish(msgROS);
}

void CUDPAgent::publish(const TelemetryBatch &msg)
{
    interface::msg::TelemetryBatch msgROS;

    msgROS.packet_num = msg.packet_num;
    msgROS.trajectory_id = msg.trajectory_id;

    uint32_t valid_elements = std::min(msg.data_count, msg.MAX_DATA_SIZE);
    msgROS.data.resize(valid_elements);

    auto mapAxis = [](interface::msg::AxisData& rosAxis, const AxisData& udpAxis) {
        rosAxis.position = udpAxis.position;
        rosAxis.velocity = udpAxis.velocity;
        rosAxis.torque   = udpAxis.torque;
    };

    for (uint32_t i = 0; i < valid_elements; ++i)
    {
        const auto &udpFrame = msg.data[i];
        auto &rosFrame = msgROS.data[i];

        rosFrame.time_us = udpFrame.time_us;
        rosFrame.idx     = udpFrame.idx;
        rosFrame.gripper = udpFrame.gripper;

        mapAxis(rosFrame.axis1, udpFrame.axis1);
        mapAxis(rosFrame.axis2, udpFrame.axis2);
        mapAxis(rosFrame.axis3, udpFrame.axis3);
        mapAxis(rosFrame.axis4, udpFrame.axis4);
        mapAxis(rosFrame.axis5, udpFrame.axis5);
        mapAxis(rosFrame.axis6, udpFrame.axis6);
    }

    RCLCPP_INFO(get_logger(), "Sent trajectory");
    mTelemetryPub->publish(msgROS);
}

void CUDPAgent::udpSend(const interface::msg::HardwareCommand::SharedPtr msg)
{
    HardwareCommand msgUDP;

    msgUDP.command_id = msg->command_id;
    msgUDP.action     = msg->action;

    mUDPClient->sendMessage(MessageType::HARDWARE_COMMAND, msgUDP);
}

void CUDPAgent::udpSend(const interface::msg::HeartbeatQuery::SharedPtr msg)
{
    HeartbeatQuery msgUDP;

    msgUDP.query_id = msg->query_id;
    
    msgUDP.sent_timestamp.sec  = msg->sent_timestamp.sec;
    msgUDP.sent_timestamp.nsec = msg->sent_timestamp.nanosec;

    mUDPClient->sendMessage(MessageType::HEARTBEAT_QUERY, msgUDP);
}

void CUDPAgent::udpSend(const interface::msg::TrajectoryBatch::SharedPtr msg)
{
    TrajectoryBatch msgUDP;

    msgUDP.packet_num        = msg->packet_num;
    msgUDP.trajectory_id     = msg->trajectory_id;
    msgUDP.trajectory_status = msg->trajectory_status;
    
    uint32_t valid_elements = std::min(static_cast<uint32_t>(msg->data.size()), msgUDP.MAX_DATA_SIZE);
    msgUDP.data_count = valid_elements;

    auto mapAxisToUDP = [](AxisData& udpAxis, const interface::msg::AxisData& rosAxis) {
        udpAxis.position = rosAxis.position;
        udpAxis.velocity = rosAxis.velocity;
        udpAxis.torque   = rosAxis.torque;
    };

    for (uint32_t i = 0; i < valid_elements; ++i)
    {
        const auto &rosFrame = msg->data[i];
        auto &udpFrame = msgUDP.data[i];

        udpFrame.idx     = rosFrame.idx;
        udpFrame.gripper = rosFrame.gripper;

        mapAxisToUDP(udpFrame.axis1, rosFrame.axis1);
        mapAxisToUDP(udpFrame.axis2, rosFrame.axis2);
        mapAxisToUDP(udpFrame.axis3, rosFrame.axis3);
        mapAxisToUDP(udpFrame.axis4, rosFrame.axis4);
        mapAxisToUDP(udpFrame.axis5, rosFrame.axis5);
        mapAxisToUDP(udpFrame.axis6, rosFrame.axis6);
    }

    RCLCPP_INFO(get_logger(), "Sending trajectory batch (ID: %d, Packet: %d, Status: %d, Elements: %d)", 
                msgUDP.trajectory_id, msgUDP.packet_num, static_cast<int>(msgUDP.trajectory_status), msgUDP.data_count);

    mUDPClient->sendMessage(MessageType::TRAJECTORY_BATCH, msgUDP);
}

void CUDPAgent::stateCB(const interface::msg::SystemState::SharedPtr msg)
{
    static uint32_t counter = 0;
    counter++;
    if (msg->state == interface::msg::SystemState::IDLE && counter % 10 == 0) {
        mUDPClient->sendDiscovery();
    }
}
