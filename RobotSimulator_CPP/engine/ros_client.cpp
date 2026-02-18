#include "ros_client.h"
#include <iostream>
#include <spdlog/spdlog.h>

RosClient::RosClient(const std::string& url) : m_url(url), m_connected(false) {
    m_latestJoints.resize(6, 0.0f);
    m_referenceJoints.resize(6, 0.0f);
    m_jointNames = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    
    m_webSocket.setUrl(m_url);
    m_webSocket.setOnMessageCallback([this](const ix::WebSocketMessagePtr& msg) {
        OnMessage(msg);
    });
}

RosClient::~RosClient() {
    Disconnect();
}

void RosClient::Connect() {
    m_webSocket.start();
}

void RosClient::Disconnect() {
    m_webSocket.stop();
}

bool RosClient::IsConnected() const {
    return m_webSocket.getReadyState() == ix::ReadyState::Open;
}

void RosClient::Subscribe(const std::string& topic, const std::string& type, std::function<void(const nlohmann::json&)> callback) {
    if (!IsConnected()) return;

    nlohmann::json msg;
    msg["op"] = "subscribe";
    msg["topic"] = topic;
    msg["type"] = type;
    m_webSocket.send(msg.dump());
}

void RosClient::SubscribeReference(const std::string& topic) {
    m_refTopic = topic;
    if (m_connected && !m_refTopic.empty()) {
         Subscribe(m_refTopic, "sensor_msgs/msg/JointState", nullptr);
    }
}

void RosClient::OnMessage(const ix::WebSocketMessagePtr& msg) {
    if (msg->type == ix::WebSocketMessageType::Message) {
        try {
            auto j = nlohmann::json::parse(msg->str);
            if (j.contains("op") && j["op"] == "publish") {
                if (j.contains("topic")) {
                    std::string topic = j["topic"];
                    
                    // Parse Joint State (Primary OR Reference)
                    if (topic == "/joint_states" || (!m_refTopic.empty() && topic == m_refTopic)) {
                        
                        auto& msg_data = j["msg"];
                        if (msg_data.contains("name") && msg_data.contains("position")) {
                            std::lock_guard<std::mutex> lock(m_mutex);
                            auto& names = msg_data["name"];
                            auto& pos = msg_data["position"];
                            
                            // Pointer to target vector
                            std::vector<float>* targetVec = (topic == m_refTopic) ? &m_referenceJoints : &m_latestJoints;

                            // Robust Name-Based Mapping
                            for (size_t i = 0; i < names.size() && i < pos.size(); i++) {
                                std::string jointName = names[i];
                                // Find which index this corresponds to in our model
                                for(size_t k=0; k<m_jointNames.size(); ++k) {
                                    if(m_jointNames[k] == jointName) {
                                        (*targetVec)[k] = pos[i];
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            spdlog::error("JSON Parse Error: {}", e.what());
        }
    } else if (msg->type == ix::WebSocketMessageType::Open) {
        spdlog::info("Connected to ROS Bridge");
        m_connected = true;
        // Auto-subscribe to joint states
        Subscribe("/joint_states", "sensor_msgs/msg/JointState", nullptr);
        
        // Subscribe to reference if set
        if (!m_refTopic.empty()) {
            Subscribe(m_refTopic, "sensor_msgs/msg/JointState", nullptr);
        }
        
    } else if (msg->type == ix::WebSocketMessageType::Close) {
        spdlog::info("Disconnected from ROS Bridge");
        m_connected = false;
    }
}
