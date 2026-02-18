#pragma once

#include <string>
#include <functional>
#include <vector>
#include <thread>
#include <mutex>
#include <ixwebsocket/IXWebSocket.h>
#include <nlohmann/json.hpp>

class RosClient {
public:
    RosClient(const std::string& url = "ws://localhost:9090");
    ~RosClient();

    void Connect();
    void Disconnect();
    bool IsConnected() const;

    void PublishJoints(const std::vector<float>& joints);
    void Subscribe(const std::string& topic, const std::string& type, std::function<void(const nlohmann::json&)> callback);

    // Specific for our robot
    std::vector<float> GetLatestJointState() const { return m_latestJoints; }
    std::vector<float> GetReferenceJointState() const { return m_referenceJoints; }

    void SubscribeReference(const std::string& topic);

private:
    ix::WebSocket m_webSocket;
    std::string m_url;
    std::string m_refTopic;
    bool m_connected;
    
    std::vector<float> m_latestJoints;
    std::vector<float> m_referenceJoints; // From Gazebo/Planner
    std::vector<std::string> m_jointNames;
    mutable std::mutex m_mutex;
    
    void OnMessage(const ix::WebSocketMessagePtr& msg);
};
