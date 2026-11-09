#include <dobot_bringup/command.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
CRCommanderRos2::CRCommanderRos2(const std::string &ip)
    : current_joint_{}, tool_vector_{}, is_running_(false)
{
    is_running_ = false;
    real_time_data_ = std::make_shared<RealTimeData>();
    source_session_ = std::to_string(source_monotonic_ns());
    real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
    dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
}

CRCommanderRos2::~CRCommanderRos2()
{
    is_running_ = false;
    thread_->join();
}

void CRCommanderRos2::getCurrentJointStatus(double *joint)
{
    mutex_.lock();
    memcpy(joint, current_joint_, sizeof(current_joint_));
    mutex_.unlock();
}

void CRCommanderRos2::getToolVectorActual(double *val)
{
    mutex_.lock();
    memcpy(val, tool_vector_, sizeof(tool_vector_));
    mutex_.unlock();
}

void CRCommanderRos2::recvTask()
{
    uint32_t has_read;
    while (is_running_)
    {
        if (real_time_tcp_->isConnect())
        {
            try
            {
                RealTimeData incoming{};
                uint8_t *tmpData = reinterpret_cast<uint8_t *>(&incoming);
                if (real_time_tcp_->tcpRecv(tmpData, sizeof(RealTimeData), has_read, 5000))
                {
                    const auto now_ns=source_monotonic_ns();
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (has_read!=sizeof(RealTimeData)) {
                        source_state_.valid=false; source_state_.fault="incomplete controller frame";
                        continue;
                    }
                    if (!source_state_.accept(incoming.len,incoming.test_value,incoming.controller_timer,now_ns))
                        continue;
                    *real_time_data_=incoming;
                    for (uint32_t i=0;i<6;i++) current_joint_[i]=deg2Rad(incoming.q_actual[i]);
                    memcpy(tool_vector_,incoming.tool_vector_actual,sizeof(tool_vector_));
                }
                else
                {
                    if (has_read) real_time_tcp_->disConnect(); // Discard partial stream by reconnect, never parse a shifted packet.
                    RCLCPP_WARN(rclcpp::get_logger("CRCommanderRos2"), "tcp recv timeout");
                }
            }
            catch (const TcpClientException &err)
            {
                real_time_tcp_->disConnect();
                { std::lock_guard<std::mutex> lock(mutex_);
                  source_state_.valid=false;source_state_.fault="feedback transport disconnected"; }
                RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "tcp recv error: %s", err.what());
            }
        }
        else
        {
            try
            {
                real_time_tcp_->connect();
                { std::lock_guard<std::mutex> lock(mutex_);
                  source_session_=std::to_string(source_monotonic_ns());
                  source_state_=SourceFeedbackState{}; }
            }
            catch (const TcpClientException &err)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "tcp recv Error: %s", err.what());
                sleep(3);
            }
        }

        // A latched command-channel fault must not starve the independent feedback receiver.
        if (!dash_board_tcp_->isConnect() && !dash_board_tcp_->dashboardFault())
        {
            try
            {
                dash_board_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "tcp recv ERROR: %s", err.what());
                sleep(3);
            }
        }
    }
}

void CRCommanderRos2::init()
{
    try
    {
        is_running_ = true;
        thread_ = std::unique_ptr<std::thread>(new std::thread(&CRCommanderRos2::recvTask, this));
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "Commander: %s", err.what());
    }
}
void CRCommanderRos2::doTcpCmd(std::shared_ptr<TcpClient> &tcp, const char *cmd,
                               int32_t &err_id, std::vector<std::string> &result)
{
    // dashboardExchange validates framing/ErrorID/command association under one owner.
    // Exceptions propagate to callRosService; never leave the default ROS res=0 on failure.
    err_id=-1;
    const std::string response=tcp->dashboardExchange(cmd);
    if (std::string(cmd)=="Stop()")
        RCLCPP_WARN(rclcpp::get_logger("CRCommanderRos2"), "STOP_NATIVE_RESPONSE response=%s", response.c_str());
    const auto open=response.find(",{"),close=response.find("},",open+2);
    err_id=std::stoi(response.substr(0,open));
    result.clear();result.push_back(response.substr(open+1,close-open));
}

void CRCommanderRos2::doTcpCmd_f(std::shared_ptr<TcpClient> &tcp, const char *cmd,
                                 int32_t &err_id, std::string &mode_id,
                                 std::vector<std::string> &result)
{
    mode_id.clear();
    doTcpCmd(tcp,cmd,err_id,result);
    mode_id=result.front();
}

bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const std::exception &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "DASHBOARD_LOCAL_FAILURE command=%s cause=%s", cmd.c_str(), err.what());
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService_f(const std::string cmd, int32_t &err_id,std::string &mode_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd_f(this->dash_board_tcp_, cmd.c_str(), err_id,mode_id, result_);
        return true;
    }
    catch (const std::exception &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "callRosService_f: %s", err.what());
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id, std::vector<std::string> &result_)
{
    try
    {
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const std::exception &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "DASHBOARD_LOCAL_FAILURE command=%s cause=%s", cmd.c_str(), err.what());
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::isEnable() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return real_time_data_->robot_mode == 5;
}

bool CRCommanderRos2::isConnected() const
{
    bool dash_connected = dash_board_tcp_->isConnect();
    bool real_time_connected = real_time_tcp_->isConnect();
    
    // 仅在连接断开时打印日志，帮助分析问题
    static bool last_connected = true;
    bool current_connected = dash_connected && real_time_connected;
    
    if (!current_connected && last_connected) {
        RCLCPP_WARN(rclcpp::get_logger("CRCommanderRos2"), 
                    "Robot disconnected - Dashboard: %s, Real-time: %s", 
                    dash_connected ? "connected" : "disconnected", 
                    real_time_connected ? "connected" : "disconnected");
    }
    last_connected = current_connected;
    
    return current_connected;
}

uint16_t CRCommanderRos2::getRobotMode() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return real_time_data_->robot_mode;
}

std::shared_ptr<RealTimeData> CRCommanderRos2::getRealData() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return std::make_shared<RealTimeData>(*real_time_data_);
}

FeedbackSnapshot CRCommanderRos2::getFeedbackSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return FeedbackSnapshot{*real_time_data_,source_state_,source_session_};
}
