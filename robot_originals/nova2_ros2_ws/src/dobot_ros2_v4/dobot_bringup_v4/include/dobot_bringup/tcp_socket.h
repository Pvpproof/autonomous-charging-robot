/**
 ***********************************************************************************************************************
 *
 * @author qiufengwu TX_F
 * @date   2024/01/23
 * @date   2025/02/27
 *
 ***********************************************************************************************************************
 */

#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>
#include <chrono>
#include <stdint.h>
#include <cerrno>
#include <cstring>
#include <utility>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>

/**
 * TcpClientException
 */
class TcpClientException : public std::logic_error
{
public:
    TcpClientException(const std::string &what) : std::logic_error(what)
    {
    }
};

/**
 * TcpClient
 */
class TcpClient
{
private:
    int fd_;
    uint16_t port_;
    std::string ip_;
    std::atomic<bool> is_connected_;
    std::timed_mutex dashboard_mutex_;
    std::atomic<bool> dashboard_fault_{false};

public:
    /**
     * Ctor
     */
    TcpClient(std::string ip, uint16_t port);

    /**
     * Dtor
     */
    ~TcpClient();

    /**
     * close
     */
    void close();

    /**
     * connect
     */
    void connect();

    /**
     * disConnect
     */
    void disConnect();

    /**
     * isConnect
     */
    bool isConnect() const;
    bool dashboardFault() const { return dashboard_fault_.load(); }

    /**
     * write
     */
    void tcpSend(const void *buf, uint32_t len);

    /**
     * tcpRecv
     */
    bool tcpRecv(void *buf, uint32_t len, uint32_t &has_read, uint32_t timeout);

    // One owner, one response. A transport fault latches until process replacement.
    // timeout_ms bounds transport work, not controller motion or physical stopping.
    std::string dashboardExchange(const std::string &command, uint32_t timeout_ms=2000);

    std::string toString();
};

#endif // TCPSOCKET_H
