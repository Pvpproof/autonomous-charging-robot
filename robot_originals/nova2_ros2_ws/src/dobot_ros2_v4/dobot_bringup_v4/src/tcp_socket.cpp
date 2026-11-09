#include <dobot_bringup/tcp_socket.h>

TcpClient::TcpClient(std::string ip, uint16_t port) : fd_(-1), port_(port), ip_(std::move(ip)), is_connected_(false)
{
}

TcpClient::~TcpClient()
{
    close();
}

void TcpClient::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        is_connected_ = false;
        fd_ = -1;
    }
}

void TcpClient::connect()
{
    if (dashboard_fault_) throw TcpClientException("Dashboard fault latched; manual inspection/restart required");
    if (fd_ < 0)
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0)
            throw TcpClientException(toString() + std::string(" socket : ") + strerror(errno));
    }

    sockaddr_in addr = {};

    memset(&addr, 0, sizeof(addr));
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);

    if (::connect(fd_, (sockaddr *)&addr, sizeof(addr)) < 0)
        throw TcpClientException(toString() + std::string(" connect : ") + strerror(errno));
    is_connected_ = true;

    RCLCPP_INFO(rclcpp::get_logger("TcpClient"), "connect successfully: %s", toString().c_str());
}

std::string TcpClient::dashboardExchange(const std::string &command, uint32_t timeout_ms)
{
    using Clock=std::chrono::steady_clock;
    if (port_!=29999 || command.empty() || !timeout_ms)
        throw TcpClientException("invalid Dashboard exchange arguments");
    const auto deadline=Clock::now()+std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::timed_mutex> owner(dashboard_mutex_,std::defer_lock);
    if (!owner.try_lock_until(deadline)) throw TcpClientException("Dashboard owner deadline exceeded");
    try {
        if (dashboard_fault_ || !is_connected_) throw TcpClientException("Dashboard unavailable/fault latched");
        auto wait_fd=[&](bool writing) {
            for (;;) {
                auto remaining=std::chrono::duration_cast<std::chrono::microseconds>(deadline-Clock::now()).count();
                if (remaining<=0) throw TcpClientException("Dashboard total response deadline exceeded");
                fd_set fds;FD_ZERO(&fds);FD_SET(fd_,&fds);
                timeval tv{static_cast<long>(remaining/1000000),static_cast<long>(remaining%1000000)};
                int result=::select(fd_+1,writing?nullptr:&fds,writing?&fds:nullptr,nullptr,&tv);
                if (result<0 && errno==EINTR) continue;
                if (result<=0) throw TcpClientException("Dashboard wait failed/deadline exceeded");
                return;
            }
        };
        size_t sent=0;
        while (sent<command.size()) {
            wait_fd(true);
            ssize_t count=::send(fd_,command.data()+sent,command.size()-sent,MSG_NOSIGNAL|MSG_DONTWAIT);
            if (count<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR)) continue;
            if (count<=0) throw TcpClientException("Dashboard send failed");
            sent+=static_cast<size_t>(count);
        }
        std::string response;
        while (response.size()<65536) {
            wait_fd(false);char buf[1024];ssize_t count=::recv(fd_,buf,sizeof(buf),MSG_DONTWAIT);
            if (count<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR)) continue;
            if (count<=0) throw TcpClientException("Dashboard disconnected during response");
            response.append(buf,static_cast<size_t>(count));
            auto end=response.find(';');
            if (end!=std::string::npos) {
                if (end+1!=response.size()) throw TcpClientException("unexpected trailing/coalesced Dashboard response");
                auto open=response.find(",{");auto close=response.find("},",open+2);
                if (open==std::string::npos || close==std::string::npos || close<open+1 || response.find('\0')!=std::string::npos)
                    throw TcpClientException("malformed Dashboard response");
                size_t parsed=0;std::string code=response.substr(0,open);
                std::stoi(code,&parsed);
                if (parsed!=code.size()) throw TcpClientException("invalid Dashboard ErrorID");
                auto method=command.substr(0,command.find('('));
                auto echo=response.substr(close+2);
                if (method.empty() || echo.compare(0,method.size()+1,method+"(")!=0)
                    throw TcpClientException("Dashboard response method mismatch");
                return response;
            }
        }
        throw TcpClientException("Dashboard response size exceeded");
    } catch (...) {
        dashboard_fault_=true;disConnect();throw;
    }
}

void TcpClient::disConnect()
{
    if (is_connected_)
    {
        is_connected_ = false;
        ::close(fd_);
        fd_ = -1;
    }
}

bool TcpClient::isConnect() const
{
    return is_connected_;
}

void TcpClient::tcpSend(const void *buf, uint32_t len)
{
    if (!is_connected_)
        throw TcpClientException("tcp is disconnected");

    const auto *tmp = (const uint8_t *)buf;
    while (len)
    {
        int err = (int)::send(fd_, tmp, len, MSG_NOSIGNAL);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::send() ") + strerror(errno));
        }
        len -= err;
        tmp += err;
    }
}

bool TcpClient::tcpRecv(void *buf, uint32_t len, uint32_t &has_read, uint32_t timeout)
{
    uint8_t *tmp = (uint8_t *)buf;
    fd_set read_fds;
    timeval tv = {0, 0};

    has_read = 0;
    while (len > 0)
    {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        int err = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" select() : ") + strerror(errno));
        }
        else if (err == 0)
        {
            return false;
        }

        err = (int)::read(fd_, tmp, len);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::read() ") + strerror(errno));
        }
        else if (err == 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" tcp server has disconnected"));
        }

        has_read += err;
        
        for (int i = 0; i < err; ++i)
        {
            if (port_ != 30004 && tmp[i] == ';')
            {
                return true;
            }
        }

        len -= err;
        tmp += err;
    }
    return true;
}

std::string TcpClient::toString()
{
    return ip_ + ":" + std::to_string(port_);
}
