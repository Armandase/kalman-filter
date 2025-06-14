#include "../inc/UdpClient.h"


UdpClient::UdpClient()
{
    this->m_sockfd = -1;
    this->m_buffer = "";
    this->m_port = -1;
    memset(&m_servaddr, 0, sizeof(m_servaddr));
    m_address = "127.0.0.1";
}

UdpClient::~UdpClient()
{
    if (this->m_port != -1)
        close(this->m_port);
}

int    UdpClient::initializeClient(int port, std::string addr)
{
    // Creating socket file descriptor
    if ((this->m_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cerr << "initializeClient" << std::strerror(errno) << std::endl;
        return 1;
    }
    struct timeval tv;
    tv.tv_sec = 5; // 5 seconds timeout
    tv.tv_usec = 0;
    if (setsockopt(m_sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        std::cerr << "Error setting timeout: " << std::strerror(errno) << std::endl;
        return 1;
    }
    // Filling server information
    m_servaddr.sin_family = AF_INET;
    m_servaddr.sin_port = htons(port);
    m_servaddr.sin_addr.s_addr = inet_addr(addr.c_str());
    this->m_port = port;
    this->m_address = addr;
    return 0;
}

int UdpClient::sendMessage(std::string msg)
{
    std::cout << "Sending message: " << msg << std::endl;
    if (sendto(m_sockfd, (const char *)msg.c_str(), msg.length(),
        0, (const struct sockaddr *)&m_servaddr,
        sizeof(m_servaddr))  == -1 )
    {
        std::cerr << "Send Message " << std::strerror(errno) << std::endl;
        return 1;
    }
    std::cout << "Message sent" << std::endl;
    return 0;
}

int UdpClient::readMessage()
{
    socklen_t len = sizeof(m_servaddr);

    char tmp_buffer[BUFFER_SIZE];
    memset(&tmp_buffer, 0, sizeof(tmp_buffer));

    int totalBytesRead = recvfrom(m_sockfd, (char *)tmp_buffer, BUFFER_SIZE,
        0, (struct sockaddr *)&m_servaddr, &len);
    // (socklen_t *)&len);
    if (totalBytesRead < 0 )
    {
        std::cerr << "ReadMessage " << std::strerror(errno) << std::endl;
        return 1;
    }
    tmp_buffer[totalBytesRead] = '\0';
    this->m_buffer = std::string(tmp_buffer);
    return 0;
}

std::string UdpClient::getBuffer()
{
    return (this->m_buffer);
}
