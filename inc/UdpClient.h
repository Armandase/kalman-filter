#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <iostream>

#define BUFFER_SIZE 1024
#define TIMEOUT_SEC 5

class UdpClient
{
    public:
        UdpClient();
        ~UdpClient();

        int initializeClient(int port=4242, std::string addr="127.0.0.1");
        int sendMessage(std::string msg);
        int readMessage();

        std::string getBuffer();
    private:
        int m_sockfd;
        std::string m_buffer;
        struct sockaddr_in m_servaddr;
        int m_port;
        std::string m_address;
};  