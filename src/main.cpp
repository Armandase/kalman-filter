#include <iostream>
#include "../inc/UdpClient.h"
#include <thread>
#include <chrono>

#define PORT 4242
#define MAXLINE 1024

int main()
{
    std::string firstMsg = "READY";
    UdpClient client;

    client.initializeClient();
    client.sendMessage(firstMsg);

    bool running = true;
    while (running)
    {
        if (client.readMessage())
        {
            running = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::string buffer = client.getBuffer();
        if (buffer.empty())
            continue;
        std::cout << client.getBuffer() << std::endl;
        // if (buffer == "MSG_END")
            // running = false;
    }
    return 0;
}