#include <iostream>
#include <thread>
#include <chrono>
#include "../inc/UdpClient.h"
#include "../inc/KalmanFilter.h"
#include "../inc/DataStorage.h"
#include "../inc/main.h"

#define PORT 4242
#define MAXLINE 1024

std::string computeResponse(KalmanFilter &kf, DataStorage &data)
{
    if (!kf.isInitialized())
    {
        VectorXd truePos = data.getTruePosition();
        VectorXd accel = data.getAcceleration();
        VectorXd dir = data.getDirection();
        double speed = data.getSpeedValue();
        kf.initMatrices(truePos, accel, dir, speed);
    }

    VectorXd u = computeVelocity(data.getAcceleration(), data.getDirection(), kf.getVelocity(), DT);
    kf.predict(u);

    
    if (data.isPositionEmpty() == false){
        VectorXd pos = data.getPosition();
        if (pos.size() == 3) {
            std::cout << "Updating Kalman filter with position: " << pos.transpose() << std::endl;
            kf.update(pos);
        } else {
            std::cerr << "Position vector size is not 3, skipping update." << std::endl;
        }
    }

    VectorXd estimatedPos = kf.getPos();
    std::string response = vectorToResponse(estimatedPos);
    return response;
}

int main()
{
    std::string firstMsg = "READY";
    UdpClient client;
    // KalmanFilter kf(6, 3);
    KalmanFilter kf(3, 6);
    DataStorage data;

    client.initializeClient();
    client.sendMessage(firstMsg);

    bool running = true;
    int nbResponseSend = 0;
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
        data.readBuffer(buffer);
        if (buffer == "MSG_END"){
            std::string response = computeResponse(kf, data);
            client.sendMessage(response);

            data.clearData();
            nbResponseSend++;
            std::cout << "Response " << nbResponseSend << std::endl;
        }

    }
    return 0;
}