#include <iostream>
#include <thread>
#include <chrono>
#include <cstdlib>
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

	kf.predict(data.getAcceleration());

	if (data.isPositionEmpty() == false)
	{
		VectorXd pos = data.getPosition();
		if (pos.size() == 3)
		{
			std::cout << "Updating Kalman filter with position: " << pos.transpose() << std::endl;
			kf.update(pos);
		}
		else
		{
			std::cerr << "Position vector size is not 3, skipping update." << std::endl;
		}
	}

	VectorXd estimatedPos = kf.getPos();

	std::ofstream plotFile("estimated_positions.txt", std::ios::app);
	for (int i = 0; i < estimatedPos.size(); ++i)
	{
		plotFile << estimatedPos[i];
		if (i < estimatedPos.size() - 1)
			plotFile << " ";
	}
	plotFile << std::endl;
	plotFile.close();

	kf.plotvariance();

	std::string response = vectorToResponse(estimatedPos);
	return response;
}

int main()
{
	std::string firstMsg = "READY";
	UdpClient client;
	KalmanFilter kf(3, 6);
	DataStorage data;

	client.initializeClient();
	client.sendMessage(firstMsg);

	std::remove("estimated_positions.txt");
	std::remove("variance.txt");

	bool running = true;
	int nbResponseSend = 0;
	while (running)
	{
		if (client.readMessage())
		{
			running = false;
		}
		std::string buffer = client.getBuffer();
		if (buffer.empty())
			continue;
		data.readBuffer(buffer);
		if (buffer == "MSG_END")
		{
			std::string response = computeResponse(kf, data);
			client.sendMessage(response);

			data.clearData();
			nbResponseSend++;
			std::cout << "Response " << nbResponseSend << std::endl;
		}
	}
	return 0;
}