#pragma once

#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <Eigen/Dense>
#include "main.h"

using namespace Eigen;

# define START "MSG_START"
# define END "MSG_END"


typedef struct s_point {
    double x;
    double y;
    double z;
} t_point;

class DataStorage
{
    public:
        DataStorage();
        ~DataStorage();

        void readBuffer(std::string &buffer);
        void printData() const;
        void clearData();

        VectorXd getTruePosition();
        VectorXd getSpeed();
        double getSpeedValue();
        VectorXd getAcceleration();
        VectorXd getDirection();
        VectorXd getPosition();

        bool isTruePosEmpty() const;
        bool isSpeedEmpty() const;
        bool isAccelEmpty() const;
        bool isDirectionEmpty() const;
        bool isPositionEmpty() const;
        bool isEmpty() const;

        std::vector<t_point> getHistory() const;
        void appendToHistory(double x, double y, double z);


    private:
        std::string extractTitle(std::string &buffer);
        std::map<std::string, VectorXd> m_dataMap; // Key-value pairs for data storage
        std::vector<t_point> m_history;
};  