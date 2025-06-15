#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <Eigen/Dense>

using namespace Eigen;

# define START "MSG_START"
# define END "MSG_END"

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

    private:
        std::string extractTitle(std::string &buffer);
        std::map<std::string, VectorXd> m_dataMap; // Key-value pairs for data storage
};  