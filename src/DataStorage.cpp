#include "../inc/DataStorage.h"


DataStorage::DataStorage()
{
    this->m_dataMap["TRUE POSITION"] = VectorXd::Zero(3);
    this->m_dataMap["SPEED"] = VectorXd::Zero(1);
    this->m_dataMap["ACCELERATION"] = VectorXd::Zero(3);
    this->m_dataMap["DIRECTION"] = VectorXd::Zero(3);
    this->m_dataMap["POSITION"] = VectorXd::Zero(3);
}

DataStorage::~DataStorage()
{
}

void DataStorage::clearData()
{
    for (auto &pair : m_dataMap)
    {
        pair.second.setZero(); // Reset each vector to zero
    }
}

std::string DataStorage::extractTitle(std::string &buffer)
{
    std::string title = buffer.substr(0, buffer.find("\n"));

    size_t startTimePos = buffer.find("[");
    size_t endTimePos = buffer.find("]");
    if (startTimePos != std::string::npos || endTimePos != std::string::npos || startTimePos < endTimePos)
    {
        buffer.erase(0, title.length() + 1);
        title.erase(0, endTimePos + 1);
    }
    return title;
}

void DataStorage::readBuffer(std::string &buffer)
{
    std::string title = extractTitle(buffer);
    if (title == START || title == END || title.empty())
    {
        return;
    }
    if (m_dataMap.find(title) == m_dataMap.end())
    {
        std::cerr << "Title not found in data map: " << title << std::endl;
        return;
    }

    std::string line;
    std::istringstream iss(buffer);

    int idx = 0;
    while (std::getline(iss, line))
    {
        if (line.empty())
            continue;


        double value;
        value = std::stod(line); // Assuming the line contains a single value
        if (idx < m_dataMap[title].size())
        {
            m_dataMap[title](idx) = value; // Store the value in the corresponding vector
            idx++;
        }
        else
        {
            std::cerr << "Index out of bounds for title: " << title << std::endl;
            return;
        }
    }
}


VectorXd DataStorage::getTruePosition() {
    return this->m_dataMap["TRUE POSITION"];
}

VectorXd DataStorage::getSpeed() {
    return this->m_dataMap["SPEED"];
}

double DataStorage::getSpeedValue() {
    return this->m_dataMap["SPEED"](0);
}

VectorXd DataStorage::getAcceleration() {
    return this->m_dataMap["ACCELERATION"];
}

VectorXd DataStorage::getDirection() {
    return this->m_dataMap["DIRECTION"];
}

VectorXd DataStorage::getPosition() {
    return this->m_dataMap["POSITION"];
}

bool DataStorage::isTruePosEmpty() const {
    return this->m_dataMap.at("TRUE POSITION").isZero();
}

bool DataStorage::isSpeedEmpty() const {
    return this->m_dataMap.at("SPEED").isZero();
}

bool DataStorage::isAccelEmpty() const {
    return this->m_dataMap.at("ACCELERATION").isZero();
}

bool DataStorage::isDirectionEmpty() const {
    return this->m_dataMap.at("DIRECTION").isZero();
}

bool DataStorage::isPositionEmpty() const {
    return this->m_dataMap.at("POSITION").isZero();
}

bool DataStorage::isEmpty() const {
    return isTruePosEmpty() && isSpeedEmpty() && isAccelEmpty() && isDirectionEmpty() && isPositionEmpty();
}
