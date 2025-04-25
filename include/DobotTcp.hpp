#ifndef DOBOTTCP_HPP
#define DOBOTTCP_HPP

#include <QObject>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <regex>
#include <iostream>
#include <algorithm>

#include "Dashboard.h"
#include "Feedback.h"
#include "ErrorInfoBean.h"
#include "ErrorInfoHelper.h"

class DobotTcp : public QObject {
Q_OBJECT
public:
    explicit DobotTcp(QObject *parent = nullptr);

    ~DobotTcp() override;

    void setInitState();

    int setModbusStation();

    void getRobotPose(int user, int tool, float &x, float &y, float &z, float &rx, float &ry, float &rz);

    void closeModbusStation(int index);

    void setEffectorRegs(int index, int addr, int data, int ms = 1000);

    int getEffectorRegs(int index, int addr);

    void moveRobotOrigin();

    void moveRobotSecond();

    void moveRobotEnd();

    void moveRobotCustom(std::vector<double> xyzrpy, int v = 100);

    void moveRobotCustomOffsetZ(std::vector<double> xyzrpy, double offset = 0, int v = 100);

    void moveRobotRel(double x, double y, double z, double rx, double ry, double rz);

    void moveRobotRelJ6(double deg);

    void getRobotJoint(double &j1, double &j2, double &j3, double &j4, double &j5, double &j6);

    void waitMilliSeconds(double x);

    void testEffector();

    void pauseRobot();

    void continueRobot();

    void stopRobot();

    void resetRobot();

public:
    int index;   // modbus端口
    bool isRobotMode11{false};

private:
    void getFeedBackInfo();

    void moveArriveFinish(int currentCommandID);

    std::vector<std::string> regexRecv(std::string getRecvInfo);

    void clearRobotError();

    void getCurrentCommandID(std::string recvData, int &currentCommandID);

    void parsePoseData(const std::string &recvData, float &x, float &y, float &z, float &rx, float &ry, float &rz);

private:
    Dobot::CDashboard m_Dashboard;
    Dobot::CFeedback m_CFeedback;
    Dobot::CFeedbackData feedbackData;
    Dobot::CErrorInfoBeans m_ErrorInfoBeans;
    Dobot::CErrorInfoHelper m_CErrorInfoHelper;
    bool isStateFinish{false};
    std::thread threadGetFeedBackInfo;
    std::thread threadClearRobotError;
    std::mutex m_mutexValue;
    std::mutex m_mutexState;
    bool finishState{false};

    void log(const std::string &message) {
        Q_EMIT logMessage(QString::fromStdString(message));
    }

Q_SIGNALS:

    void logMessage(const QString &msg);
};

#endif // DOBOTTCP_HPP