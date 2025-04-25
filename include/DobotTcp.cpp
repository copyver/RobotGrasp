
#include "DobotTcp.hpp"
#include <fstream>
#include <future>
#include <chrono>

DobotTcp::DobotTcp(QObject *parent)
        : QObject(parent), index{0} {
    std::string robotIp = "192.168.5.1";
    unsigned int controlPort = 29999;
    unsigned int feedPort = 30004;

    auto futureControl = std::async(std::launch::async, [&]() {
        return m_Dashboard.Connect(robotIp, controlPort);
    });
    if (futureControl.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        if (futureControl.get()) {
            qDebug("机器人端口29999已连接");
        } else {
            qDebug("机器人端口29999连接失败");
            return;
        }
    } else {
        qDebug("机器人端口29999连接超时");
        return;
    }

    auto futureFeed = std::async(std::launch::async, [&]() {
        return m_CFeedback.Connect(robotIp, feedPort);
    });
    if (futureFeed.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        if (futureFeed.get()) {
            qDebug("机器人端口30004已连接");
        } else {
            qDebug("机器人端口30004连接失败");
            return;
        }
    } else {
        qDebug("机器人端口30004连接超时");
        return;
    }
    m_CErrorInfoHelper.ParseControllerJsonFile("../alarmController.json");
    m_CErrorInfoHelper.ParseServoJsonFile("../alarmServo.json");
    threadGetFeedBackInfo = std::thread(&DobotTcp::getFeedBackInfo, this);
    threadGetFeedBackInfo.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threadClearRobotError = std::thread(&DobotTcp::clearRobotError, this);
    threadClearRobotError.detach();
}

DobotTcp::~DobotTcp() {
    closeModbusStation(index);
    m_Dashboard.Disconnect();
    m_CFeedback.Disconnect();
    if (threadGetFeedBackInfo.joinable())
        threadGetFeedBackInfo.join();
    if (threadClearRobotError.joinable())
        threadClearRobotError.join();
}


void DobotTcp::getCurrentCommandID(std::string recvData, int &currentCommandID) {
    log("recvData: " + recvData);
    currentCommandID = 2147483647;    // 初始值  int-max
    if (recvData.find("device does not connected") != std::string::npos) {
        log("device does not connected");
        return;
    }

    if (recvData.find("send error") != std::string::npos) {
        log("device does not connected");
        return;
    }

    // recvData 为 0,{2},MovJ(joint={-90, 20, 0, 0, 0, 0})     vecRecv为所有数字的集合 [ 0,2,-90, 20, 0, 0, 0, 0]
    std::vector<std::string> vecRecv = regexRecv(recvData);

    // vecRecv[0]为指令是否下发成功   vecRecv[1]为返回运动指令currentCommandID
    if (vecRecv.size() >= 2U && std::stoi(vecRecv[0]) == 0) {
        currentCommandID = std::stoi(vecRecv[1]);
    }
}

void DobotTcp::getFeedBackInfo() {
    std::cout << "Start GetFeedBackInfo" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            feedbackData = m_CFeedback.GetFeedbackData();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DobotTcp::moveArriveFinish(int currentCommandID) {
    std::cout << "Wait moveArriveFinish" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            if (feedbackData.CurrentCommandId > currentCommandID) {
                break;
            }
            if (feedbackData.CurrentCommandId == currentCommandID && feedbackData.RobotMode == 5) {
                break;
            }
        }

        {
            std::unique_lock<std::mutex> lockValue(m_mutexState);
            if (finishState) {
                finishState = false;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};

std::vector<std::string> DobotTcp::regexRecv(std::string getRecvInfo) {
    std::regex pattern("-?\\d+");
    std::smatch matches;
    std::string::const_iterator searchStart(getRecvInfo.cbegin());
    std::vector<std::string> vecErrorId;
    while (std::regex_search(searchStart, getRecvInfo.cend(), matches, pattern)) {
        for (auto &match: matches) {
            vecErrorId.push_back(match.str());
        }
        searchStart = matches.suffix().first;
    }
    return vecErrorId;
};

void DobotTcp::clearRobotError() {
    std::cout << "Start CheckRobotError" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            if (feedbackData.ErrorStatus) {
                std::vector<std::string> errorIdVec = regexRecv(m_Dashboard.GetErrorID());
                for (int i = 1; i < errorIdVec.size(); i++) {
                    Dobot::CErrorInfoBean beanController;
                    Dobot::CErrorInfoBean beanServo;
                    if (std::stoi(errorIdVec[i]) != 0) {
                        printf("告警码：%s\n", errorIdVec[i].c_str());
                        if (m_CErrorInfoHelper.FindController(std::stoi(errorIdVec[i]), beanController)) {
                            printf("控制器告警：%d, 告警原因：%s,%s\n", beanController.id,
                                   beanController.zh_CN.description.c_str(), beanController.en.description.c_str());
                        } else {
                            if (m_CErrorInfoHelper.FindServo(std::stoi(errorIdVec[i]), beanServo)) {
                                printf("伺服告警：%d,告警原因：%s, %s\n", beanServo.id,
                                       beanServo.zh_CN.description.c_str(), beanServo.en.description.c_str());
                            }
                        }
                    }
                }
                char choose[50] = {""};
                std::cout << "输入1, 将清除错误, 机器继续运行:" << std::endl;
                std::cin >> choose;
                std::cout << "您的选择： " << choose << std::endl;
                try {
                    int result = std::stoi(choose);
                    if (result == 1) {
                        std::cout << "清除错误，机器继续运行！" << std::endl;
                        m_Dashboard.ClearError();
                    }
                } catch (const std::exception &e) {
                    std::cerr << "Exception caught: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "Unknown exception caught." << std::endl;
                }
            } else {
                if (feedbackData.RobotMode == 11) {
//                    std::cout << "机器发生碰撞 " << std::endl;
//                    char choose[50] = {""};
//                    std::cout << "输入1, 将清除碰撞, 机器继续运行: " << std::endl;
//                    std::cin >> choose;
//                    std::cout << "您的选择： " << choose << std::endl;
//                    try {
//                        int result = std::stoi(choose);
//                        if (result == 1) {
//                            std::cout << "清除错误，机器继续运行！" << std::endl;
//                            m_Dashboard.ClearError();
//                        }
//                    } catch (const std::exception &e) {
//                        std::cerr << "Exception caught: " << e.what() << std::endl;
//                    } catch (...) {
//                        std::cerr << "Unknown exception caught." << std::endl;
//                    }
                    std::cout << "机器发生碰撞 " << std::endl;
                    try {
                        m_Dashboard.ClearError();
                        std::cout << "清除错误，机器继续运行！" << std::endl;
                        isRobotMode11 = true;
                    } catch (const std::exception &e) {
                        std::cerr << "Exception caught: " << e.what() << std::endl;
                    } catch (...) {
                        std::cerr << "Unknown exception caught." << std::endl;
                    }
                }
                if (!feedbackData.EnableStatus) {
//                    std::cout << "机器未使能 " << std::endl;
//                    char choose[50] = {""};
//                    std::cout << "输入1, 机器将使能: " << std::endl;
//                    std::cin >> choose;
//                    std::cout << "您的选择： " << choose << std::endl;
//                    try {
//                        int result = std::stoi(choose);
//                        if (result == 1) {
//                            std::cout << "机器使能！" << std::endl;
//                            m_Dashboard.EnableRobot();
//                        }
//                    } catch (const std::exception &e) {
//                        std::cerr << "Exception caught: " << e.what() << std::endl;
//                    } catch (...) {
//                        std::cerr << "Unknown exception caught." << std::endl;
//                    }
                    std::cout << "机器未使能 " << std::endl;
                    try {
                        m_Dashboard.EnableRobot();
                        std::cout << "机器使能！" << std::endl;
                    } catch (const std::exception &e) {
                        std::cerr << "Exception caught: " << e.what() << std::endl;
                    } catch (...) {
                        std::cerr << "Unknown exception caught." << std::endl;
                    }
                }

                if (!feedbackData.ErrorStatus && feedbackData.EnableStatus && feedbackData.RobotMode == 5) {
                    std::unique_lock<std::mutex> lockValue(m_mutexState);
                    finishState = true;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}

void DobotTcp::pauseRobot() {
    int cID = 0;
    getCurrentCommandID(m_Dashboard.Pause(), cID);
}

void DobotTcp::setInitState() {
    int cID = 0;
    getCurrentCommandID(m_Dashboard.ClearError(), cID);
    getCurrentCommandID(m_Dashboard.PowerOn(), cID);
    getCurrentCommandID(m_Dashboard.EnableRobot(), cID);
    getCurrentCommandID(m_Dashboard.SpeedFactor(15), cID);
    getCurrentCommandID(m_Dashboard.CP(10), cID);
    getCurrentCommandID(m_Dashboard.User(0), cID);
    getCurrentCommandID(m_Dashboard.Tool(3), cID);
}

void DobotTcp::moveRobotCustom(std::vector<double> xyzrpy, int v) {
    if (xyzrpy.size() < 6) {
        throw std::runtime_error("Input pose must have at least 6 doubles.");
    }

    Dobot::CDescartesPoint point{};

    std::memcpy(&point, xyzrpy.data(), 6 * sizeof(double));
    std::string v_str = "v=" + std::to_string(v);

    int cID = 0;
    getCurrentCommandID(m_Dashboard.MovJ(point, v_str), cID);
    moveArriveFinish(cID);
}


void DobotTcp::moveRobotCustomOffsetZ(std::vector<double> xyzrpy, double offset, int v) {
    if (xyzrpy.size() < 6) {
        throw std::runtime_error("Input pose must have at least 6 doubles.");
    }

    Dobot::CDescartesPoint point{};

    std::memcpy(&point, xyzrpy.data(), 6 * sizeof(double));
    point.z += offset;
    std::string v_str = "v=" + std::to_string(v);

    int cID = 0;
    getCurrentCommandID(m_Dashboard.MovJ(point, v_str), cID);
    moveArriveFinish(cID);
}


void DobotTcp::moveRobotOrigin() {
    double xyzrpy[6] = {120, 430, 225, -179, 0, 0};
    Dobot::CDescartesPoint point{};

    std::memcpy(&point, xyzrpy, sizeof(point));
    int cID = 0;
    getCurrentCommandID(m_Dashboard.MovJ(point), cID);
    moveArriveFinish(cID);
}

void DobotTcp::moveRobotSecond() {
    double xyzrpy[6] = {440.27, 70.73, 225, -179, 0, -65};
    Dobot::CDescartesPoint point{};

    std::memcpy(&point, xyzrpy, sizeof(point));
    int cID = 0;
    getCurrentCommandID(m_Dashboard.MovJ(point), cID);
    moveArriveFinish(cID);

}

void DobotTcp::moveRobotEnd() {
    double xyzrpy[6] = {120, 430, 175, -179, 0, 0};
    Dobot::CDescartesPoint point{};

    std::memcpy(&point, xyzrpy, sizeof(point));
    int cID = 0;
    getCurrentCommandID(m_Dashboard.MovJ(point), cID);
    moveArriveFinish(cID);

}


void DobotTcp::moveRobotRel(double x, double y, double z, double rx, double ry, double rz) {
    double offset[6] = {x, y, z, rx, ry, rz};
    Dobot::CDescartesPoint offset_point{};
    std::memcpy(&offset_point, offset, sizeof(offset_point));
    int cID = 0;
    getCurrentCommandID(m_Dashboard.RelMovJUser(offset_point), cID);
    moveArriveFinish(cID);
}

void DobotTcp::getRobotJoint(double &j1, double &j2, double &j3, double &j4, double &j5, double &j6) {
    std::vector<std::string> vecRecv = regexRecv(m_Dashboard.GetAngle());
    if (vecRecv.size() < 7) {
        std::cerr << "Error: Received angle data is incomplete!" << std::endl;
        return;
    }
    try {
        // vecRecv 中的第2到第7个数字对应 j1 ~ j6（索引 1 到 6）
        j1 = std::stod(vecRecv[1]);
        j2 = std::stod(vecRecv[3]);
        j3 = std::stod(vecRecv[5]);
        j4 = std::stod(vecRecv[7]);
        j5 = std::stod(vecRecv[9]);
        j6 = std::stod(vecRecv[11]);
    }
    catch (const std::invalid_argument &e) {
        std::cerr << "Conversion error: invalid argument. " << e.what() << std::endl;
    }
    catch (const std::out_of_range &e) {
        std::cerr << "Conversion error: value out of range. " << e.what() << std::endl;
    }

}

void DobotTcp::testEffector() {
    int cID = 0;
    getCurrentCommandID(m_Dashboard.SetTool485(115200, "N", 1), cID);
    getCurrentCommandID(m_Dashboard.ModbusRTUCreate(1, 115200, "N", 8, 1), cID);
    int index = cID;
    getCurrentCommandID(m_Dashboard.GetHoldRegs(index, 40000, 1, "U16"), cID);
    getCurrentCommandID(m_Dashboard.SetHoldRegs(index, 40000, 1, "{0}", "U16"), cID);
    getCurrentCommandID(m_Dashboard.ModbusClose(index), cID);
}

void DobotTcp::moveRobotRelJ6(double deg) {
    int cID = 0;
    Dobot::CJointPoint pt{0, 0, 0, 0, 0, deg};
    getCurrentCommandID(m_Dashboard.RelJointMovJ(pt), cID);

}

int DobotTcp::setModbusStation() {
    int cID = 0;
    getCurrentCommandID(m_Dashboard.SetTool485(115200, "N", 1), cID);
    for (int i = 0; i < 5; ++i) {
        closeModbusStation(i);
    }
    getCurrentCommandID(m_Dashboard.ModbusRTUCreate(1, 115200, "N", 8, 1), cID);
    index = cID;
    return cID;
}

void DobotTcp::closeModbusStation(int index) {
    int cID = 0;
    getCurrentCommandID(m_Dashboard.ModbusClose(index), cID);
}

void DobotTcp::setEffectorRegs(int index, int addr, int data, int ms) {
    int cID = 0;
    std::string formattedData = "{" + std::to_string(data) + "}";
    getCurrentCommandID(m_Dashboard.SetHoldRegs(index, addr, 1, formattedData, "U16"), cID);
    waitMilliSeconds(ms);
}

void DobotTcp::waitMilliSeconds(double x) {
    std::chrono::duration<double, std::milli> delay(x);
    std::this_thread::sleep_for(delay);
}

int DobotTcp::getEffectorRegs(int index, int addr) {
    int cID;
    getCurrentCommandID(m_Dashboard.GetHoldRegs(index, addr, 1, "U16"), cID);
    return cID;

}

void DobotTcp::getRobotPose(int user, int tool, float &x, float &y, float &z, float &rx, float &ry, float &rz) {
    std::string recvData = m_Dashboard.GetPose(user, tool);
    if (recvData.find("device does not connected") != std::string::npos) {
        std::cout << "device does not connected " << std::endl;
        return;
    }

    if (recvData.find("send error") != std::string::npos) {
        std::cout << "send error" << std::endl;
        return;
    }
    parsePoseData(recvData, x, y, z, rx, ry, rz);
}

void DobotTcp::continueRobot() {
    int cID;
    getCurrentCommandID(m_Dashboard.Continue(), cID);
}

void DobotTcp::stopRobot() {
    int cID;
    getCurrentCommandID(m_Dashboard.Stop(), cID);
}

void DobotTcp::resetRobot() {
    int cID;
    moveRobotSecond();
    moveRobotOrigin();
    setEffectorRegs(index, 40000, 100, 1000);
    setEffectorRegs(index, 40000, 0, 0);
}

void
DobotTcp::parsePoseData(const std::string &recvData, float &x, float &y, float &z, float &rx, float &ry, float &rz) {
    // 查找包含位姿数据的子字符串
    size_t startPos = recvData.find("{");
    size_t endPos = recvData.find("}");

    if (startPos != std::string::npos && endPos != std::string::npos) {
        // 提取位姿数据部分
        std::string poseData = recvData.substr(startPos + 1, endPos - startPos - 1);

        // 使用字符串流分割数据
        std::stringstream ss(poseData);
        char comma; // 用于丢弃逗号
        ss >> x >> comma >> y >> comma >> z >> comma >> rx >> comma >> ry >> comma >> rz;
    } else {
        std::cout << "Failed to find valid pose data in the string!" << std::endl;
    }

}
