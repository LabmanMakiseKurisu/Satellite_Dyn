/*
 * @Author: Amadeus
 * @Date: 2024-04-12 12:00:54
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 13:19:30
 * @FilePath: /Satellite/src/api/APIServer.hh
 * @Description: 
 */
/*
 * @Author: Amadeus
 * @Date: 2024-04-12 12:00:54
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 12:14:19
 * @FilePath: /Satellite/src/api/APIServer.hh
 * @Description: 
 */
#pragma once
#include <httplib.h>
#include <thread>

class APIServer {
public:
    APIServer();
    APIServer(const std::string &host, int port);
    ~APIServer();
    void run();
    void stop();
private:
    std::string host_;
    int port_;
    std::unique_ptr<httplib::Server> svr_; // 使用智能指针管理Server实例
    std::thread thread_;
};
