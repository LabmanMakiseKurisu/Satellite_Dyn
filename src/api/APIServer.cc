#include"APIServer.hh"
#include"GlobalSetting.hh"
#include "APIHandler.hh"
// 委托构造函数
APIServer::APIServer() : APIServer(GlobalSettings::GetInstance()->Get<std::string>("/APIServer/Host"),
                                   GlobalSettings::GetInstance()->Get<int>("/APIServer/Port"))
{
}
APIServer::APIServer(const std::string &host, int port)
: host_(host), port_(port), svr_(new httplib::Server()) {

}

APIServer::~APIServer()
{
    stop();
}

void APIServer::run()
{
    thread_ = std::thread([this]()
                          {
        svr_->Options("/Data/(.+)", [](const httplib::Request& req, httplib::Response& res) {
            APIHandler::handleOptions(req, res);
        });
        svr_->Get("/Data/(.+)", [](const httplib::Request &req, httplib::Response &res) {
                APIHandler::handleGet(req, res);
            });
        svr_->Put("/Data/(.+)", [](const httplib::Request &req, httplib::Response &res) {
                APIHandler::handlePut(req, res);
            });
            std::cout << "APIServer starting at " << host_ << ":" << port_ << std::endl;
        svr_->listen(host_.c_str(), port_); });
}

void APIServer::stop()
{
    if (svr_)
    {
        svr_->stop(); // 停止服务器
        svr_.reset(); // 释放服务器实例
    }
    if (thread_.joinable())
    {
        thread_.join(); // 等待服务器线程结束
    }
}
